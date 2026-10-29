"""Mesh Flow Phase 3 ΓÇö minimal core generator (R1/R3/R4/R5, F41).

Orchestrates direction_estimator -> lane_planner -> surface_intersector, then
reuses app.path.resampler (rdp_simplify/resample_arc) and the same
is_fwd = visible_count % 2 == 0 boustrophedon pattern already proven in
app.path.generator.generate_route (line 159), and (Phase 5)
app.meshflow.transition_planner.plan_transitions for air-move connectors
between passes.

Mesh Surface's generate_route() is not called, imported from, or modified by this
module ΓÇö this is a fully parallel code path, selected only via path_target=='mesh_flow'.
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.mesh.preprocessor import MeshData
from app.meshflow import direction_estimator as _direction_estimator
from app.meshflow import lane_planner as _lane_planner
from app.meshflow import surface_intersector as _surface_intersector
from app.meshflow import transition_planner as _transition_planner
from app.meshflow.open_decisions import LanePitchStrategy
from app.path import local_normals as _local_normals
from app.path import surface_safety as _surface_safety
from app.path.path_model import MeshFlowRegion, PaintPass, PaintRoute
from app.path.resampler import rdp_simplify, resample_arc

_RDP_EPSILON = 0.3   # mm ΓÇö same constant app.path.generator uses, kept consistent


def _arc_length(pts: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def _longest_polyline(polylines: list[np.ndarray]) -> np.ndarray | None:
    """Keep the primary (longest) stitched polyline per plane ΓÇö same "primary pass"
    concept as app.path.generator._filter_polylines, reimplemented minimally here
    rather than reaching into that module's private helper, to keep Mesh Flow fully
    decoupled from Mesh Surface's code path."""
    candidates = [np.asarray(p, dtype=float) for p in polylines if len(p) >= 2]
    if not candidates:
        return None
    return max(candidates, key=_arc_length)


def generate_mesh_flow_route(
    mesh_data: MeshData,
    region: MeshFlowRegion,
    lane_pitch_strategy: LanePitchStrategy,
    explicit_pitch_mm: float | None,
    waypoint_spacing_mm: float = 0.0,
    user_direction: np.ndarray | None = None,
    standoff_mm: float = 0.0,
) -> PaintRoute:
    """R1/R3/R4/R5 ΓÇö generate a boustrophedon PaintRoute for one region.

    Populates region.primary_direction/spacing_direction/lane_pitch_mm in place
    (mirroring how app.path.generator leaves PaintPass.normals/tangent for a later
    step) before generating lanes.

    standoff_mm (F59/C1): when > 0, every pass point is lifted off the surface
    along ITS OWN nearest-triangle normal, looked up against a SUBMESH of
    just this region's own faces (never the full mesh) via
    app.path.local_normals.interpolate_normals. Two independent, diagnosed
    failure modes made a simpler offset wrong on real-world meshes: (1) a
    single region-mean normal is wrong wherever the true local normal
    diverges from the average on curved geometry; (2) looking up the nearest
    triangle against the FULL mesh can return an unrelated nearby part's
    normal on a multi-body assembly (mirrors/handles/trim sitting close to a
    body panel) ΓÇö restricting the lookup to this region's own submesh makes
    that structurally impossible. On top of both, when the full mesh is
    watertight, any point that still ends up on the wrong side is caught via
    app.path.surface_safety.inside_mesh_mask (F57) and its normal is flipped
    ΓÇö a self-verifying result, not just a formula trusted to be right.
    """
    mesh: trimesh.Trimesh = mesh_data.trimesh_mesh

    primary_direction, spacing_direction = _direction_estimator.local_scan_frame(
        region, mesh, user_direction,
    )
    region.primary_direction = primary_direction
    region.spacing_direction = spacing_direction

    mean_normal = mesh.face_normals[region.face_indices].mean(axis=0)
    mean_normal_norm = np.linalg.norm(mean_normal)
    spray_normal = mean_normal / mean_normal_norm if mean_normal_norm > 1e-9 else np.zeros(3)

    pitch_mm = _lane_planner.resolve_lane_pitch(explicit_pitch_mm, lane_pitch_strategy)
    region.lane_pitch_mm = pitch_mm

    planes = _lane_planner.generate_lane_planes(region, mesh, pitch_mm)

    # Built once per region, reused by every pass's standoff offset below ΓÇö
    # restricts the nearest-triangle lookup to THIS region's own geometry so
    # it can never return an unrelated part's normal on a multi-body mesh.
    region_mesh = mesh.submesh([region.face_indices], append=True) if standoff_mm > 0.0 else None

    passes: list[PaintPass] = []
    pass_id = 0
    visible_count = 0   # counts planes that actually produced a pass ΓÇö drives alternation (F41)
    for plane_normal, plane_origin, position in planes:
        polylines = _surface_intersector.intersect_region(mesh, region, plane_normal, plane_origin)
        pts = _longest_polyline(polylines)
        if pts is None:
            continue

        pts = rdp_simplify(pts, _RDP_EPSILON)
        if waypoint_spacing_mm > 0.0:
            pts = resample_arc(pts, waypoint_spacing_mm)
        if len(pts) < 2:
            continue

        # F41: d_k = (-1)^k d ΓÇö same visible_count-based alternation as
        # app.path.generator.generate_route (line 159), so skipped/empty planes
        # don't break the serpentine pattern.
        is_fwd = (visible_count % 2 == 0)
        visible_count += 1
        if not is_fwd:
            pts = pts[::-1]

        pass_normals = None
        pass_tangent = None
        if standoff_mm > 0.0:
            pass_normals = _local_normals.interpolate_normals(pts, region_mesh)
            if mesh.is_watertight:
                offset_preview = pts + pass_normals * standoff_mm
                wrong_side = _surface_safety.inside_mesh_mask(offset_preview, mesh)
                if wrong_side is not None and wrong_side.any():
                    pass_normals[wrong_side] *= -1.0
            pts = pts + pass_normals * standoff_mm
            pass_tangent = _local_normals.build_tcp_frames(pts, pass_normals)

        passes.append(PaintPass(
            id=pass_id, region_id=region.region_id,
            direction='mesh_flow', points=pts,
            is_forward=is_fwd, sub_index=0,
            slice_position=position,
            normals=pass_normals, tangent=pass_tangent,
        ))
        pass_id += 1

    connections = _transition_planner.plan_transitions(
        passes, mesh, region, spray_width_mm=pitch_mm, waypoint_spacing_mm=waypoint_spacing_mm,
    )
    total_len = sum(_arc_length(p.points) for p in passes)

    return PaintRoute(
        region_id=region.region_id, passes=passes, connections=connections,
        unit='mm', spacing_mm=pitch_mm,
        total_passes=len(passes), total_length_mm=total_len,
        spray_normal=spray_normal,
    )
