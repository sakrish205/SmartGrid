"""Mesh Flow Phase 3 — minimal core generator (R1/R3/R4/R5, F41).

Orchestrates direction_estimator -> lane_planner -> surface_intersector, then
reuses app.path.resampler (rdp_simplify/resample_arc) and the same
is_fwd = visible_count % 2 == 0 boustrophedon pattern already proven in
app.path.generator.generate_route (line 159), and (Phase 5)
app.meshflow.transition_planner.plan_transitions for air-move connectors
between passes.

Mesh Surface's generate_route() is not called, imported from, or modified by this
module — this is a fully parallel code path, selected only via path_target=='mesh_flow'.
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
from app.path.path_model import MeshFlowRegion, PaintPass, PaintRoute
from app.path.resampler import rdp_simplify, resample_arc

_RDP_EPSILON = 0.3   # mm — same constant app.path.generator uses, kept consistent


def _arc_length(pts: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def _longest_polyline(polylines: list[np.ndarray]) -> np.ndarray | None:
    """Keep the primary (longest) stitched polyline per plane — same "primary pass"
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
) -> PaintRoute:
    """R1/R3/R4/R5 — generate a boustrophedon PaintRoute for one region.

    Populates region.primary_direction/spacing_direction/lane_pitch_mm in place
    (mirroring how app.path.generator leaves PaintPass.normals/tangent for a later
    step) before generating lanes.
    """
    mesh: trimesh.Trimesh = mesh_data.trimesh_mesh

    primary_direction, spacing_direction = _direction_estimator.local_scan_frame(
        region, mesh, user_direction,
    )
    region.primary_direction = primary_direction
    region.spacing_direction = spacing_direction

    pitch_mm = _lane_planner.resolve_lane_pitch(explicit_pitch_mm, lane_pitch_strategy)
    region.lane_pitch_mm = pitch_mm

    planes = _lane_planner.generate_lane_planes(region, mesh, pitch_mm)

    passes: list[PaintPass] = []
    pass_id = 0
    visible_count = 0   # counts planes that actually produced a pass — drives alternation (F41)
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

        # F41: d_k = (-1)^k d — same visible_count-based alternation as
        # app.path.generator.generate_route (line 159), so skipped/empty planes
        # don't break the serpentine pattern.
        is_fwd = (visible_count % 2 == 0)
        visible_count += 1
        if not is_fwd:
            pts = pts[::-1]

        passes.append(PaintPass(
            id=pass_id, region_id=region.region_id,
            direction='mesh_flow', points=pts,
            is_forward=is_fwd, sub_index=0,
            slice_position=position,
            normals=None, tangent=None,
        ))
        pass_id += 1

    connections = _transition_planner.plan_transitions(
        passes, mesh, region, spray_width_mm=pitch_mm, waypoint_spacing_mm=waypoint_spacing_mm,
    )
    total_len = sum(_arc_length(p.points) for p in passes)

    mean_normal = mesh.face_normals[region.face_indices].mean(axis=0)
    mean_normal_norm = np.linalg.norm(mean_normal)
    spray_normal = mean_normal / mean_normal_norm if mean_normal_norm > 1e-9 else np.zeros(3)

    return PaintRoute(
        region_id=region.region_id, passes=passes, connections=connections,
        unit='mm', spacing_mm=pitch_mm,
        total_passes=len(passes), total_length_mm=total_len,
        spray_normal=spray_normal,
    )
