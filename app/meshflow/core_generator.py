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
from app.path import connector as _connector
from app.path import local_normals as _local_normals
from app.path.path_model import MeshFlowRegion, PaintPass, PaintRoute
from app.path.resampler import rdp_simplify, resample_arc

_RDP_EPSILON = 0.3   # mm — same constant app.path.generator uses, kept consistent


def _arc_length(pts: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


_MIN_CHAIN_LENGTH_FRACTION = 0.1   # vs. this plane's own longest chain — see _significant_polylines


def _significant_polylines(polylines: list[np.ndarray], cover_all: bool = False) -> list[np.ndarray]:
    """Keep every stitched chain substantial enough to be a real pass on this plane,
    not just the single longest.

    A region's real surface can legitimately be more than one connected panel
    (region_manager._significant_connected_components keeps all of them, e.g.
    TOP's hood/roof/trunk) — a single lane plane can cross more than one of those
    panels and produce more than one real intersection chain. Picking only the
    longest (the original "primary pass" behaviour, same concept as
    app.path.generator._filter_polylines) silently dropped every other panel's
    coverage at that plane position, which is what produced the sparse/missing
    top coverage once the region-membership fix restored the full panel set.
    Still filters out true noise (a decoy part's tiny intersection, a sliver at
    a panel's very edge) relative to this plane's own longest chain, same
    relative-cutoff logic as the region-level filter."""
    candidates = [np.asarray(p, dtype=float) for p in polylines if len(p) >= 2]
    if not candidates:
        return []
    if cover_all:
        return candidates
    lengths = [_arc_length(p) for p in candidates]
    cutoff = max(lengths) * _MIN_CHAIN_LENGTH_FRACTION
    return [p for p, length in zip(candidates, lengths) if length >= cutoff]




def _region_components(mesh: trimesh.Trimesh, face_indices: np.ndarray) -> list[np.ndarray]:
    """Split region.face_indices (region_manager has already dropped noise/junk
    components) into its connected surface patches, largest first.

    A region's real surface is often more than one physical panel (hood/roof/
    trunk, mirrored left/right pairs — see
    region_manager._significant_connected_components). Computing ONE scan
    direction for all of them blended together is wrong whenever they're
    tilted differently: a plain vector mean cancels out opposite-signed
    components on mirrored pairs (confirmed on Part1.STL — e.g. normals
    (-0.54,0.09,0.82) and (0.54,0.09,0.82) average to roughly (0,0.09,0.82),
    which matches NEITHER panel), leaving lane planes oriented for no real
    surface and producing sparse/missing coverage on exactly the panels that
    got cancelled out of the average. Running direction_estimator/lane_planner
    per component instead of once per region (see generate_mesh_flow_route)
    fixes that with the exact same formulas, just at the right granularity.
    """
    if len(face_indices) <= 1:
        return [face_indices]
    adjacency = mesh.face_adjacency
    in_region = np.isin(adjacency, face_indices).all(axis=1)
    components = trimesh.graph.connected_components(
        adjacency[in_region], nodes=face_indices, min_len=1,
    )
    if len(components) <= 1:
        return [face_indices]
    areas = mesh.area_faces
    return sorted(components, key=lambda c: -areas[c].sum())


def generate_mesh_flow_route(
    mesh_data: MeshData,
    region: MeshFlowRegion,
    lane_pitch_strategy: LanePitchStrategy,
    explicit_pitch_mm: float | None,
    waypoint_spacing_mm: float = 0.0,
    user_direction: np.ndarray | None = None,
    standoff_mm: float = 0.0,
    cover_all: bool = False,
) -> PaintRoute:
    """R1/R3/R4/R5 — generate a boustrophedon PaintRoute for one region.

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
    body panel) — restricting the lookup to this region's own submesh makes
    that structurally impossible. On top of both, every offset point is
    checked with a LOCAL signed-distance test against its own nearest
    triangle on the region submesh (same pattern app.path.face_grid_generator
    uses for Conform/Adaptive's _clip_inside) — unlike a global
    mesh.contains() check, this needs no watertightness. BOUNDED guarantee,
    not absolute: on flat geometry, offsetting straight along a wrong normal
    and re-querying "nearest local triangle" almost always re-finds the SAME
    triangle, so there is no local disagreement available to detect a
    uniformly-wrong region with — this check is a real, useful supplement on
    curved/partially-wrong geometry (confirmed empirically: corrects the
    large majority of a partially-flipped curved region), never a substitute
    for Fix A's load-time, whole-mesh winding repair, which is what actually
    has to catch a uniformly-wrong flat region.
    """
    mesh: trimesh.Trimesh = mesh_data.trimesh_mesh

    components = _region_components(mesh, region.face_indices)

    # The largest component's frame is both (a) the region-level summary other
    # code reads (region.primary_direction/spacing_direction — e.g. viewer
    # chevrons) and (b) the fallback for components too small to fit their own
    # stable direction (see _MIN_COMPONENT_FACES_FOR_OWN_FRAME).
    largest_direction, largest_spacing = _direction_estimator.local_scan_frame(
        MeshFlowRegion(region_id=region.region_id, face_indices=components[0]),
        mesh, user_direction,
    )
    region.primary_direction = largest_direction
    region.spacing_direction = largest_spacing

    # Outward normal derived from the LARGEST component only, not all region faces.
    # Interior faces (cabin floor, cavity walls) contaminate the whole-region mean
    # and can flip it, causing the hemisphere filter to invert and drop exterior faces.
    # The largest component is the dominant exterior panel — its mean is reliable.
    largest_mean_n = mesh.face_normals[components[0]].mean(axis=0)
    largest_mean_n_norm = np.linalg.norm(largest_mean_n)
    spray_normal = largest_mean_n / largest_mean_n_norm if largest_mean_n_norm > 1e-9 else np.zeros(3)

    pitch_mm = _lane_planner.resolve_lane_pitch(explicit_pitch_mm, lane_pitch_strategy)
    region.lane_pitch_mm = pitch_mm

    # Built once per region, reused by every pass's standoff offset below —
    # restricts the nearest-triangle lookup to THIS region's own geometry so
    # it can never return an unrelated part's normal on a multi-body mesh.
    region_mesh = mesh.submesh([region.face_indices], append=True) if standoff_mm > 0.0 else None

    passes: list[PaintPass] = []
    pass_id = 0
    visible_count = 0   # counts planes that actually produced a pass — drives alternation (F41)

    for component_faces in components:
        # Drop faces pointing strongly away from the region's outward direction —
        # true interior surfaces (cavity walls, underside panels) have normals at
        # 120°–180° from the outward direction (dot < -0.5). Edge/transition faces
        # (door frames, window sills) are at ~90° (dot ≈ 0) and must be kept.
        # Threshold -0.5 removes only genuinely inward-pointing faces, not edges.
        outward_mask = mesh.face_normals[component_faces] @ spray_normal > -0.5
        component_faces = component_faces[outward_mask]
        if len(component_faces) == 0:
            continue

        # All components share the largest component's scan frame so every pass
        # in the region is parallel — same angle, same spacing direction.
        # Per-component frames caused angle drift between panels (issue 3).
        d_hat, s_hat = largest_direction, largest_spacing

        sub_region = MeshFlowRegion(
            region_id=region.region_id, face_indices=component_faces,
            primary_direction=d_hat, spacing_direction=s_hat,
        )
        planes = _lane_planner.generate_lane_planes(sub_region, mesh, pitch_mm)

        for plane_normal, plane_origin, position in planes:
            polylines = _surface_intersector.intersect_region(mesh, sub_region, plane_normal, plane_origin)
            chains = _significant_polylines(polylines, cover_all=cover_all)
            if not chains:
                continue

            for sub_index, pts in enumerate(chains):
                pts = rdp_simplify(pts, _RDP_EPSILON)
                if waypoint_spacing_mm > 0.0:
                    pts = resample_arc(pts, waypoint_spacing_mm)
                if len(pts) < 2:
                    continue

                # F41: d_k = (-1)^k d — same visible_count-based alternation as
                # app.path.generator.generate_route (line 159), so skipped/empty planes
                # don't break the serpentine pattern. Each chain on a plane (one panel's
                # worth of coverage) advances it independently of its sibling chains.
                is_fwd = (visible_count % 2 == 0)
                visible_count += 1
                if not is_fwd:
                    pts = pts[::-1]

                pass_normals = None
                pass_tangent = None
                if standoff_mm > 0.0:
                    pass_normals = _local_normals.interpolate_normals(pts, region_mesh)
                    offset_pts = pts + pass_normals * standoff_mm

                    # Local signed-distance check (watertightness-independent): is the
                    # offset point on the correct side of its OWN nearest triangle on
                    # the region submesh? Negative means it went the wrong way.
                    closest, _dist, tri_ids = region_mesh.nearest.on_surface(offset_pts)
                    local_face_n = region_mesh.face_normals[tri_ids]
                    signed = np.einsum('ij,ij->i', offset_pts - closest, local_face_n)
                    wrong_side = signed < 0
                    if wrong_side.any():
                        pass_normals[wrong_side] *= -1.0
                        offset_pts[wrong_side] = pts[wrong_side] + pass_normals[wrong_side] * standoff_mm

                    pts = offset_pts
                    pass_tangent = _local_normals.build_tcp_frames(pts, pass_normals)

                passes.append(PaintPass(
                    id=pass_id, region_id=region.region_id,
                    direction='mesh_flow', points=pts,
                    is_forward=is_fwd, sub_index=sub_index,
                    slice_position=position,
                    normals=pass_normals, tangent=pass_tangent,
                ))
                pass_id += 1

    # Generation order is "all of component A's passes, then component B's" —
    # nothing to do with which panel is actually nearest in space. Reorder by a
    # greedy nearest-neighbor walk BEFORE building connectors, and keep that
    # same order for the final PaintRoute.passes, since the viewer/exporters
    # walk route.passes in list order and look up each pass's outgoing
    # connector by id — the two have to agree, or output lands out of sequence.
    passes = _connector.reorder_passes_nearest_neighbor(passes)

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
