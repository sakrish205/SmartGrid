"""Geodesic iso-contour toolpath generation for Mesh Surface mode.

Replaces the slice-plane approach for Mesh Surface mode when a robot profile
is active. Uses potpourri3d heat-method distance field to guarantee even
surface-distance pitch regardless of face angle.

Pipeline:
  submesh → seed verts → geodesic dist field → iso-contour segments
  → stitch_segments (reused) → resample → per-point normals → PaintRoute
"""
from __future__ import annotations
import logging
import numpy as np
import trimesh

from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, PaintRoute
from app.path import stitcher as _stitcher
from app.path import connector as _connector
from app.path.resampler import rdp_simplify, resample_arc, prune_collinear
from app.robot.robot_profile import RobotProfile

_log = logging.getLogger(__name__)
_RDP_EPSILON = 0.3


def _isocontour(
    mesh: trimesh.Trimesh,
    dist: np.ndarray,          # per-vertex scalar (geodesic distance)
    level: float,
) -> np.ndarray | None:
    """Marching-triangles iso-contour at `level`.

    Returns (K, 2, 3) segments or None if no crossing found.
    Identical output shape to slice_region() so stitch_segments() is reused.
    """
    faces = mesh.faces            # (F, 3)
    verts = mesh.vertices         # (V, 3)
    d = dist[faces]               # (F, 3) — scalar at each face vertex

    # Which faces cross the level? At least one vertex above AND one below.
    above = d > level             # (F, 3)
    n_above = above.sum(axis=1)   # (F,)
    crossing = (n_above > 0) & (n_above < 3)

    if not crossing.any():
        return None

    cf = faces[crossing]          # (K, 3)
    cd = d[crossing]              # (K, 3)

    segments = []
    for tri, scalars in zip(cf, cd):
        pts_on_edge = []
        for i in range(3):
            j = (i + 1) % 3
            a, b = scalars[i], scalars[j]
            if (a - level) * (b - level) < 0:
                t = (level - a) / (b - a)
                pt = verts[tri[i]] + t * (verts[tri[j]] - verts[tri[i]])
                pts_on_edge.append(pt)
        if len(pts_on_edge) == 2:
            segments.append(pts_on_edge)

    if not segments:
        return None
    return np.array(segments, dtype=float)   # (K, 2, 3)


def _find_seed_vertices(
    sub_mesh: trimesh.Trimesh,
    sweep_axis: int,
    step: float,
) -> np.ndarray:
    """Vertices within step/4 of the minimum coordinate along sweep_axis."""
    coords = sub_mesh.vertices[:, sweep_axis]
    threshold = coords.min() + step * 0.25
    idx = np.where(coords <= threshold)[0]
    if len(idx) == 0:
        idx = np.array([int(np.argmin(coords))])
    return idx


def _per_point_normals(
    sub_mesh: trimesh.Trimesh,
    region_face_indices: np.ndarray,
    mesh_data: MeshData,
    pts: np.ndarray,
) -> np.ndarray:
    """Per-waypoint surface normal via nearest face on the submesh.

    sub_mesh.faces[i] corresponds to mesh_data face region_face_indices[i],
    so we can look up the original face normal for correct orientation.
    """
    if len(pts) == 0:
        return np.zeros((0, 3), dtype=float)
    _, _, face_ids = trimesh.proximity.closest_point(sub_mesh, pts)
    # face_ids are indices into sub_mesh.faces; map back to original mesh
    original_ids = region_face_indices[face_ids]
    normals = mesh_data.face_normals[original_ids].copy()
    # Normalise (face_normals should already be unit, but be safe)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms = np.where(norms < 1e-9, 1.0, norms)
    return normals / norms


def generate_geodesic_route(
    mesh_data: MeshData,
    region_face_indices: np.ndarray,
    region_id: str,
    spray_width_mm: float,
    robot_profile: RobotProfile,
    waypoint_spacing_mm: float = 0.0,
    direction: str = 'horizontal',
) -> PaintRoute:
    """Generate a geodesic iso-contour toolpath for the given region."""
    try:
        import potpourri3d as pp3d
    except ImportError as exc:
        raise ImportError(
            'potpourri3d is required for geodesic path generation. '
            'Install it with: pip install potpourri3d'
        ) from exc

    if len(region_face_indices) == 0:
        raise ValueError(f"Region '{region_id}' has no classified triangles.")

    # Build submesh (shared vertices, only region faces)
    full = mesh_data.trimesh_mesh
    sub_mesh = trimesh.Trimesh(
        vertices=full.vertices,
        faces=full.faces[region_face_indices],
        process=False,
    )

    # Determine sweep axis — same logic as slicer.compute_slice_config
    up = mesh_data.up_axis
    fwd = (up + 1) % 3
    mean_n = np.abs(mesh_data.face_normals[region_face_indices]).mean(axis=0)
    if region_id in ('TOP', 'BOTTOM'):
        sweep_axis = fwd
    elif region_id in ('FRONT', 'REAR', 'LEFT', 'RIGHT'):
        sweep_axis = up
    else:
        dominant = int(np.argmax(mean_n))
        sweep_axis = fwd if dominant == up else up
    if direction == 'vertical':
        fn_axis = int(np.argmax(mean_n))
        sweep_axis = 3 - fn_axis - sweep_axis

    # Step = spray width × 0.85 × (1 - overlap%)
    step = spray_width_mm * 0.85 * (1.0 - robot_profile.overlap_factor_pct / 100.0)
    step = max(step, 1.0)  # guard against zero/negative

    # Geodesic distance field from seed edge
    seed_verts = _find_seed_vertices(sub_mesh, sweep_axis, step)
    _log.debug('geodesic: region=%s  seed_verts=%d', region_id, len(seed_verts))

    solver = pp3d.MeshHeatMethodDistanceSolver(sub_mesh.vertices, sub_mesh.faces)
    dist = solver.compute_distance_multisource(seed_verts.tolist())

    d_max = float(dist.max())
    if d_max < step:
        _log.warning('geodesic: region %s geodesic span %.1f mm < step %.1f mm', region_id, d_max, step)

    pass_levels = np.arange(step / 2.0, d_max, step)
    _log.debug('geodesic: %d pass levels over %.1f mm', len(pass_levels), d_max)

    all_passes: list[PaintPass] = []
    pass_id = 0

    for i, level in enumerate(pass_levels):
        segments = _isocontour(sub_mesh, dist, float(level))
        if segments is None:
            continue
        polylines = _stitcher.stitch_segments(segments)
        if not polylines:
            continue

        # Sort by arc length — longest first (reuse _filter_polylines concept)
        lengths = [float(np.sum(np.linalg.norm(np.diff(np.asarray(p), axis=0), axis=1))) for p in polylines]
        order = sorted(range(len(polylines)), key=lambda k: lengths[k], reverse=True)
        polylines = [polylines[k] for k in order]

        is_forward = (i % 2 == 0)

        for sub_idx, poly in enumerate(polylines):
            if len(poly) < 2:
                continue
            pts = np.asarray(poly, dtype=float)
            if not is_forward:
                pts = pts[::-1].copy()
            pts = rdp_simplify(pts, _RDP_EPSILON)
            if waypoint_spacing_mm > 0 and len(pts) >= 2:
                pts = resample_arc(pts, waypoint_spacing_mm)
            pts = prune_collinear(pts)
            if len(pts) < 2:
                continue
            normals = _per_point_normals(sub_mesh, region_face_indices, mesh_data, pts)
            all_passes.append(PaintPass(
                id=pass_id,
                region_id=region_id,
                direction=direction,
                points=pts,
                is_forward=is_forward,
                sub_index=sub_idx,
                slice_position=float(level),
                normals=normals,
            ))
            pass_id += 1

    if not all_passes:
        _log.warning('geodesic: region %s produced 0 passes', region_id)

    connections = _connector.connect_passes(
        all_passes,
        spray_width_mm=spray_width_mm,
        waypoint_spacing_mm=waypoint_spacing_mm,
    )

    total_length = sum(
        float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1)))
        for p in all_passes if len(p.points) >= 2
    )

    spray_normal = mesh_data.face_normals[region_face_indices].mean(axis=0)
    n = np.linalg.norm(spray_normal)
    spray_normal = spray_normal / n if n > 1e-9 else spray_normal

    return PaintRoute(
        region_id=region_id,
        passes=all_passes,
        connections=connections,
        unit='mm',
        spacing_mm=spray_width_mm,
        total_passes=len(all_passes),
        total_length_mm=total_length,
        spray_normal=spray_normal,
    )
