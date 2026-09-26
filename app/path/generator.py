"""Orchestrates: slice → stitch → filter → simplify → resample → connect."""
from __future__ import annotations
from collections import defaultdict
import logging
import numpy as np
import trimesh as _trimesh

_log = logging.getLogger(__name__)
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, PaintRoute
from app.path import slicer as _slicer
from app.path import stitcher as _stitcher
from app.path import connector as _connector
from app.path.resampler import rdp_simplify, resample_arc, prune_collinear

_RDP_EPSILON       = 0.3   # mm — remove micro-jaggies from triangle discretisation
_MIN_PASS_FRACTION = 0.10  # drop passes shorter than 10% of spray_width_mm …
_MIN_PASS_ABS_MM   = 1.0   # … absolute floor — was 5 mm, lowered so narrow tooth-tops and curved edges survive
_MAX_ANGLE_DEV_DEG = 65.0  # angle limit only for SHORT passes; long passes kept at any angle
_MAX_SUB_PER_LEVEL = 6     # max sub-index passes per slice level (prevents fragment explosion)


def _arc_length(pts: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def _dominant_dir(pts: np.ndarray) -> np.ndarray:
    """Unit vector from first to last point (global pass direction)."""
    d = pts[-1] - pts[0]
    n = np.linalg.norm(d)
    return d / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])


def _filter_polylines(
    polylines: list,
    spray_width_mm: float,
) -> list:
    """Remove corner fragments and misaligned short sub-passes.

    Keeps the primary (longest) pass unconditionally; drops any
    shorter polyline that is:
      - shorter than _MIN_PASS_FRACTION × spray_width_mm, OR
      - its start→end direction deviates > _MAX_ANGLE_DEV_DEG from
        the primary pass direction (catches diagonal corner clips).
    Genuine hole sub-passes are parallel to the primary — they survive
    both tests.
    """
    if not polylines:
        return polylines

    # Sort by arc length so primary is always index 0 — avoids primary_idx mismatch
    # when a dense short fragment has more points than the real long pass.
    lengths = [_arc_length(np.asarray(p, dtype=float)) for p in polylines]
    order   = sorted(range(len(polylines)), key=lambda i: lengths[i], reverse=True)
    polylines = [polylines[i] for i in order]
    lengths   = [lengths[i]   for i in order]

    # Minimum length: fraction of spray_width, but never drop passes above the absolute floor
    min_len   = max(spray_width_mm * _MIN_PASS_FRACTION, _MIN_PASS_ABS_MM)
    cos_limit = np.cos(np.radians(_MAX_ANGLE_DEV_DEG))
    primary_dir = _dominant_dir(np.asarray(polylines[0], dtype=float))

    kept = [polylines[0]]   # always keep primary (longest)
    for poly, arc_len in zip(polylines[1:], lengths[1:]):
        if arc_len < min_len:
            continue            # too short — corner clip
        # Only apply direction check to SHORT passes.  Long passes (≥ 4× min_len)
        # are legitimate at any angle: gear tooth tops, bumper curved edges, etc.
        # Applying the angle check unconditionally drops all perpendicular passes.
        if arc_len < min_len * 4:
            d = _dominant_dir(np.asarray(poly, dtype=float))
            if abs(np.dot(d, primary_dir)) < cos_limit:
                continue        # short AND wrong angle — diagonal corner clip
        kept.append(poly)

    return kept


def generate_route(
    mesh_data: MeshData,
    region_id: str,
    region_face_indices: np.ndarray,
    spray_width_mm: float,
    waypoint_spacing_mm: float = 0.0,   # 0 = keep raw slicer points
    direction: str = 'horizontal',
) -> PaintRoute:
    """Main entry point: given a face selection, return a complete PaintRoute."""
    if len(region_face_indices) == 0:
        raise ValueError(f"Region '{region_id}' has no classified triangles.")

    planes = _slicer.compute_slice_planes(
        mesh_data.trimesh_mesh,
        region_face_indices,
        region_id,
        mesh_data.up_axis,
        spray_width_mm,
        direction=direction,
    )

    # Slice only the region faces per plane — avoids scanning all F_total faces N times.
    # Vertices are shared (no copy); face normals are preserved.
    full_mesh = mesh_data.trimesh_mesh
    sub_mesh  = _trimesh.Trimesh(
        vertices=full_mesh.vertices,
        faces=full_mesh.faces[region_face_indices],
        process=False,
    )
    sub_all = np.arange(len(sub_mesh.faces), dtype=np.int64)

    all_passes: list[PaintPass] = []
    pass_id = 0

    for plane_index, (plane_normal, plane_origin, slice_pos) in enumerate(planes):
        segments = _slicer.slice_region(
            sub_mesh,
            sub_all,
            plane_normal,
            plane_origin,
            region_id=region_id,
            up_axis=mesh_data.up_axis,
        )
        if segments is None:
            continue

        polylines = _stitcher.stitch_segments(segments)
        if not polylines:
            continue

        # Drop corner clips, diagonal fragments, and sort by arc length
        # (_filter_polylines handles sorting internally)
        polylines = _filter_polylines(polylines, spray_width_mm)

        # Cap fragments per level: too many means edge/corner noise on complex meshes
        polylines = polylines[:_MAX_SUB_PER_LEVEL]

        # Direction alternates by plane index, not by total pass count,
        # so holes/sub-passes don't disrupt the pattern.
        is_forward = (plane_index % 2 == 0)

        for sub_idx, polyline in enumerate(polylines):
            if len(polyline) < 2:
                continue
            pts = polyline if is_forward else polyline[::-1].copy()
            # Smooth micro-jaggies from mesh triangulation, then resample uniformly
            pts = rdp_simplify(pts, _RDP_EPSILON)
            if waypoint_spacing_mm > 0 and len(pts) >= 2:
                pts = resample_arc(pts, waypoint_spacing_mm)
            pts = prune_collinear(pts)
            all_passes.append(PaintPass(
                id=pass_id,
                region_id=region_id,
                direction=direction,
                points=pts,
                is_forward=is_forward,
                sub_index=sub_idx,
                slice_position=float(slice_pos),
            ))
            pass_id += 1

    # TSP-lite: group passes by slice level; within each level pick the sub-pass
    # whose nearest endpoint is closest to the current tool position (greedy).
    _level_map: dict[float, list[PaintPass]] = defaultdict(list)
    for _p in all_passes:
        _level_map[round(_p.slice_position, 4)].append(_p)

    _sorted_passes: list[PaintPass] = []
    for _pos in sorted(_level_map.keys()):
        _group = _level_map[_pos]
        if len(_group) > 1:
            _cur = _sorted_passes[-1].points[-1] if _sorted_passes else _group[0].points[0]
            _rem = list(_group)
            _ordered: list[PaintPass] = []
            while _rem:
                _i = min(range(len(_rem)), key=lambda i: min(
                    np.linalg.norm(_rem[i].points[0] - _cur),
                    np.linalg.norm(_rem[i].points[-1] - _cur),
                ))
                _p = _rem.pop(_i)
                if np.linalg.norm(_p.points[-1] - _cur) < np.linalg.norm(_p.points[0] - _cur):
                    _p = PaintPass(id=_p.id, region_id=_p.region_id, direction=_p.direction,
                                   points=_p.points[::-1].copy(), is_forward=not _p.is_forward,
                                   sub_index=_p.sub_index, slice_position=_p.slice_position)
                _ordered.append(_p)
                _cur = _p.points[-1]
            _group = _ordered
        _sorted_passes.extend(_group)
    all_passes = _sorted_passes

    if not all_passes:
        _log.warning(
            "generate_route: region '%s' produced 0 passes — "
            "all slice planes missed the geometry. "
            "Try reducing spray_width_mm or check the up_axis setting.",
            region_id,
        )

    # Connect ALL passes in execution order (sub-index passes are real passes, not orphans)
    connections = _connector.connect_passes(
        all_passes,
        spray_width_mm=spray_width_mm,
        waypoint_spacing_mm=waypoint_spacing_mm,
    )

    total_length = sum(
        float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1)))
        for p in all_passes
        if len(p.points) >= 2
    )

    normals = mesh_data.face_normals[region_face_indices]
    mean_n = normals.mean(axis=0)
    norm = np.linalg.norm(mean_n)
    spray_normal = mean_n / norm if norm > 1e-9 else mean_n

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
