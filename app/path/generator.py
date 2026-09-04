"""Orchestrates: slice → stitch → filter → simplify → resample → connect."""
from __future__ import annotations
import numpy as np
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, PaintRoute
from app.path import slicer as _slicer
from app.path import stitcher as _stitcher
from app.path import connector as _connector
from app.path.resampler import rdp_simplify, resample_arc

_RDP_EPSILON       = 0.3   # mm — remove micro-jaggies from triangle discretisation
_MIN_PASS_FRACTION = 0.15  # drop passes shorter than this × spray_width_mm …
_MIN_PASS_ABS_MM   = 5.0   # … but never drop passes longer than this absolute floor
_MAX_ANGLE_DEV_DEG = 55.0  # drop passes whose direction deviates more than this from the primary


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
        d = _dominant_dir(np.asarray(poly, dtype=float))
        if abs(np.dot(d, primary_dir)) < cos_limit:
            continue            # wrong angle — diagonal fragment
        kept.append(poly)

    return kept


def generate_route(
    mesh_data: MeshData,
    region_id: str,
    region_face_indices: np.ndarray,
    spray_width_mm: float,
    waypoint_spacing_mm: float = 0.0,   # 0 = keep raw slicer points
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
    )

    all_passes: list[PaintPass] = []
    pass_id = 0

    for plane_index, (plane_normal, plane_origin, slice_pos) in enumerate(planes):
        segments = _slicer.slice_region(
            mesh_data.trimesh_mesh,
            region_face_indices,
            plane_normal,
            plane_origin,
        )
        if segments is None:
            continue

        polylines = _stitcher.stitch_segments(segments)
        if not polylines:
            continue

        # Drop corner clips, diagonal fragments, and sort by arc length
        # (_filter_polylines handles sorting internally)
        polylines = _filter_polylines(polylines, spray_width_mm)

        # Direction alternates by plane index, not by total pass count,
        # so holes/sub-passes don't disrupt the pattern.
        is_forward = (plane_index % 2 == 0)

        for sub_idx, polyline in enumerate(polylines):
            if len(polyline) < 2:
                continue
            pts = polyline if is_forward else polyline[::-1]
            # Smooth micro-jaggies from mesh triangulation, then resample uniformly
            pts = rdp_simplify(pts, _RDP_EPSILON)
            if waypoint_spacing_mm > 0 and len(pts) >= 2:
                pts = resample_arc(pts, waypoint_spacing_mm)
            all_passes.append(PaintPass(
                id=pass_id,
                region_id=region_id,
                direction='horizontal',
                points=pts,
                is_forward=is_forward,
                sub_index=sub_idx,
                slice_position=float(slice_pos),
            ))
            pass_id += 1

    # Connect ALL passes in execution order (sub-index passes are real passes, not orphans)
    connections = _connector.connect_passes(
        all_passes, mesh_data, region_face_indices,
        simplify_epsilon=1.0,
    )

    total_length = sum(
        float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1)))
        for p in all_passes
        if len(p.points) >= 2
    )

    return PaintRoute(
        region_id=region_id,
        passes=all_passes,
        connections=connections,
        unit='mm',
        spacing_mm=spray_width_mm,
        total_passes=len(all_passes),
        total_length_mm=total_length,
    )
