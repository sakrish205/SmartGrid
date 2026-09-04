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
_MIN_PASS_FRACTION = 0.3   # drop passes shorter than this × spray_width_mm
_MAX_ANGLE_DEV_DEG = 40.0  # drop passes whose direction deviates more than this from the primary


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

    min_len   = spray_width_mm * _MIN_PASS_FRACTION
    cos_limit = np.cos(np.radians(_MAX_ANGLE_DEV_DEG))

    # Primary pass: longest by arc length
    lengths = [_arc_length(np.asarray(p, dtype=float)) for p in polylines]
    primary_idx = int(np.argmax(lengths))
    primary_dir = _dominant_dir(np.asarray(polylines[primary_idx], dtype=float))

    kept = []
    for i, (poly, arc_len) in enumerate(zip(polylines, lengths)):
        if i == primary_idx:
            kept.append(poly)   # always keep primary
            continue
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

        # Sort so the longest chain comes first (primary pass for this level)
        polylines.sort(key=lambda p: len(p), reverse=True)

        # Drop corner clips and misaligned diagonal fragments
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

    # Only connect the primary (sub_index == 0) passes
    primary_passes = [p for p in all_passes if p.sub_index == 0]
    connections = _connector.connect_passes(
        primary_passes, mesh_data, region_face_indices,
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
        total_passes=len(primary_passes),
        total_length_mm=total_length,
    )
