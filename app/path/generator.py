"""Orchestrates: slice → stitch → filter → simplify → resample → connect."""
from __future__ import annotations
from collections import defaultdict
import logging
import numpy as np
import trimesh

_log = logging.getLogger(__name__)
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, PaintRoute
from app.path import stitcher as _stitcher
from app.path import connector as _connector
from app.path.resampler import rdp_simplify, resample_arc, prune_collinear
from app.path.face_grid_generator import _compute_surface_basis, _get_basis_faces
from app.path.local_normals import interpolate_normals, build_tcp_frames

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
    waypoint_spacing_mm: float = 0.0,
    direction: str = 'horizontal',
) -> PaintRoute:
    """Mesh Surface: delegates to Conform logic with standoff=0."""
    from app.path.face_grid_generator import generate_conform_route
    return generate_conform_route(
        region=region_id,
        face_indices=region_face_indices,
        mesh=mesh_data.trimesh_mesh,
        up_axis=mesh_data.up_axis,
        spray_width_mm=spray_width_mm,
        waypoint_spacing_mm=waypoint_spacing_mm,
        standoff_mm=0.0,
        direction=direction,
    )
