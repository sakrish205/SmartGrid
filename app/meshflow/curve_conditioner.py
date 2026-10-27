"""Mesh Flow Phase 4 — curve conditioner (R6).

F38 (arc-length resampling) via app.path.resampler.resample_arc (reused, not
reimplemented), then a smoothing pass bounded by F22 (point-to-mesh deviation):
smoothing a waypoint toward the local average of its neighbors is only accepted
if the result stays within max_surface_deviation_mm of the mesh; otherwise that
waypoint is left unsmoothed rather than silently exceeding the bound.
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.resampler import resample_arc

_SMOOTH_WEIGHT = 0.5   # how far each waypoint moves toward its neighbor average per pass
_MAX_ITERS = 3


def resample_and_smooth(
    points: np.ndarray,
    max_surface_deviation_mm: float,
    mesh: trimesh.Trimesh,
    arc_spacing_mm: float,
) -> np.ndarray:
    """R6 — F38 resample, then bounded smoothing (F22 dist-to-mesh constraint).

    max_surface_deviation_mm should be MeshQualityReport.planning_time_budget_mm's
    result when simplification was used for planning (never the raw estimate —
    see app.path.mesh_quality.simplification_deviation), or the direct
    AcceptanceTolerances.surface_deviation_mm otherwise. This function does not
    know which source it was given; the caller is responsible for passing the
    correct bound (see Phase 4's plan writeup on the triangle-inequality budget).
    """
    pts = resample_arc(np.asarray(points, dtype=float), arc_spacing_mm)
    if len(pts) < 3:
        return pts

    smoothed = pts.copy()
    for _ in range(_MAX_ITERS):
        neighbor_avg = smoothed.copy()
        neighbor_avg[1:-1] = (smoothed[:-2] + smoothed[2:]) / 2.0
        candidate = smoothed + _SMOOTH_WEIGHT * (neighbor_avg - smoothed)
        candidate[0] = pts[0]     # endpoints never move
        candidate[-1] = pts[-1]

        _closest, dists, _tri = mesh.nearest.on_surface(candidate)
        within_bound = dists <= max_surface_deviation_mm
        # Only accept the smoothed position where it stays within the F22 bound;
        # waypoints that would exceed it keep their previous (less-smoothed) position.
        smoothed = np.where(within_bound[:, None], candidate, smoothed)

    return smoothed
