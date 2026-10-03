"""Greedy standoff-band straightener for toolpath passes."""
from __future__ import annotations

import numpy as np
import trimesh


def straighten_pass(
    mesh: trimesh.Trimesh,
    curve_points: np.ndarray,
    standoff_mm: float,
    standoff_tol_mm: float = 5.0,
    verify_sample_mm: float = 5.0,
) -> np.ndarray:
    """Reduce waypoints while staying within ±standoff_tol_mm of the mesh.

    Greedy: start at i=0, extend to the farthest j where every sampled point
    along the straight segment i→j is within [standoff_mm - tol, standoff_mm + tol]
    of the mesh surface.  Endpoints are always preserved.

    ponytail: O(N²) worst case on adversarial inputs; in practice each segment
              spans many source points so iterations are far fewer than N.
              Switch to an interval tree if profiler says so.
    """
    pts = np.asarray(curve_points, dtype=float)
    n = len(pts)
    if n < 3:
        return pts.copy()

    lo = standoff_mm - standoff_tol_mm
    hi = standoff_mm + standoff_tol_mm

    def _ok(a: np.ndarray, b: np.ndarray) -> bool:
        length = float(np.linalg.norm(b - a))
        if length < 1e-9:
            return True
        n_s = max(2, int(np.ceil(length / verify_sample_mm)) + 1)
        samples = a + np.linspace(0.0, 1.0, n_s)[:, None] * (b - a)
        _, dists, _ = trimesh.proximity.closest_point(mesh, samples)
        return bool(np.all((dists >= lo) & (dists <= hi)))

    corners = [pts[0]]
    i = 0
    while i < n - 1:
        j = i + 1
        while j + 1 <= n - 1 and _ok(pts[i], pts[j + 1]):
            j += 1
        if j < n - 1:
            corners.append(pts[j])
        i = j
    corners.append(pts[-1])
    return np.array(corners)
