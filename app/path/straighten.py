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

    All candidate extensions from one start point are batched into a single
    closest_point call so the cKDTree is built once per outer iteration instead
    of once per segment check.  This cuts KDTree builds from O(N) down to
    O(N/avg_seg_len) — the dominant cost on meshes with many vertices.

    ponytail: O(N²) sample-point count in the worst case (adversarial inputs
              where every extension fails immediately); typical curves from
              first_pass_line span many points per segment so total samples
              stay well below N².  Switch to an interval-tree or a per-start
              progressive batching scheme if profiler says so.
    """
    pts = np.asarray(curve_points, dtype=float)
    n = len(pts)
    if n < 3:
        return pts.copy()

    lo = standoff_mm - standoff_tol_mm
    hi = standoff_mm + standoff_tol_mm

    def _n_samples(a: np.ndarray, b: np.ndarray) -> int:
        length = float(np.linalg.norm(b - a))
        return 1 if length < 1e-9 else max(2, int(np.ceil(length / verify_sample_mm)) + 1)

    corners = [pts[0]]
    i = 0
    while i < n - 1:
        # Gather samples for every extension pts[i]→pts[i+2], pts[i]→pts[i+3], ...
        # (pts[i]→pts[i+1] is the fallback and needs no check — see original greedy logic).
        exts = range(i + 2, n)
        if not exts:
            i = n - 1
            break

        sizes = [_n_samples(pts[i], pts[j]) for j in exts]
        # build sample arrays for all extensions in one allocation
        all_samples = np.empty((sum(sizes), 3), dtype=float)
        pos = 0
        for j, ns in zip(exts, sizes):
            if ns == 1:
                all_samples[pos] = pts[i]
            else:
                all_samples[pos:pos + ns] = (
                    pts[i] + np.linspace(0.0, 1.0, ns)[:, None] * (pts[j] - pts[i])
                )
            pos += ns

        _, dists, _ = trimesh.proximity.closest_point(mesh, all_samples)

        # walk extensions greedily, stopping at the first failure
        j = i + 1   # fallback: always accept i→i+1
        pos = 0
        for k, (j2, ns) in enumerate(zip(exts, sizes)):
            end = pos + ns
            if bool(np.all((dists[pos:end] >= lo) & (dists[pos:end] <= hi))):
                j = j2
            else:
                break
            pos = end

        if j < n - 1:
            corners.append(pts[j])
        i = j

    corners.append(pts[-1])
    return np.array(corners)
