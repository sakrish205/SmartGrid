"""Polyline utilities: RDP simplification and uniform arc-length resampling."""
from __future__ import annotations
import numpy as np


def rdp_simplify(points: np.ndarray, epsilon: float) -> np.ndarray:
    """Iterative Ramer-Douglas-Peucker simplification.

    Removes points that deviate less than epsilon from the straight line
    between their neighbours. Keeps first and last point always.
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) <= 2 or epsilon <= 0:
        return pts

    n = len(pts)
    keep = np.ones(n, dtype=bool)
    stack = [(0, n - 1)]

    while stack:
        start, end = stack.pop()
        if end - start < 2:
            continue

        seg_vec = pts[end] - pts[start]
        seg_len = np.linalg.norm(seg_vec)

        if seg_len < 1e-12:
            keep[start + 1:end] = False
            continue

        seg_unit = seg_vec / seg_len
        vecs  = pts[start + 1:end] - pts[start]
        proj  = np.dot(vecs, seg_unit)
        perp  = vecs - np.outer(proj, seg_unit)
        dists = np.linalg.norm(perp, axis=1)

        max_local = int(np.argmax(dists))
        if dists[max_local] > epsilon:
            split = start + 1 + max_local
            stack.append((start, split))
            stack.append((split, end))
        else:
            keep[start + 1:end] = False

    return pts[keep]


def resample_arc(points: np.ndarray, spacing: float) -> np.ndarray:
    """Resample a polyline at uniform arc-length spacing.

    Always includes the exact start and end point.
    Returns at least 2 points even if the path is shorter than spacing.
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return pts

    seg_lens = np.linalg.norm(np.diff(pts, axis=0), axis=1)   # (N-1,)
    cum      = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total    = cum[-1]

    if total < 1e-9:
        return pts[[0, -1]]

    n_pts    = max(2, int(np.round(total / spacing)) + 1)
    sample_s = np.linspace(0.0, total, n_pts)

    # Vectorised: np.interp handles each axis independently
    result = np.column_stack([
        np.interp(sample_s, cum, pts[:, ax]) for ax in range(3)
    ])

    # Pin exact endpoints — avoids floating-point drift at boundaries
    result[0]  = pts[0]
    result[-1] = pts[-1]
    return result
