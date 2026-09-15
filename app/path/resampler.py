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


_LEAD_MM = 75.0   # ponytail: fixed lead; make a param if per-region tuning is needed


def lead_inout(points: np.ndarray, lead_mm: float = _LEAD_MM) -> np.ndarray:
    """Prepend / append a lead-in / lead-out segment along the pass direction."""
    pts = np.asarray(points, dtype=float)
    if lead_mm <= 0 or len(pts) < 2:
        return pts
    d_in = pts[0] - pts[1]; n = np.linalg.norm(d_in)
    if n > 1e-9: d_in /= n
    d_out = pts[-1] - pts[-2]; n = np.linalg.norm(d_out)
    if n > 1e-9: d_out /= n
    return np.vstack([pts[0] + d_in * lead_mm, pts, pts[-1] + d_out * lead_mm])


def prune_collinear(points: np.ndarray, angle_tol_deg: float = 0.5) -> np.ndarray:
    """Drop intermediate waypoints where direction change is below angle_tol_deg."""
    pts = np.asarray(points, dtype=float)
    if len(pts) <= 2:
        return pts
    cos_tol = np.cos(np.radians(angle_tol_deg))
    keep = np.ones(len(pts), dtype=bool)
    for i in range(1, len(pts) - 1):
        d1 = pts[i] - pts[i - 1]; n1 = np.linalg.norm(d1)
        d2 = pts[i + 1] - pts[i]; n2 = np.linalg.norm(d2)
        if n1 > 1e-9 and n2 > 1e-9 and np.dot(d1 / n1, d2 / n2) > cos_tol:
            keep[i] = False
    return pts[keep]
