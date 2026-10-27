"""Shared curvature/smoothness formulas — relocated from app/reference/reference_metrics.py
(Mesh Flow Phase 2) so both the reference-diagnostics side (Phase 2) and the
generated-path side (Phase 4+) call one implementation instead of two that could
drift. Behavior is unchanged from the Phase 2 originals; reference_metrics.py
re-imports these names.
"""
from __future__ import annotations
import numpy as np

from app.path.resampler import resample_arc

# F13 note: "skip duplicate points" — points closer than this are treated as one.
_DUP_EPS_MM = 1e-6


def segment_directions(points: np.ndarray) -> np.ndarray:
    """F13 — d_i = normalize(q_(i+1) - q_i), duplicate points skipped."""
    diffs = np.diff(points, axis=0)
    lengths = np.linalg.norm(diffs, axis=1)
    keep = lengths > _DUP_EPS_MM
    return diffs[keep] / lengths[keep, None]


def second_differences(points: np.ndarray) -> np.ndarray:
    """F17 — delta^2 q_i = q_(i+1) - 2*q_i + q_(i-1)."""
    if len(points) < 3:
        return np.zeros((0, points.shape[1]))
    return points[2:] - 2.0 * points[1:-1] + points[:-2]


def smoothness_penalty(points: np.ndarray) -> float:
    """F18 — E_smooth = sum ||delta^2 q_i||^2."""
    d2 = second_differences(points)
    return float(np.sum(np.linalg.norm(d2, axis=1) ** 2))


def approximate_curvature(points: np.ndarray, arc_spacing_mm: float) -> np.ndarray:
    """F19 — kappa_i ~= ||delta^2 q_i|| / delta_s^2, after F38 arc-length resampling."""
    resampled = resample_arc(points, arc_spacing_mm) if len(points) >= 2 else points
    d2 = second_differences(resampled)
    return np.linalg.norm(d2, axis=1) / (arc_spacing_mm ** 2)


def curvature_change_penalty(curvatures: np.ndarray) -> float:
    """F52 — E_kappa = sum((kappa_(i+1) - kappa_i)^2), over arc-length-resampled
    curvatures (i.e. the output of approximate_curvature). Discrete curvature
    variation penalty — "use consistent sampling... do not smooth across region
    breaks" per the spec's own implementation note, so callers must pass curvatures
    from one contiguous pass/lane only, never concatenated across passes."""
    if len(curvatures) < 2:
        return 0.0
    return float(np.sum(np.diff(curvatures) ** 2))
