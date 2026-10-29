"""Mesh Flow Phase 3 — lane planner (R4).

Formulas (quoted exactly from the spec, per the plan's formula-review gate):
  F11. Scan-plane equation: s_hat . (x - p0) = k*Delta
  F12. Number of planes: N = floor((s_max - s_min)/Delta) + 1
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.meshflow.open_decisions import LanePitchStrategy, OpenDecisionBlocked
from app.path.path_model import MeshFlowRegion


def resolve_lane_pitch(
    explicit_mm: float | None,
    strategy: LanePitchStrategy,
    width_overlap: tuple[float, float] | None = None,
    reference_pitch_mm: float | None = None,
) -> float:
    """R4 — only EXPLICIT_SETTING is implemented (decision #1)."""
    if strategy is LanePitchStrategy.EXPLICIT_SETTING:
        if explicit_mm is None or explicit_mm <= 0:
            raise ValueError("LanePitchStrategy.EXPLICIT_SETTING requires a positive explicit_mm value")
        return float(explicit_mm)
    if strategy is LanePitchStrategy.FROM_WIDTH_OVERLAP:
        raise OpenDecisionBlocked(1, "lane pitch from spray width/overlap (F40) not implemented")
    if strategy is LanePitchStrategy.FROM_REFERENCE:
        raise OpenDecisionBlocked(1, "lane pitch from measured reference pitch (F39) not implemented")
    raise AssertionError(f"unhandled LanePitchStrategy: {strategy}")


def generate_lane_planes(
    region: MeshFlowRegion,
    mesh: trimesh.Trimesh,
    pitch_mm: float,
) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """F11/F12 — a family of parallel planes spaced uniformly at pitch_mm along the
    region's own spacing direction (s_hat), spanning the region's extent along that
    direction. Spacing is constant across the region — no curvature adaptation —
    so the generated path always follows the configured spray width.

    Same plane-family math as app.path.generator._build_planes_combined (lines 59-95),
    generalized from a fixed world axis to the region's own (possibly non-axis-aligned)
    direction from direction_estimator.local_scan_frame — a new function, since the
    existing one stays axis-locked and untouched.
    """
    if region.spacing_direction is None or region.primary_direction is None:
        raise ValueError(
            f"region {region.region_id!r}: primary_direction/spacing_direction must be set "
            f"(see app.meshflow.direction_estimator.local_scan_frame) before planning lanes"
        )
    if pitch_mm <= 0:
        raise ValueError("pitch_mm must be positive")

    s_hat = region.spacing_direction
    face_centroids = mesh.triangles_center[region.face_indices]
    p0 = face_centroids.mean(axis=0)

    s_coords = (face_centroids - p0) @ s_hat
    s_min = float(s_coords.min())
    s_max = float(s_coords.max())
    span = s_max - s_min

    n = int(np.floor(span / pitch_mm)) + 1   # F12
    n = max(1, n)
    step = span / n
    first = s_min + step / 2.0

    planes: list[tuple[np.ndarray, np.ndarray, float]] = []
    for k in range(n):
        pos = first + k * step
        origin = p0 + pos * s_hat
        planes.append((s_hat.copy(), origin, pos))

    return planes
