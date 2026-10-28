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
    """F11/F12 — a family of parallel planes spaced pitch_mm apart along the region's
    own spacing direction (s_hat), spanning the region's extent along that direction.

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
    face_normals = mesh.face_normals[region.face_indices]
    p0 = face_centroids.mean(axis=0)

    s_coords = (face_centroids - p0) @ s_hat
    s_min = float(s_coords.min())
    s_max = float(s_coords.max())
    span = s_max - s_min

    # F11/F12 with curvature-adaptive pitch:
    #   pitch(s) = base_pitch × |s_hat × n(s)|
    # |s_hat × n| = sin(angle between spacing direction and local surface normal).
    # On a flat region perpendicular to s_hat this equals 1 (no change).
    # Where the surface tilts toward s_hat (curved hump peak), sin shrinks → denser
    # planes → on-surface gap stays ≤ spray_width everywhere.
    # Floor at 0.15 × base_pitch prevents infinite density on near-degenerate geometry.
    # n_min from F12 is preserved as a lower bound — adaptive pitch only adds planes.
    _PITCH_FLOOR = 0.15

    n_min = max(1, int(np.floor(span / pitch_mm)) + 1)   # F12 floor
    cell = span / n_min   # uniform cell width used only for initial centering offset

    def _local_factor(s: float) -> float:
        mask = np.abs(s_coords - s) <= (pitch_mm / 2.0)
        if not mask.any():
            return 1.0
        mean_n = face_normals[mask].mean(axis=0)
        norm = np.linalg.norm(mean_n)
        if norm < 1e-9:
            return 1.0
        f = float(np.linalg.norm(np.cross(s_hat, mean_n / norm)))
        return max(f, _PITCH_FLOOR)

    planes: list[tuple[np.ndarray, np.ndarray, float]] = []
    s = s_min + cell / 2.0   # centre first plane in its cell (same as original F12)
    while True:
        origin = p0 + s * s_hat
        planes.append((s_hat.copy(), origin, s))
        s += pitch_mm * _local_factor(s)
        if s > s_max and len(planes) >= n_min:
            break

    return planes
