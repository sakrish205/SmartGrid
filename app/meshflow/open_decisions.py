"""Mesh Flow's open-decision strategy points (spec §12).

None of the 7 open decisions in the spec are resolved by guessing a default here.
Each becomes an explicit, swappable strategy point: exactly one real implementation
(the most conservative/explicit-setting branch) plus every other branch raising
OpenDecisionBlocked, naming which §12 bullet blocks it.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum


class OpenDecisionBlocked(NotImplementedError):
    """Raised when a code path requires a human policy decision not yet made (spec §12)."""

    def __init__(self, decision_no: int, what: str):
        super().__init__(f"blocked on open decision #{decision_no} (spec §12): {what}")
        self.decision_no = decision_no


class LanePitchStrategy(Enum):           # decision #1
    EXPLICIT_SETTING = 'explicit'        # implemented
    FROM_WIDTH_OVERLAP = 'width_overlap'     # raises OpenDecisionBlocked(1, ...)
    FROM_REFERENCE = 'reference'             # raises OpenDecisionBlocked(1, ...)


class RegionBoundaryPolicy(Enum):        # decision #2
    EXPLICIT_FACE_SELECTION = 'explicit_faces'        # implemented — reuses app.mesh.regions.classify_regions
    HOLE_AWARE = 'hole_aware'                          # raises OpenDecisionBlocked(2, ...)
    DISCONNECTED_COMPONENT_AWARE = 'component_aware'   # raises OpenDecisionBlocked(2, ...)
    PROTECTED_FEATURE_AWARE = 'feature_aware'          # raises OpenDecisionBlocked(2, ...)


@dataclass(frozen=True)
class ToolAxisConvention:
    """Decision #3 — one instance per exporter, each wrapping the convention ALREADY
    hard-coded in app/export/olp_export.py::_get_tcp_orientation and
    app/reference/reference_metrics.py::pointing_axis — named/made swappable here,
    not reimplemented.

    F30 status (decided — proceed provisional, do not block on this):
      ABB   — VALIDATED: a_hat = -n_hat confirmed against all 60 real robtargets in
              Exercise_2.mod (tz column points into the part, 60/60).
      FANUC / DURR — PROVISIONAL: not empirically verified against real tool-
              calibration data (no tPaintGun-equivalent definition available for
              these two). verified=False until real calibration data exists.
    """
    controller: str   # 'ABB' | 'FANUC' | 'DURR'
    verified: bool


ABB_TOOL_AXIS_CONVENTION = ToolAxisConvention(controller='ABB', verified=True)
FANUC_TOOL_AXIS_CONVENTION = ToolAxisConvention(controller='FANUC', verified=False)
DURR_TOOL_AXIS_CONVENTION = ToolAxisConvention(controller='DURR', verified=False)


def tool_axis_convention_for(controller: str) -> ToolAxisConvention:
    """Returns the one sanctioned ToolAxisConvention for a controller. There is no
    'unverified/guessed' 4th option — an unsupported controller name is a bug to fix,
    not a case to silently default."""
    try:
        return {
            'ABB': ABB_TOOL_AXIS_CONVENTION,
            'FANUC': FANUC_TOOL_AXIS_CONVENTION,
            'DURR': DURR_TOOL_AXIS_CONVENTION,
        }[controller]
    except KeyError:
        raise OpenDecisionBlocked(3, f"no tool-axis convention defined for controller {controller!r}")


@dataclass
class ProcessConstraints:                # decision #4 — every field None until a human sets it
    standoff_range_mm: tuple[float, float] | None = None
    max_angle_limit_deg: float | None = None
    max_speed_mmpm: float | None = None
    overlap_ratio: float | None = None
    paint_while_turning_allowed: bool | None = None


@dataclass
class AcceptanceTolerances:              # decision #5 — same None-until-set pattern
    surface_deviation_mm: float | None = None
    orientation_jump_deg: float | None = None
    collision_clearance_mm: float | None = None
    coverage_ratio_min: float | None = None
