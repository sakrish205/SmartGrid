from __future__ import annotations
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
import numpy as np

UNIT_TO_MM: dict[str, float] = {
    'mm': 1.0,
    'cm': 10.0,
    'm':  1000.0,
    'in': 25.4,
    'ft': 304.8,
}


_SOFTWARE = 'SmartGrid 1.5'


@dataclass
class GenerationParams:
    """All settings used for one path generation run — included in every export for repeatability."""
    source_file:         str        # mesh filename (basename only)
    path_mode:           str        # 'Boundary Box' | 'Face Grid / Adaptive' | 'Face Grid / Conform'
                                     # | 'Mesh Surface' | 'Mesh Flow'
    regions:             list       # e.g. ['TOP', 'FRONT']
    up_axis:             str        # 'X' | 'Y' | 'Z'
    spray_width_mm:      float
    standoff_mm:         float      # 0.0 = none applied
    waypoint_spacing_mm: float      # 0.0 = disabled (uniform mesh vertices used as-is)
    direction:           str        # 'horizontal' | 'vertical' | 'both'
    sweep:               str        # 'CW' | 'CCW'
    paint_speed_mmpm:    float      = 1000.0   # robot TCP speed during spray passes (mm/min)
    generated_at:        str = field(default_factory=lambda: datetime.now().isoformat(timespec='seconds'))
    software:            str = field(default=_SOFTWARE)


@dataclass
class PaintPass:
    id: int
    region_id: str
    direction: str        # "horizontal" — extensible to "vertical" later
    points: np.ndarray    # shape (N, 3), world coordinates in mm
    is_forward: bool      # True = first stitched direction, False = reversed
    sub_index: int        # 0 = primary chain, >0 = additional chain at same level (holes)
    slice_position: float # coordinate along slice axis, for debugging
    normals: np.ndarray | None = None   # (N, 3) per-waypoint surface normals (Z-axis of TCP frame)
    tangent: np.ndarray | None = None   # (N, 3) per-waypoint corrected path tangent (X-axis of TCP frame)
    # Adapted from Noether createTransform: B = cross(N, T_raw), tangent = cross(B, N)
    standoff_deviation_mm: float | None = None  # max |actual_dist_to_surface - nominal_standoff| for this pass
    # Mesh Flow Phase 5 — None = Mesh Surface behavior unchanged (R8). See ProcessState below.
    process_state: 'ProcessState | None' = None


@dataclass
class Connection:
    id: int
    from_pass_id: int
    to_pass_id: int
    points: np.ndarray    # shape (M, 3), ordered points along the actual mesh boundary
    is_air_move: bool     # False for valid MVP boundary connectors
    # Mesh Flow Phase 5 — default preserves the old is_air_move-only meaning (R8).
    # field(default_factory=...) instead of a plain class-level default so this
    # doesn't require MotionSegmentKind to be defined earlier in this module.
    kind: 'MotionSegmentKind' = field(default_factory=lambda: MotionSegmentKind.LANE_CONNECTOR)
    # None = Mesh Surface behavior unchanged. Added alongside .kind (not only on
    # PaintPass as first drafted) so a connector can carry its own gun-off state
    # explicitly rather than relying on the absence of a PaintPass's process_state.
    process_state: 'ProcessState | None' = None


@dataclass
class PaintRoute:
    region_id: str
    passes: list          # list[PaintPass]
    connections: list     # list[Connection]
    unit: str             # always 'mm' internally
    spacing_mm: float
    total_passes: int
    total_length_mm: float
    spray_normal: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Unit vector: spray gun approach direction perpendicular to the surface plane.


# ---------------------------------------------------------------------------
# Mesh Flow — Phase 3 (see convertmdfiles/MeshFlow_Phase0_Rename_Audit.md and the
# Mesh Flow implementation plan). Additive only; Mesh Surface's PaintPass/Connection/
# PaintRoute above are untouched by these additions.
# ---------------------------------------------------------------------------

class RegionConfidence(Enum):
    ACCEPTED = 'accepted'
    FLAGGED = 'flagged'     # usable but needs review
    REJECTED = 'rejected'   # excluded from export (spec §15.1 "missing/ambiguous" row)


@dataclass
class MeshFlowRegion:
    """Mesh Flow's region-manager record (R1) — one per region passed to the planner."""
    region_id: str
    face_indices: np.ndarray
    primary_direction: np.ndarray | None = None    # F06/F07 — None until estimated
    spacing_direction: np.ndarray | None = None     # s_hat, F07
    lane_pitch_mm: float | None = None              # resolved via LanePitchStrategy
    confidence: RegionConfidence = RegionConfidence.FLAGGED
    confidence_reasons: list[str] = field(default_factory=list)   # spec §15.2 "traceable evidence map"


# ---------------------------------------------------------------------------
# Mesh Flow — Phase 4. New side-channel reports, not new PaintPass fields (quality
# reports are keyed by pass_id so the Phase 0 baseline fixture, which serializes
# PaintPass fields, stays valid). PaintPass.normals/.tangent (already-existing
# optional fields above) ARE populated for Mesh Flow routes in this phase, reusing
# the same fields Mesh Surface's _offset_route_by_standoff already populates
# conditionally — no new fields needed for that part.
# ---------------------------------------------------------------------------

@dataclass
class MeshQualityReport:        # F48/F49 — per-region mesh-quality diagnostics (R6 gate)
    region_id: str
    edge_length_cv: float               # F48
    mean_normal_variation_deg: float     # F49
    simplification_deviation_fwd_estimate_mm: float | None = None   # raw sampled max — NOT guaranteed
    simplification_deviation_rev_estimate_mm: float | None = None   # raw sampled max — NOT guaranteed
    simplification_deviation_fwd_bound_mm: float | None = None      # estimate + covering radius —
    simplification_deviation_rev_bound_mm: float | None = None      # a REAL guaranteed upper bound

    def planning_time_budget_mm(self, surface_deviation_tolerance_mm: float) -> float | None:
        """Triangle-inequality bound, built on the GUARANTEED bound, never the raw
        estimate: for a point p on the generated path and q its closest point on
        M_s, dist(p, M_original) <= dist(p, M_s) + dist(q, M_original) <=
        dist(p, M_s) + simplification_deviation_rev_bound_mm. So to guarantee
        dist(p, M_original) <= surface_deviation_tolerance_mm, Phase 4's smoothing
        pass must bound dist(p, M_s) by (tolerance - rev_bound_mm). Returns None if
        the bound is unknown, or non-positive if no budget remains."""
        if self.simplification_deviation_rev_bound_mm is None:
            return None
        return surface_deviation_tolerance_mm - self.simplification_deviation_rev_bound_mm


@dataclass
class PathQualityReport:        # F14-F23, F51-F54 — kept SEPARATE per component (C5: no opaque score)
    pass_id: int
    straightness_mse: float               # F16/F51
    max_line_deviation_mm: float          # F15
    smoothness_penalty: float             # F18
    max_curvature: float                  # F19
    curvature_change_penalty: float       # F52
    surface_rms_mm: float                 # F42 — new, confirmed gap
    surface_max_mm: float                 # F43 — reuses validator.py closest_point pattern
    orientation_max_jump_deg: float | None = None  # F45


# ---------------------------------------------------------------------------
# Mesh Flow — Phase 5 (R8/R9). Additive: Connection.kind and PaintPass.process_state
# above default to values that preserve the old Mesh Surface meaning exactly.
# ---------------------------------------------------------------------------

class MotionSegmentKind(Enum):
    PAINT = 'paint'
    APPROACH = 'approach'
    RETRACT = 'retract'
    LANE_CONNECTOR = 'lane_connector'
    REGION_TRANSIT = 'region_transit'


@dataclass
class ProcessState:
    """Planning-side twin of app.reference.reference_model.ProcessState (Phase 1) —
    that one is parse-only/read-only; this one is used generatively by Mesh Flow's
    process_state_planner (Phase 5)."""
    gun_on: bool = False
    setbrush_index: int | None = None
    speed_mmpm: float | None = None
    external_axes: dict | None = None   # R9 — opaque passthrough, same convention as Phase 1's IR


# ---------------------------------------------------------------------------
# Mesh Flow — Phase 6 (R10, §18, §20.1/§20.2). Separate report dataclasses, not
# opaque fields on PaintRoute — §20.2 C7 requires target-region membership,
# surface-deviation, inside/outside, and collision to stay INDEPENDENT checks;
# bundling into one pass/fail field would violate C5/C7's "no opaque score".
# ---------------------------------------------------------------------------

class ValidationStatus(Enum):
    PASSED = 'passed'          # every mandatory check ran with a real tolerance/verified premise, none failed
    BLOCKED = 'blocked'        # at least one 'hard' finding — a check ran and failed
    INCOMPLETE = 'incomplete'  # at least one mandatory check could not run at all (missing tolerance,
                                # non-watertight mesh, or an unverified tool-axis convention) — NOT
                                # equivalent to passing; it means safety is simply unproven


@dataclass
class ValidationFinding:
    severity: str          # 'hard' | 'incomplete' | 'flag'
    rule_or_formula: str   # e.g. 'F57', 'R10', '18.3'
    pass_id: int | None
    waypoint_index: int | None
    message: str


@dataclass
class RouteValidationReport:
    region_id: str
    findings: list = field(default_factory=list)    # list[ValidationFinding]
    tolerances_used: object | None = None   # an app.meshflow.open_decisions.AcceptanceTolerances
    status: ValidationStatus = ValidationStatus.INCOMPLETE

    @property
    def exportable(self) -> bool:
        return self.status is ValidationStatus.PASSED   # BLOCKED and INCOMPLETE are both non-exportable
