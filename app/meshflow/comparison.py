"""Mesh Flow Phase 7 — controlled comparison (§9, decision #7).

Where Phase 1-2 (ReferenceProgram/ReferenceMetricsReport) and Phase 3-6
(PaintRoute/PathQualityReport/RouteValidationReport) meet for the first time.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from app.reference.reference_metrics import LaneMetrics, compute_reference_metrics, lane_pitch
from app.reference.reference_model import ReferenceProgram


class CorrespondenceConfidence(Enum):
    VERIFIED = 'verified'       # region correspondence confirmed AND traversal direction consistent
    MISMATCHED = 'mismatched'   # traversal direction verified REVERSED — auto-corrected, then VERIFIED
    AMBIGUOUS = 'ambiguous'     # region unconfirmed, or traversal-direction check inconclusive —
                                  # no pairing attempted at all


@dataclass
class LaneComparison:
    reference_lane_index: int
    generated_pass_id: int
    pair_centroid_distance_mm: float      # diagnostic only — never used to CHOOSE the pairing
    pitch_delta_mm: float | None
    smoothness_ratio: float
    curvature_delta: float
    orientation_jump_delta_deg: float | None
    length_delta_mm: float


@dataclass
class ComparisonReport:
    source_file: str
    correspondence: CorrespondenceConfidence
    lane_comparisons: list[LaneComparison] = field(default_factory=list)   # EMPTY whenever AMBIGUOUS
    unmatched_reference_lanes: int = 0
    unmatched_generated_passes: int = 0
    tolerances_applied: object | None = None   # an app.meshflow.open_decisions.AcceptanceTolerances
    notes: list[str] = field(default_factory=list)


_DIRECTION_CONFIDENCE_THRESHOLD = 0.5
_MIN_MEANINGFUL_SPAN_MM = 1e-6


def _pass_centroid(points: np.ndarray) -> np.ndarray:
    return points.mean(axis=0)


def _check_traversal_direction(
    reference_lanes: list[LaneMetrics],
    reference_centroids: list[np.ndarray],
    generated_centroids: list[np.ndarray],
) -> CorrespondenceConfidence:
    """Compute d_ref = centroid(last reference lane) - centroid(first reference
    lane), and d_gen the same for the generated passes. If either span is too
    small to be a meaningful direction signal, or the normalized dot product
    falls in (-0.5, +0.5) (neither clearly aligned nor clearly reversed),
    return AMBIGUOUS — do not guess."""
    if len(reference_centroids) < 2 or len(generated_centroids) < 2:
        return CorrespondenceConfidence.AMBIGUOUS

    d_ref = reference_centroids[-1] - reference_centroids[0]
    d_gen = generated_centroids[-1] - generated_centroids[0]
    ref_span = np.linalg.norm(d_ref)
    gen_span = np.linalg.norm(d_gen)
    if ref_span < _MIN_MEANINGFUL_SPAN_MM or gen_span < _MIN_MEANINGFUL_SPAN_MM:
        return CorrespondenceConfidence.AMBIGUOUS

    dot = float(np.dot(d_ref / ref_span, d_gen / gen_span))
    if dot > _DIRECTION_CONFIDENCE_THRESHOLD:
        return CorrespondenceConfidence.VERIFIED
    if dot < -_DIRECTION_CONFIDENCE_THRESHOLD:
        return CorrespondenceConfidence.MISMATCHED
    return CorrespondenceConfidence.AMBIGUOUS


def compare_to_reference(
    generated_route,
    generated_quality: list,
    reference_program: ReferenceProgram,
    region_correspondence_confirmed: bool,
    arc_spacing_mm: float = 10.0,
) -> ComparisonReport:
    """Matching strategy: index-order correspondence, used ONLY after both
    region correspondence (caller-confirmed — ReferenceProgram carries no mesh/
    region identity to auto-verify this, a direct consequence of decision #7)
    and traversal-direction consistency (auto-checked from centroid spans) are
    established. Nearest-centroid matching is NOT used — it is provably wrong
    on repetitive/symmetric geometry (lanes spaced evenly on one flat panel, or
    two mirrored panels, can each put a reference lane's centroid closer to the
    WRONG generated pass than its true counterpart)."""
    ref_report = compute_reference_metrics(reference_program, arc_spacing_mm)
    reference_lanes = ref_report.lanes
    generated_passes = generated_route.passes

    if not region_correspondence_confirmed:
        return ComparisonReport(
            source_file=reference_program.source_file,
            correspondence=CorrespondenceConfidence.AMBIGUOUS,
            unmatched_reference_lanes=len(reference_lanes),
            unmatched_generated_passes=len(generated_passes),
            notes=["region correspondence not confirmed by caller — no pairing attempted "
                   "(decision #7: no matched OLP+geometry corpus to auto-verify this)"],
        )

    if not reference_lanes or not generated_passes:
        return ComparisonReport(
            source_file=reference_program.source_file,
            correspondence=CorrespondenceConfidence.AMBIGUOUS,
            unmatched_reference_lanes=len(reference_lanes),
            unmatched_generated_passes=len(generated_passes),
            notes=["no reference lanes or no generated passes for this geometry — decision #7 unresolved"
                   if not reference_lanes else "generated route has no passes"],
        )

    # Reference lane centroids aren't stored directly on LaneMetrics; reconstruct
    # from mean_direction-less data is not possible, so use each lane's own
    # waypoint_count-weighted placeholder is wrong too — instead, reference
    # centroids must come from the lanes' own motion waypoints, already
    # consumed inside compute_reference_metrics. Re-derive via the program
    # directly so this stays a single source of truth.
    from app.path.segment_runs import split_contiguous_gun_on_runs
    motion = reference_program.motion_waypoints()
    lane_runs = split_contiguous_gun_on_runs(motion)
    reference_centroids = [
        _pass_centroid(np.array([w.position for w in run])) for run in lane_runs if len(run) >= 2
    ]
    generated_centroids = [_pass_centroid(p.points) for p in generated_passes]

    confidence = _check_traversal_direction(reference_lanes, reference_centroids, generated_centroids)
    if confidence is CorrespondenceConfidence.AMBIGUOUS:
        return ComparisonReport(
            source_file=reference_program.source_file,
            correspondence=CorrespondenceConfidence.AMBIGUOUS,
            unmatched_reference_lanes=len(reference_lanes),
            unmatched_generated_passes=len(generated_passes),
            notes=["traversal-direction check inconclusive — no pairing attempted"],
        )

    if confidence is CorrespondenceConfidence.MISMATCHED:
        generated_passes = list(reversed(generated_passes))
        generated_centroids = list(reversed(generated_centroids))

    n_matched = min(len(reference_lanes), len(generated_passes))
    quality_by_pass_id = {q.pass_id: q for q in generated_quality} if generated_quality else {}

    comparisons: list[LaneComparison] = []
    for i in range(n_matched):
        ref_lane = reference_lanes[i]
        gen_pass = generated_passes[i]
        gen_quality = quality_by_pass_id.get(gen_pass.id)

        pitch_delta = None
        if i + 1 < len(reference_centroids) and i + 1 < len(generated_centroids):
            spacing_dir = generated_centroids[i + 1] - generated_centroids[i]
            if np.linalg.norm(spacing_dir) > 1e-9:
                ref_pitch = lane_pitch(reference_centroids[i], reference_centroids[i + 1], spacing_dir)
                gen_pitch = lane_pitch(generated_centroids[i], generated_centroids[i + 1], spacing_dir)
                pitch_delta = abs(ref_pitch - gen_pitch)

        gen_length = float(np.sum(np.linalg.norm(np.diff(gen_pass.points, axis=0), axis=1)))
        gen_smoothness = gen_quality.smoothness_penalty if gen_quality else float('nan')
        gen_curvature = gen_quality.max_curvature if gen_quality else float('nan')
        gen_orientation = gen_quality.orientation_max_jump_deg if gen_quality else None

        comparisons.append(LaneComparison(
            reference_lane_index=i,
            generated_pass_id=gen_pass.id,
            pair_centroid_distance_mm=float(np.linalg.norm(reference_centroids[i] - generated_centroids[i])),
            pitch_delta_mm=pitch_delta,
            smoothness_ratio=(gen_smoothness / ref_lane.smoothness_penalty
                               if ref_lane.smoothness_penalty > 1e-9 else float('nan')),
            curvature_delta=abs(gen_curvature - ref_lane.max_curvature),
            orientation_jump_delta_deg=(
                abs(gen_orientation - np.degrees(ref_lane.max_orientation_jump_rad))
                if gen_orientation is not None else None
            ),
            length_delta_mm=abs(gen_length - ref_lane.total_length_mm),
        ))

    return ComparisonReport(
        source_file=reference_program.source_file,
        correspondence=CorrespondenceConfidence.VERIFIED,
        lane_comparisons=comparisons,
        unmatched_reference_lanes=max(0, len(reference_lanes) - n_matched),
        unmatched_generated_passes=max(0, len(generated_passes) - n_matched),
        notes=[],
    )
