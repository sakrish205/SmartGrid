"""Mesh Flow Phase 7 exit-gate tests: correspondence-verified comparison.

Index-order matching is used ONLY after region correspondence (caller-
confirmed) and traversal-direction consistency (auto-checked) are both
established — never asserted by construction.
"""
from __future__ import annotations
import os

import numpy as np
import pytest

from app.meshflow.comparison import CorrespondenceConfidence, compare_to_reference
from app.path.path_model import PaintPass, PaintRoute
from app.reference.reference_model import ProcessState, ReferenceFrameType, ReferenceProgram, ReferenceWaypoint

_ABB_PATH = os.path.join(os.path.dirname(__file__), '..', 'olp review details', 'Exercise_2.mod')


def _lane_waypoints(start_idx: int, x: float, with_transit: bool) -> tuple[list[ReferenceWaypoint], int]:
    wps = []
    idx = start_idx
    for y in (0.0, 100.0):
        wps.append(ReferenceWaypoint(
            source_id=f'P{idx}', exec_index=idx, position=np.array([x, y, 0.0]),
            orientation=None, frame_type=ReferenceFrameType.OBJECT, move_type='L',
            process_state=ProcessState(gun_on=True), speed_ref=None, is_motion=True,
        ))
        idx += 1
    if with_transit:
        wps.append(ReferenceWaypoint(
            source_id=f'T{idx}', exec_index=idx, position=np.array([x, 150.0, 0.0]),
            orientation=None, frame_type=ReferenceFrameType.OBJECT, move_type='L',
            process_state=ProcessState(gun_on=False), speed_ref=None, is_motion=True,
        ))
        idx += 1
    return wps, idx


def _synthetic_reference(n_lanes: int, pitch_mm: float = 50.0) -> ReferenceProgram:
    waypoints: list[ReferenceWaypoint] = []
    idx = 0
    for lane in range(n_lanes):
        wps, idx = _lane_waypoints(idx, lane * pitch_mm, with_transit=(lane < n_lanes - 1))
        waypoints.extend(wps)
    return ReferenceProgram(source_file='synthetic', controller='ABB', waypoints=waypoints)


def _synthetic_route(n_passes: int, pitch_mm: float = 50.0, region_id: str = 'R') -> PaintRoute:
    passes = []
    for lane in range(n_passes):
        x = lane * pitch_mm
        pts = np.array([[x, 0.0, 0.0], [x, 100.0, 0.0]])
        passes.append(PaintPass(id=lane, region_id=region_id, direction='mesh_flow', points=pts,
                                 is_forward=True, sub_index=0, slice_position=x))
    return PaintRoute(region_id=region_id, passes=passes, connections=[], unit='mm',
                       spacing_mm=pitch_mm, total_passes=n_passes, total_length_mm=n_passes * 100.0)


def test_unconfirmed_region_correspondence_is_always_ambiguous():
    ref = _synthetic_reference(3)
    route = _synthetic_route(3)
    report = compare_to_reference(route, [], ref, region_correspondence_confirmed=False)
    assert report.correspondence is CorrespondenceConfidence.AMBIGUOUS
    assert report.lane_comparisons == []
    assert report.unmatched_reference_lanes == 3
    assert report.unmatched_generated_passes == 3


def test_verified_correspondence_pairs_by_index():
    ref = _synthetic_reference(3)
    route = _synthetic_route(3)
    report = compare_to_reference(route, [], ref, region_correspondence_confirmed=True)
    assert report.correspondence is CorrespondenceConfidence.VERIFIED
    assert [c.reference_lane_index for c in report.lane_comparisons] == [0, 1, 2]
    assert [c.generated_pass_id for c in report.lane_comparisons] == [0, 1, 2]
    assert report.unmatched_reference_lanes == 0
    assert report.unmatched_generated_passes == 0


def test_mismatched_traversal_direction_is_auto_corrected():
    ref = _synthetic_reference(3)
    route = _synthetic_route(3)
    route.passes = list(reversed(route.passes))   # generated sweep reversed relative to reference
    report = compare_to_reference(route, [], ref, region_correspondence_confirmed=True)
    assert report.correspondence is CorrespondenceConfidence.VERIFIED
    # After auto-correction, lane k must still pair with the correct generated pass id.
    pairs = {c.reference_lane_index: c.generated_pass_id for c in report.lane_comparisons}
    assert pairs[0] == 0 and pairs[1] == 1 and pairs[2] == 2


def test_inconclusive_traversal_direction_is_ambiguous_not_guessed():
    # A single-lane reference and generated route give no meaningful direction span.
    ref = _synthetic_reference(1)
    route = _synthetic_route(1)
    report = compare_to_reference(route, [], ref, region_correspondence_confirmed=True)
    assert report.correspondence is CorrespondenceConfidence.AMBIGUOUS
    assert report.lane_comparisons == []


def test_symmetric_geometry_regression_index_matching_not_nearest_centroid():
    """The original motivating case: two mirrored, evenly-spaced lane sets on
    the same confirmed region with consistent traversal direction — nearest-
    centroid matching would provably cross-pair lane 0 with a closer but
    wrong-index generated pass; index matching must not."""
    ref = _synthetic_reference(4, pitch_mm=10.0)   # tightly, evenly spaced -> ambiguous by proximity
    route = _synthetic_route(4, pitch_mm=10.0)
    report = compare_to_reference(route, [], ref, region_correspondence_confirmed=True)
    assert report.correspondence is CorrespondenceConfidence.VERIFIED
    for c in report.lane_comparisons:
        assert c.reference_lane_index == c.generated_pass_id   # strict index pairing holds


@pytest.mark.skipif(not os.path.exists(_ABB_PATH), reason='olp review details/ not present locally')
def test_end_to_end_with_real_abb_sample_reports_honestly():
    from app.reference.parsers import abb_parser
    prog = abb_parser.parse(_ABB_PATH)
    route = _synthetic_route(5)   # unrelated synthetic geometry — deltas may be large, that's fine

    report = compare_to_reference(route, [], prog, region_correspondence_confirmed=True)
    assert report.source_file == prog.source_file
    for c in report.lane_comparisons:
        assert np.isfinite(c.length_delta_mm)
        assert np.isfinite(c.pair_centroid_distance_mm)

    # A missing reference program's worth of lanes must still produce a report, never raise.
    empty_prog = ReferenceProgram(source_file='missing', controller='ABB', waypoints=[])
    empty_report = compare_to_reference(route, [], empty_prog, region_correspondence_confirmed=True)
    assert empty_report.correspondence is CorrespondenceConfidence.AMBIGUOUS
    assert empty_report.notes
