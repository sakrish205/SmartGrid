"""Tests for Mesh Flow Phase 2 reference metrics — exact-formula unit checks
plus one integration pass over a real sample file (skipped if not present
locally, since olp review details/ is gitignored).
"""
from __future__ import annotations
import os

import numpy as np
import pytest

from app.reference.reference_metrics import (
    approximate_curvature,
    compute_reference_metrics,
    max_orientation_jump,
    orientation_continuity,
    pointing_axis,
    second_differences,
    segment_directions,
    segment_lengths,
    smoothness_penalty,
    total_length,
)

_ABB_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'olp review details', 'Exercise_2.mod',
)


def test_f13_segment_direction_skips_duplicates():
    pts = np.array([[0, 0, 0], [0, 0, 0], [10, 0, 0], [10, 0, 10]], dtype=float)
    dirs = segment_directions(pts)
    assert len(dirs) == 2  # the zero-length duplicate segment is skipped
    np.testing.assert_allclose(dirs[0], [1, 0, 0])
    np.testing.assert_allclose(dirs[1], [0, 0, 1])


def test_f36_f37_segment_and_total_length():
    pts = np.array([[0, 0, 0], [3, 0, 0], [3, 4, 0]], dtype=float)
    lengths = segment_lengths(pts)
    np.testing.assert_allclose(lengths, [3.0, 4.0])
    assert total_length(pts) == pytest.approx(7.0)


def test_f17_second_difference_zero_on_straight_line():
    pts = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], dtype=float)
    d2 = second_differences(pts)
    np.testing.assert_allclose(d2, 0.0, atol=1e-9)


def test_f17_second_difference_nonzero_on_sharp_turn():
    pts = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0]], dtype=float)
    d2 = second_differences(pts)
    assert len(d2) == 1
    assert np.linalg.norm(d2[0]) > 0


def test_f18_smoothness_penalty_rewards_straight_lines():
    straight = np.array([[i, 0, 0] for i in range(10)], dtype=float)
    assert smoothness_penalty(straight) == pytest.approx(0.0, abs=1e-9)

    zigzag = np.array([[i, (i % 2) * 5, 0] for i in range(10)], dtype=float)
    assert smoothness_penalty(zigzag) > 0


def test_f19_approximate_curvature_higher_on_bend():
    straight = np.array([[i * 10.0, 0, 0] for i in range(10)], dtype=float)
    bent = np.array([[i * 10.0, (i * 10.0) ** 1.3 * 0.01, 0] for i in range(10)], dtype=float)
    assert approximate_curvature(straight, arc_spacing_mm=5.0).max() < 1e-6
    assert approximate_curvature(bent, arc_spacing_mm=5.0).max() > 1e-6


def test_f44_f45_orientation_continuity():
    # Three axes: first two identical (0 jump), third rotated 90 degrees.
    axes = np.array([[0, 0, 1], [0, 0, 1], [1, 0, 0]], dtype=float)
    theta = orientation_continuity(axes)
    assert theta[0] == pytest.approx(0.0, abs=1e-9)
    assert theta[1] == pytest.approx(np.pi / 2, abs=1e-6)
    assert max_orientation_jump(axes) == pytest.approx(np.pi / 2, abs=1e-6)


def test_pointing_axis_abb_quaternion_round_trip():
    # Identity quaternion [w,x,y,z]=[1,0,0,0] -> rotation matrix is identity -> Z axis = [0,0,1].
    axis = pointing_axis(np.array([1.0, 0.0, 0.0, 0.0]), controller='ABB')
    np.testing.assert_allclose(axis, [0, 0, 1], atol=1e-9)


def test_pointing_axis_fanuc_euler_zero_is_identity():
    axis = pointing_axis(np.array([0.0, 0.0, 0.0]), controller='FANUC')
    np.testing.assert_allclose(axis, [0, 0, 1], atol=1e-9)


@pytest.mark.skipif(not os.path.exists(_ABB_PATH), reason='olp review details/ not present locally')
def test_compute_reference_metrics_on_real_abb_file():
    from app.reference.parsers import abb_parser

    program = abb_parser.parse(_ABB_PATH)
    report = compute_reference_metrics(program)

    assert report.controller == 'ABB'
    assert len(report.lanes) >= 1
    assert report.lanes[0].total_length_mm > 0
    assert report.lanes[0].waypoint_count == 24  # all of mvHood is one continuous gun-on run
