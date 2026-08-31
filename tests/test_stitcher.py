"""Unit tests for segment stitching — must all pass before any other path work."""
import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from app.path.stitcher import stitch_segments


def test_single_segment_returns_one_chain():
    segs = np.array([[[0, 0, 0], [1, 0, 0]]], dtype=float)
    result = stitch_segments(segs)
    assert len(result) == 1
    assert len(result[0]) == 2


def test_three_collinear_segments_form_one_chain():
    segs = np.array([
        [[1, 0, 0], [2, 0, 0]],
        [[0, 0, 0], [1, 0, 0]],
        [[2, 0, 0], [3, 0, 0]],
    ], dtype=float)
    result = stitch_segments(segs)
    assert len(result) == 1
    chain = result[0]
    assert len(chain) == 4
    assert np.allclose(chain[0], [0, 0, 0])
    assert np.allclose(chain[-1], [3, 0, 0])


def test_two_disconnected_chains():
    segs = np.array([
        [[0, 0, 0], [1, 0, 0]],
        [[1, 0, 0], [2, 0, 0]],
        [[5, 0, 0], [6, 0, 0]],
        [[6, 0, 0], [7, 0, 0]],
    ], dtype=float)
    result = stitch_segments(segs)
    assert len(result) == 2


def test_closed_loop():
    segs = np.array([
        [[0, 0, 0], [1, 0, 0]],
        [[1, 0, 0], [0.5, 1, 0]],
        [[0.5, 1, 0], [0, 0, 0]],
    ], dtype=float)
    result = stitch_segments(segs)
    assert len(result) == 1
    assert len(result[0]) >= 3


def test_floating_point_tolerance():
    eps = 1e-8
    segs = np.array([
        [[0, 0, 0], [1, 0, 0]],
        [[1 + eps, 0, 0], [2, 0, 0]],
    ], dtype=float)
    result = stitch_segments(segs, tolerance=1e-6)
    assert len(result) == 1, "Should stitch despite floating-point gap < tolerance"
