"""Tests for app.path.validator — blocking and reporting checks."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import pytest
import trimesh

from app.path.path_model import PaintPass
from app.path.validator import (
    check_straightness,
    check_collision,
    measure_standoff_deviation,
    measure_boundary_gap,
    check_tool_flip,
)


def _make_pass(pts, normals=None):
    return PaintPass(
        id=0, region_id='TEST', direction='horizontal',
        points=np.asarray(pts, dtype=float),
        is_forward=True, sub_index=0, slice_position=0.0,
        normals=normals,
    )


def test_check_straightness_passes_for_straight_line():
    pts = np.array([[i * 10.0, 0.0, 0.0] for i in range(10)])
    check_straightness(_make_pass(pts))  # must not raise


def test_check_straightness_raises_for_curved_path():
    # Curve: waypoints bow up by 20 mm at the midpoint
    pts = np.array([[i * 10.0, 0.0, 0.0] for i in range(11)], dtype=float)
    pts[5, 2] = 20.0  # 20 mm lateral deviation
    with pytest.raises(ValueError, match='check_straightness'):
        check_straightness(_make_pass(pts))


def test_check_straightness_skips_short_pass():
    # < 3 points: no check
    pts = np.array([[0., 0., 0.], [10., 0., 0.]])
    check_straightness(_make_pass(pts))  # must not raise


def test_check_collision_watertight_outside():
    mesh = trimesh.creation.box(extents=[100., 100., 100.])
    # Points well outside the box
    pts = np.array([[200., 0., 0.], [300., 0., 0.]])
    check_collision(_make_pass(pts), mesh)  # must not raise


def test_check_collision_watertight_inside():
    mesh = trimesh.creation.box(extents=[100., 100., 100.])
    pts = np.array([[0., 0., 0.]])  # centre of box — inside
    with pytest.raises(ValueError, match='check_collision'):
        check_collision(_make_pass(pts), mesh)


def test_measure_standoff_deviation_on_plane():
    # Flat plane at z=0; waypoints at z=10 → nominal standoff 10 mm → deviation 0
    mesh = trimesh.creation.box(extents=[200., 200., 1.])
    pts = np.array([[i * 10.0 - 50.0, 0.0, 10.5] for i in range(10)])
    # closest point to a flat box's top face is at z=0.5 → dist ~10 mm
    dev = measure_standoff_deviation(_make_pass(pts), mesh, nominal_standoff_mm=10.0)
    assert dev < 2.0, f"standoff deviation {dev:.2f} mm unexpectedly large"


def test_check_tool_flip_false_for_consistent_normals():
    normals = np.tile([0., 0., 1.], (5, 1))
    p = _make_pass([[i, 0., 0.] for i in range(5)], normals=normals)
    assert check_tool_flip(p) is False


def test_check_tool_flip_true_for_flipped_normals():
    normals = np.array([[0., 0., 1.], [0., 0., 1.], [0., 0., -1.], [0., 0., -1.]])
    p = _make_pass([[i, 0., 0.] for i in range(4)], normals=normals)
    assert check_tool_flip(p) is True


def test_measure_boundary_gap_open_mesh():
    # Open cylinder (no caps) has boundary edges at both ends (z ~ ±100)
    mesh = trimesh.creation.cylinder(radius=50., height=200., sections=32)
    side_mask = np.abs(mesh.face_normals[:, 2]) < 0.5
    mesh = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces[side_mask],
                           process=False)

    # Pass that covers the full height — boundary verts at z=±100 are within reach
    pts = np.array([[50., 0., z] for z in np.linspace(-100., 100., 30)])
    p = _make_pass(pts)
    gap = measure_boundary_gap([p], mesh)
    # Boundary verts at z=±100 on a 32-sided cylinder: max angular separation between
    # vertices and the single pass column (x=50, y=0) is up to full circumferential span.
    # The function should return a positive finite number — not zero, not infinity.
    assert 0.0 < gap < 400.0, f"boundary gap {gap:.1f} mm out of expected range"
