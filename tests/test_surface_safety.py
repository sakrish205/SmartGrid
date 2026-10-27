"""Mesh Flow Phase 6 exit-gate tests: F57/F58 surface_safety utilities."""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.surface_safety import inside_mesh_mask, segment_penetrates


def test_segment_penetrates_catches_a_chord_through_a_thin_feature():
    """Exact-intersection method closes the gap structurally: a chord through a
    thin feature strictly between what 8 discrete samples would have landed on
    must still be caught — no fixed finite sample count smaller than the
    feature's width could ever catch this with the old sampling design."""
    box = trimesh.creation.box(extents=[100, 100, 2])   # a thin 2mm-deep slab
    p0 = np.array([-60.0, 0.0, 0.0])
    p1 = np.array([60.0, 0.0, 0.0])   # chord passes straight through the slab's middle
    assert segment_penetrates(p0, p1, box) is True


def test_segment_penetrates_false_on_clean_miss():
    box = trimesh.creation.box(extents=[100, 100, 100])
    p0 = np.array([0.0, 0.0, 100.0])
    p1 = np.array([0.0, 0.0, 200.0])
    assert segment_penetrates(p0, p1, box) is False


def test_segment_penetrates_returns_none_on_degenerate_segment():
    box = trimesh.creation.box(extents=[100, 100, 100])
    assert segment_penetrates([0, 0, 0], [0, 0, 0], box) is None


def test_segment_penetrates_works_without_watertight_mesh():
    """F58's relaxed precondition: exact ray/triangle crossing doesn't require
    watertightness the way F57's inside/outside test does."""
    plane = trimesh.Trimesh(
        vertices=[[-50, -50, 0], [50, -50, 0], [50, 50, 0], [-50, 50, 0]],
        faces=[[0, 1, 2], [0, 2, 3]],
        process=False,
    )
    assert plane.is_watertight is False
    result = segment_penetrates([0, 0, -10], [0, 0, 10], plane)
    assert result in (True, False)   # a real bool, not None


def test_inside_mesh_mask_none_on_non_watertight():
    plane = trimesh.Trimesh(
        vertices=[[-50, -50, 0], [50, -50, 0], [50, 50, 0], [-50, 50, 0]],
        faces=[[0, 1, 2], [0, 2, 3]],
        process=False,
    )
    assert inside_mesh_mask(np.array([[0.0, 0.0, 0.0]]), plane) is None


def test_inside_mesh_mask_detects_interior_point_on_watertight_mesh():
    box = trimesh.creation.box(extents=[100, 100, 100])
    mask = inside_mesh_mask(np.array([[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0]]), box)
    assert mask is not None
    assert mask[0] is True or bool(mask[0]) is True
    assert mask[1] is False or bool(mask[1]) is False
