"""Tests for trimesh plane intersection + region filtering."""
import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from app.mesh.preprocessor import preprocess
from app.mesh.regions import classify_regions
from app.path.slicer import slice_region, compute_slice_config


def test_slice_returns_none_when_plane_misses(unit_box_mesh):
    data = preprocess(unit_box_mesh, "test")
    regions = classify_regions(data)
    top_faces = regions.get('TOP', np.array([]))
    # Plane far above the box — should miss entirely
    result = slice_region(
        unit_box_mesh, top_faces,
        plane_normal=np.array([0, 0, 1.0]),
        plane_origin=np.array([0, 0, 100.0]),
    )
    assert result is None or len(result) == 0


def test_slice_config_top_region(unit_box_mesh):
    data = preprocess(unit_box_mesh, "test", up_axis=2)
    cfg = compute_slice_config('TOP', up_axis=2)
    # For TOP with Z-up: slice_axis should be the front/rear axis (Y = axis 1)
    assert cfg['slice_axis'] == 1
    assert cfg['plane_normal'][1] == 1.0
