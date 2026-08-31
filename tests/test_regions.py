"""Tests for triangle region classification."""
import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from app.mesh.preprocessor import preprocess
from app.mesh.regions import classify_regions


def test_six_regions_on_box(unit_box_mesh):
    data = preprocess(unit_box_mesh, "test")
    regions = classify_regions(data, threshold=0.3)
    assert set(regions.keys()) >= {'TOP', 'BOTTOM', 'FRONT', 'REAR', 'LEFT', 'RIGHT'}
    # No face appears in two regions
    all_faces = np.concatenate(list(regions.values()))
    assert len(all_faces) == len(np.unique(all_faces)), "Duplicate face assignments"


def test_top_normals_point_up(unit_box_mesh):
    data = preprocess(unit_box_mesh, "test")
    regions = classify_regions(data, threshold=0.3)
    top = regions.get('TOP', np.array([]))
    assert len(top) > 0, "No faces classified as TOP"
    top_normals = data.face_normals[top]
    assert np.all(top_normals[:, 2] > 0.5), "TOP faces have normals not pointing +Z"


def test_all_faces_classified_on_box(unit_box_mesh):
    data = preprocess(unit_box_mesh, "test")
    regions = classify_regions(data, threshold=0.3)
    all_classified = np.concatenate(list(regions.values()))
    assert len(all_classified) == len(unit_box_mesh.faces), (
        f"Expected {len(unit_box_mesh.faces)} faces classified, "
        f"got {len(all_classified)}"
    )
