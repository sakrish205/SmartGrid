"""Shared pytest fixtures."""
import numpy as np
import pytest
import trimesh


@pytest.fixture
def unit_box_mesh() -> trimesh.Trimesh:
    """Axis-aligned box from [-0.5, -0.5, -0.5] to [0.5, 0.5, 0.5]."""
    return trimesh.creation.box()


@pytest.fixture
def cylinder_mesh() -> trimesh.Trimesh:
    return trimesh.creation.cylinder(radius=50.0, height=200.0, sections=64)


@pytest.fixture
def flat_plane_mesh() -> trimesh.Trimesh:
    """Flat XZ-plane quad at Y=0, 200 mm x 200 mm."""
    verts = np.array([
        [-100, 0, -100],
        [ 100, 0, -100],
        [ 100, 0,  100],
        [-100, 0,  100],
    ], dtype=float)
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    return trimesh.Trimesh(vertices=verts, faces=faces, process=False)
