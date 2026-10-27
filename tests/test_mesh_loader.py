"""app.mesh.loader exit-gate tests: STL/OBJ winding repair.

Real-world STL/OBJ exports (scanned/online models, not clean in-house CAD)
routinely have a subset of triangles with flipped/inconsistent winding —
confirmed directly against several real downloaded car-body STLs during
development. _load_step already repairs this for STEP via gmsh; load_mesh
must do the same for STL/OBJ, since every path mode's standoff offset
(app.path.local_normals) depends on face normals pointing the right way.
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.mesh.loader import load_mesh


def _box_with_flipped_faces(flip_fraction: float = 0.3) -> trimesh.Trimesh:
    box = trimesh.creation.box(extents=[50.0, 50.0, 50.0])
    faces = box.faces.copy()
    rng = np.random.default_rng(0)
    n_flip = max(1, int(len(faces) * flip_fraction))
    flip_idx = rng.choice(len(faces), n_flip, replace=False)
    faces[flip_idx] = faces[flip_idx][:, ::-1]   # reverse winding order
    return trimesh.Trimesh(vertices=box.vertices, faces=faces, process=False)


def test_load_mesh_repairs_inconsistent_stl_winding(tmp_path):
    flipped = _box_with_flipped_faces()
    assert flipped.is_winding_consistent is False   # sanity: the fixture is actually broken

    path = tmp_path / 'flipped.stl'
    flipped.export(str(path))

    loaded = load_mesh(str(path))
    assert loaded.is_winding_consistent is True
    # Geometry itself must be unchanged -- fix_normals only reorders winding.
    assert np.allclose(
        sorted(loaded.bounding_box.extents), sorted(flipped.bounding_box.extents), atol=1e-6,
    )


def test_load_mesh_leaves_already_consistent_mesh_unchanged(tmp_path):
    box = trimesh.creation.box(extents=[20.0, 30.0, 40.0])
    assert box.is_winding_consistent is True   # sanity: the fixture is already clean

    path = tmp_path / 'clean.stl'
    box.export(str(path))

    loaded = load_mesh(str(path))
    assert loaded.is_winding_consistent is True
    assert np.allclose(
        sorted(loaded.bounding_box.extents), sorted(box.bounding_box.extents), atol=1e-6,
    )
