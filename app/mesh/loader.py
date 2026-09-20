from __future__ import annotations
import os
import trimesh


def _load_step(filepath: str) -> trimesh.Trimesh:
    import gmsh
    import numpy as np
    gmsh.initialize()
    gmsh.option.setNumber('General.Verbosity', 0)
    gmsh.model.occ.importShapes(filepath)
    gmsh.model.occ.synchronize()
    gmsh.model.mesh.generate(2)
    _, coords, _ = gmsh.model.mesh.getNodes()
    verts = coords.reshape(-1, 3)
    _, _, conn = gmsh.model.mesh.getElements(2)   # triangles
    faces = conn[0].reshape(-1, 3).astype(np.int64) - 1  # gmsh is 1-indexed
    gmsh.finalize()
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=True)
    trimesh.repair.fix_normals(mesh)
    return mesh


def load_mesh(filepath: str) -> trimesh.Trimesh:
    """Load STL, OBJ, or STEP, repair winding/normals, validate."""
    if os.path.splitext(filepath)[1].lower() in ('.step', '.stp'):
        return _load_step(filepath)
    raw = trimesh.load(filepath, force='mesh')

    # OBJ with material groups returns a Scene; merge into one Trimesh
    if isinstance(raw, trimesh.Scene):
        raw = raw.dump(concatenate=True)

    if not isinstance(raw, trimesh.Trimesh):
        raise ValueError(f"Could not load a single mesh from: {filepath}")

    if raw.is_empty or len(raw.faces) < 4:
        raise ValueError("Mesh has no usable faces — check the file.")

    return raw
