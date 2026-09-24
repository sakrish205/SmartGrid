from __future__ import annotations
import os
import trimesh


def _load_step(filepath: str, mesh_size_factor: float = 0.05) -> trimesh.Trimesh:
    """Tessellate a STEP file via gmsh.

    mesh_size_factor controls triangle density relative to the model bounding box:
      0.02 = fine  (slow, many triangles)
      0.05 = medium (default, good balance)
      0.10 = coarse (fast, fewer triangles)
    """
    import gmsh
    import numpy as np
    gmsh.initialize()
    gmsh.option.setNumber('General.Verbosity', 0)
    gmsh.model.occ.importShapes(filepath)
    gmsh.model.occ.synchronize()
    # Prevent over-tessellation on large/complex CAD by setting a minimum element size
    bbox = gmsh.model.getBoundingBox(-1, -1)  # (xmin,ymin,zmin,xmax,ymax,zmax)
    diagonal = ((bbox[3]-bbox[0])**2 + (bbox[4]-bbox[1])**2 + (bbox[5]-bbox[2])**2) ** 0.5
    min_size = max(diagonal * mesh_size_factor, 0.5)
    gmsh.option.setNumber('Mesh.CharacteristicLengthMin', min_size)
    gmsh.model.mesh.generate(2)
    _, coords, _ = gmsh.model.mesh.getNodes()
    verts = coords.reshape(-1, 3)
    _, _, conn = gmsh.model.mesh.getElements(2)   # triangles
    faces = conn[0].reshape(-1, 3).astype(np.int64) - 1  # gmsh is 1-indexed
    gmsh.finalize()
    # gmsh output is already clean — skip expensive process=True repair
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
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
