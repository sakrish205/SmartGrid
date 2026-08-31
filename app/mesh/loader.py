from __future__ import annotations
import trimesh


def load_mesh(filepath: str) -> trimesh.Trimesh:
    """Load STL or OBJ, repair winding/normals, validate."""
    raw = trimesh.load(filepath, force='mesh')

    # OBJ with material groups returns a Scene; merge into one Trimesh
    if isinstance(raw, trimesh.Scene):
        raw = raw.dump(concatenate=True)

    if not isinstance(raw, trimesh.Trimesh):
        raise ValueError(f"Could not load a single mesh from: {filepath}")

    trimesh.repair.fix_normals(raw)
    trimesh.repair.fix_winding(raw)

    if raw.is_empty or len(raw.faces) < 4:
        raise ValueError("Mesh has no usable faces — check the file.")

    return raw
