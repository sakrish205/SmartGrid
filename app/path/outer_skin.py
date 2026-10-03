"""Outer skin detection: identify faces visible from outside the part."""
from __future__ import annotations
import numpy as np
import trimesh


def get_outer_skin_faces(mesh: trimesh.Trimesh) -> np.ndarray:
    """Return indices of faces visible from outside (not occluded by other geometry).

    Casts one ray per face from centroid + epsilon outward along its normal.
    Any face whose ray intersects another face is occluded (inner skin).
    Faces whose ray reaches free space unobstructed are outer skin.

    Uses trimesh's batched ray intersector — one query for all faces.
    ponytail: multiple_hits=False — one hit per ray is enough to decide
              occlusion; upgrading to True adds cost without changing result.
    """
    epsilon    = 1e-3 * float(mesh.scale)
    centroids  = mesh.triangles_center   # (F, 3)
    normals    = mesh.face_normals       # (F, 3)  already unit vectors
    origins    = centroids + epsilon * normals
    directions = normals

    _, ray_indices, _ = mesh.ray.intersects_location(
        ray_origins=origins,
        ray_directions=directions,
        multiple_hits=False,
    )

    outer = np.ones(len(mesh.faces), dtype=bool)
    if len(ray_indices):
        outer[np.unique(ray_indices)] = False
    return np.where(outer)[0].astype(np.intp)
