from __future__ import annotations
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np
import pyvista as pv
import trimesh


@dataclass
class MeshData:
    source_path: str
    trimesh_mesh: trimesh.Trimesh
    face_normals: np.ndarray      # (F, 3) unit normals
    face_centroids: np.ndarray    # (F, 3) triangle centres
    bbox_min: np.ndarray          # (3,)
    bbox_max: np.ndarray          # (3,)
    bbox_center: np.ndarray       # (3,)
    bbox_extents: np.ndarray      # (3,) per-axis lengths
    adjacency: dict               # face_id -> list[face_id] (edge-neighbours)
    up_axis: int                  # 0=X, 1=Y, 2=Z set by user at load time
    pyvista_mesh: pv.PolyData     # pre-converted, stored once


def preprocess(mesh: trimesh.Trimesh, source_path: str, up_axis: int = 2) -> MeshData:
    """Compute and cache all geometry data needed by the rest of the app."""
    face_normals = mesh.face_normals.copy()   # (F, 3) — copy so trimesh can't mutate it

    # (F, 3): mean of each face's three vertex positions
    face_centroids = mesh.vertices[mesh.faces].mean(axis=1)

    bounds = mesh.bounds          # shape (2, 3): [min, max]
    bbox_min = bounds[0]
    bbox_max = bounds[1]
    bbox_center = (bbox_min + bbox_max) / 2.0
    bbox_extents = bbox_max - bbox_min

    # Edge-based face adjacency: trimesh gives (N, 2) pairs of adjacent face IDs
    adj: dict[int, list[int]] = defaultdict(list)
    for a, b in mesh.face_adjacency:
        adj[int(a)].append(int(b))
        adj[int(b)].append(int(a))

    pv_mesh = _to_pyvista(mesh)

    return MeshData(
        source_path=source_path,
        trimesh_mesh=mesh,
        face_normals=face_normals,
        face_centroids=face_centroids,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        bbox_center=bbox_center,
        bbox_extents=bbox_extents,
        adjacency=dict(adj),
        up_axis=up_axis,
        pyvista_mesh=pv_mesh,
    )


def _to_pyvista(mesh: trimesh.Trimesh) -> pv.PolyData:
    """Convert trimesh.Trimesh to pyvista.PolyData (done once, stored on MeshData)."""
    n = len(mesh.faces)
    # PyVista face format: [3, v0, v1, v2, 3, v0, v1, v2, ...]
    connectivity = np.column_stack([np.full(n, 3, dtype=np.int_), mesh.faces]).ravel()
    return pv.PolyData(mesh.vertices.copy(), connectivity)
