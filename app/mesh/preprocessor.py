from __future__ import annotations
from dataclasses import dataclass

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
    up_axis: int                  # 0=X, 1=Y, 2=Z set by user at load time
    pyvista_mesh: pv.PolyData     # pre-converted, stored once


def preprocess(mesh: trimesh.Trimesh, source_path: str, up_axis: int = 2) -> MeshData:
    """Compute and cache all geometry data needed by the rest of the app."""
    face_normals = mesh.face_normals.copy()   # (F, 3) — copy so trimesh can't mutate it

    # (F, 3): mean of each face's three vertex positions
    # Direct sum avoids a (F, 3, 3) intermediate (72 MB on 1M-face meshes)
    v, f = mesh.vertices, mesh.faces
    face_centroids = (v[f[:, 0]] + v[f[:, 1]] + v[f[:, 2]]) / 3.0

    bounds = mesh.bounds          # shape (2, 3): [min, max]
    bbox_min = bounds[0]
    bbox_max = bounds[1]
    bbox_center = (bbox_min + bbox_max) / 2.0
    bbox_extents = bbox_max - bbox_min

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
        up_axis=up_axis,
        pyvista_mesh=pv_mesh,
    )


def _to_pyvista(mesh: trimesh.Trimesh) -> pv.PolyData:
    """Convert trimesh.Trimesh to pyvista.PolyData (done once, stored on MeshData)."""
    n = len(mesh.faces)
    # PyVista face format: [3, v0, v1, v2, ...] — write directly to avoid column_stack copy
    connectivity = np.empty(n * 4, dtype=np.int64)
    connectivity[0::4] = 3
    connectivity[1::4] = mesh.faces[:, 0]
    connectivity[2::4] = mesh.faces[:, 1]
    connectivity[3::4] = mesh.faces[:, 2]
    return pv.PolyData(mesh.vertices.copy(), connectivity)
