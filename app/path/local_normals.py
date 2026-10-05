"""Per-waypoint surface normal interpolation and TCP frame construction.

Normal source: trimesh.proximity.closest_point → nearest face normal.
Adapted from:
  - Noether plane_slicer_raster_planner.cpp :: createTransform
    (https://github.com/ros-industrial/noether)
  - Boundary-Fitting getSamplePoints.m :: offsetObjectivePoint
    (https://github.com/Woomessi/Boundary-Fitting-Approach)
"""
from __future__ import annotations
import numpy as np
import trimesh
import trimesh.proximity as _prox


def _query_for(mesh: trimesh.Trimesh) -> _prox.ProximityQuery:
    """Return a cached ProximityQuery for mesh (builds BVH once per mesh object)."""
    if not hasattr(mesh, '_sg_prox_query'):
        mesh._sg_prox_query = _prox.ProximityQuery(mesh)
    return mesh._sg_prox_query


def interpolate_normals(pts: np.ndarray, mesh: trimesh.Trimesh) -> np.ndarray:
    """Return (N, 3) surface normals at the closest mesh face for each point in pts.

    Uses face normals of the nearest triangle (same approach as BF offsetObjectivePoint
    which takes the normal from the sliced surface point directly).
    """
    _, _, tri_ids = _query_for(mesh).on_surface(pts)
    return mesh.face_normals[tri_ids].copy()


def build_tcp_frames(
    pts: np.ndarray,
    normals: np.ndarray,
) -> np.ndarray:
    """Return (N, 3) corrected path tangents (TCP X-axis) for each waypoint.

    Frame construction adapted from Noether createTransform:
      T_raw  = finite-difference tangent along path
      B      = normalize(cross(N, T_raw))   — binormal (TCP Y-axis)
      T_corr = normalize(cross(B, N))       — corrected tangent, orthogonal to N (TCP X-axis)
      Z-axis = N (nozzle axis, toward surface)

    For the last point, uses the tangent of the previous segment (same as Noether).
    """
    n = len(pts)
    tangents = np.empty_like(pts)

    # finite-difference tangents: forward differences, last point repeats prev segment
    for i in range(n - 1):
        t = pts[i + 1] - pts[i]
        norm = np.linalg.norm(t)
        tangents[i] = t / norm if norm > 1e-9 else tangents[i - 1] if i > 0 else np.array([1., 0., 0.])
    tangents[-1] = tangents[-2] if n > 1 else np.array([1., 0., 0.])

    N = normals / np.linalg.norm(normals, axis=1, keepdims=True).clip(1e-9)
    B = np.cross(N, tangents)
    b_norm = np.linalg.norm(B, axis=1, keepdims=True).clip(1e-9)
    B = B / b_norm
    T_corr = np.cross(B, N)
    t_norm = np.linalg.norm(T_corr, axis=1, keepdims=True).clip(1e-9)
    return T_corr / t_norm
