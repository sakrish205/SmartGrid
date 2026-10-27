"""Mesh Flow Phase 6 — general-purpose surface-safety utilities (F56-F59, C1-C3).

Confirmed gap: no general signed-distance/connected-component utility existed
outside ad hoc call sites (app.path.face_grid_generator._clip_inside) before
this module.
"""
from __future__ import annotations
import numpy as np
import trimesh
import trimesh.proximity as _prox


def closest_point_distance(points: np.ndarray, mesh: trimesh.Trimesh) -> np.ndarray:
    """F56 — thin named wrapper around the same trimesh.proximity.closest_point
    call app.path.validator.py:85 and app.path.collision.py:66,76 already make."""
    _closest, dists, _tri = _prox.closest_point(mesh, points)
    return dists


def inside_mesh_mask(points: np.ndarray, mesh: trimesh.Trimesh) -> np.ndarray | None:
    """F57/C2 — returns None (indeterminate) when the mesh isn't watertight,
    matching the existing guard in app.path.validator.py:60 and
    app.path.collision.py:41-47, but as an explicit tri-state the caller must
    handle, instead of a silent skip."""
    if not mesh.is_watertight:
        return None
    return mesh.contains(points)


def segment_penetrates(p0: np.ndarray, p1: np.ndarray, mesh: trimesh.Trimesh) -> bool | None:
    """F58/C3 — EXACT geometric segment/triangle intersection, not discrete
    sampling. A fixed sample count has a structural gap: a segment can dip into
    the mesh and back out between any two consecutive samples no matter how many
    samples are used, and here the consequence is a missed safety violation, not
    an imprecise diagnostic — sampling is the wrong tool, not just an imprecise
    one. Uses trimesh's BVH-accelerated ray-triangle intersection fired from p0
    toward p1, keeping only intersections strictly within the segment's bounds
    (not beyond p1) — this checks the entire continuous segment against every
    triangle, not a finite sample set.

    Returns True if the segment provably crosses the mesh surface at least once
    within its bounds (a 'hard' finding regardless of watertightness — a chord
    crossing the surface at all is already the concern, independent of whether
    "inside" is well-defined); False if provably zero crossings; None if the
    intersection test itself could not run (e.g. mesh.ray unavailable, or a
    degenerate zero-length segment) — callers must map None to 'incomplete',
    never to a silent False.

    Side benefit: unlike inside_mesh_mask, this does NOT require a watertight
    mesh — a crossing is a crossing on any triangulated surface, open or closed.
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    seg_vec = p1 - p0
    seg_len = float(np.linalg.norm(seg_vec))
    if seg_len < 1e-9:
        return None   # degenerate zero-length segment — nothing to test

    try:
        locations, _index_ray, _index_tri = mesh.ray.intersects_location(
            ray_origins=p0[None, :], ray_directions=seg_vec[None, :],
        )
    except Exception:
        return None   # ray backend unavailable — cannot run the test at all

    if len(locations) == 0:
        return False

    t = np.dot(locations - p0, seg_vec) / (seg_len ** 2)
    on_segment = (t > 1e-9) & (t < 1.0 - 1e-9)
    return bool(on_segment.any())


def connected_components(mesh: trimesh.Trimesh, face_indices: np.ndarray) -> list[np.ndarray]:
    """Generalizes the ad hoc trimesh.graph.connected_component_labels use in
    app.path.geodesic_generator (currently only for seeding) into a reusable
    region-splitting utility."""
    if len(face_indices) == 0:
        return []
    sub_adjacency_mask = np.isin(mesh.face_adjacency[:, 0], face_indices) & \
        np.isin(mesh.face_adjacency[:, 1], face_indices)
    sub_edges = mesh.face_adjacency[sub_adjacency_mask]

    face_to_local = {int(f): i for i, f in enumerate(face_indices)}
    local_edges = np.array([
        [face_to_local[int(a)], face_to_local[int(b)]] for a, b in sub_edges
    ]) if len(sub_edges) else np.zeros((0, 2), dtype=np.int64)

    labels = trimesh.graph.connected_component_labels(local_edges, node_count=len(face_indices))
    components: list[np.ndarray] = []
    for label in np.unique(labels):
        components.append(face_indices[labels == label])
    return components


def outward_tcp_offset(
    surface_point: np.ndarray,
    outward_normal: np.ndarray,
    standoff_mm: float,
) -> np.ndarray:
    """F59/C1 — p_TCP = q_surface + h*n_hat, h > 0. Distinguishes the surface
    point q from the TCP p_TCP per C1 ("the painting target belongs on the
    surface; the gun itself stays outside it"). Mesh Flow's equivalent of the
    inline math in app.main_window._offset_route_by_standoff (lines 288-334),
    which stays untouched for Mesh Surface."""
    n = np.asarray(outward_normal, dtype=float)
    n = n / np.linalg.norm(n)
    return np.asarray(surface_point, dtype=float) + standoff_mm * n
