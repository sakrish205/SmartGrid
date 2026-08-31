"""Boundary-following connectors between adjacent paint passes."""
from __future__ import annotations
from collections import defaultdict, deque
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree

from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, Connection


def connect_passes(
    primary_passes: list[PaintPass],
    mesh_data: MeshData,
    region_face_indices: np.ndarray,
) -> list[Connection]:
    """Connect adjacent primary passes by walking the selected-face boundary."""
    if len(primary_passes) < 2:
        return []

    bverts, badj, bpositions = _build_boundary_graph(
        mesh_data.trimesh_mesh, region_face_indices
    )
    if bverts is None:
        return []

    tree = cKDTree(bpositions)
    connections: list[Connection] = []

    for i in range(len(primary_passes) - 1):
        end_pt   = primary_passes[i].points[-1]      # (3,)
        start_pt = primary_passes[i + 1].points[0]   # (3,)

        # Nearest boundary vertices to each pass endpoint
        _, ei = tree.query(end_pt)
        _, si = tree.query(start_pt)
        end_vid   = bverts[ei]
        start_vid = bverts[si]

        bpath = _bfs_walk(end_vid, start_vid, badj)
        if bpath is None:
            continue   # no boundary route — omit this connection

        # Connector: pass endpoint → boundary vertices → next pass start
        walk_pts = mesh_data.trimesh_mesh.vertices[bpath]
        pts = np.vstack([
            end_pt[np.newaxis],
            walk_pts,
            start_pt[np.newaxis],
        ])

        connections.append(Connection(
            id=i,
            from_pass_id=primary_passes[i].id,
            to_pass_id=primary_passes[i + 1].id,
            points=pts,
            is_air_move=False,
        ))

    return connections


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _build_boundary_graph(
    mesh,
    region_face_indices: np.ndarray,
) -> tuple[Optional[list[int]], Optional[dict], Optional[np.ndarray]]:
    """
    Identify the boundary of the selected face set and return it as a graph.

    A boundary edge is one shared by exactly one selected face (the other
    side is either unselected or open mesh boundary).

    Returns
    -------
    bverts    : sorted list of boundary vertex indices
    adj       : dict mapping each boundary vertex → set of adjacent boundary vertices
    positions : (K, 3) float array of vertex positions for bverts
    """
    # Count how many selected faces contain each edge
    edge_count: dict[tuple[int, int], int] = {}
    faces = mesh.faces
    for fi in region_face_indices:
        f = faces[fi]
        for j in range(3):
            a, b = int(f[j]), int(f[(j + 1) % 3])
            edge = (min(a, b), max(a, b))
            edge_count[edge] = edge_count.get(edge, 0) + 1

    boundary_edges = [e for e, c in edge_count.items() if c == 1]
    if not boundary_edges:
        return None, None, None

    adj: dict[int, set[int]] = defaultdict(set)
    for v0, v1 in boundary_edges:
        adj[v0].add(v1)
        adj[v1].add(v0)

    bverts = sorted(adj.keys())
    positions = mesh.vertices[bverts].copy()   # (K, 3)
    return bverts, dict(adj), positions


def _bfs_walk(
    start_vid: int,
    end_vid: int,
    adj: dict[int, set[int]],
    max_depth: int = 10_000,
) -> Optional[list[int]]:
    """Shortest-path BFS through the boundary graph. Returns None if unreachable."""
    if start_vid == end_vid:
        return [start_vid]

    queue: deque[list[int]] = deque([[start_vid]])
    visited: set[int] = {start_vid}
    depth = 0

    while queue:
        path = queue.popleft()
        depth += 1
        if depth > max_depth:
            return None   # boundary too large — skip this connection
        for nb in adj.get(path[-1], set()):
            if nb == end_vid:
                return path + [nb]
            if nb not in visited:
                visited.add(nb)
                queue.append(path + [nb])

    return None
