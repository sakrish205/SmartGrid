"""Flood-fill selection; holds the selected_face_ids set."""
from __future__ import annotations
import numpy as np
from app.mesh.preprocessor import MeshData


def flood_select_from_face(
    mesh_data: MeshData,
    seed_face: int,
    angle_threshold_deg: float = 30.0,
) -> np.ndarray:
    """BFS from seed_face, expanding to neighbours whose normal is within threshold."""
    threshold_dot = np.cos(np.radians(angle_threshold_deg))
    seed_normal = mesh_data.face_normals[seed_face]

    visited: set[int] = {seed_face}
    queue: list[int] = [seed_face]

    while queue:
        current = queue.pop()
        for neighbour in mesh_data.adjacency.get(current, []):
            if neighbour in visited:
                continue
            if np.dot(mesh_data.face_normals[neighbour], seed_normal) >= threshold_dot:
                visited.add(neighbour)
                queue.append(neighbour)

    return np.array(sorted(visited), dtype=np.int64)
