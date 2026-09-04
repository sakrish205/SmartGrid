"""Straight-line air connectors between adjacent paint passes."""
from __future__ import annotations
import numpy as np
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, Connection


def connect_passes(
    primary_passes: list[PaintPass],
    mesh_data: MeshData,
    region_face_indices: np.ndarray,
    simplify_epsilon: float = 1.0,
) -> list[Connection]:
    """Connect adjacent primary passes with straight-line air moves."""
    if len(primary_passes) < 2:
        return []

    connections: list[Connection] = []
    for i in range(len(primary_passes) - 1):
        end_pt   = primary_passes[i].points[-1]
        start_pt = primary_passes[i + 1].points[0]
        connections.append(Connection(
            id=i,
            from_pass_id=primary_passes[i].id,
            to_pass_id=primary_passes[i + 1].id,
            points=np.array([end_pt, start_pt], dtype=float),
            is_air_move=True,
        ))
    return connections
