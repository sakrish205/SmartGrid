"""Straight-line air connectors between adjacent paint passes.

Connects every pass in execution order (including sub-index passes for holes).
Picks the nearest endpoint pair to minimize connector travel distance.
"""
from __future__ import annotations
import numpy as np
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, Connection


def connect_passes(
    passes: list[PaintPass],
    mesh_data: MeshData,
    region_face_indices: np.ndarray,
    simplify_epsilon: float = 1.0,
) -> list[Connection]:
    """Connect every consecutive pass with a straight-line air move.

    Picks the endpoint pair (end→start or end→end) that minimises connector
    distance. When end→end is shorter the next pass's points are reversed
    so the connector arrives at the correct entry point.
    """
    if len(passes) < 2:
        return []

    connections: list[Connection] = []
    for i in range(len(passes) - 1):
        cur = passes[i]
        nxt = passes[i + 1]

        end_pt = cur.points[-1]

        # Choose the nearest endpoint of the next pass
        dist_to_start = float(np.linalg.norm(end_pt - nxt.points[0]))
        dist_to_end   = float(np.linalg.norm(end_pt - nxt.points[-1]))

        if dist_to_end < dist_to_start:
            # Reverse next pass so the connector goes to the nearer endpoint
            nxt.points    = nxt.points[::-1].copy()
            nxt.is_forward = not nxt.is_forward

        start_pt = nxt.points[0]
        connections.append(Connection(
            id=i,
            from_pass_id=cur.id,
            to_pass_id=nxt.id,
            points=np.array([end_pt, start_pt], dtype=float),
            is_air_move=True,
        ))

    return connections
