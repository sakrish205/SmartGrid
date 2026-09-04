"""Straight-line air connectors between adjacent paint passes.

Connects every pass in execution order (including sub-index passes for holes).
Picks the nearest endpoint pair to minimise connector travel distance.
Skips connectors that would exceed the maximum allowed distance to avoid
wild zigzags caused by noise fragments on complex mesh geometry.
"""
from __future__ import annotations
import numpy as np
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, Connection

_MAX_CONNECTOR_FACTOR = 3.0   # connector distance must be ≤ this × spray_width_mm
_MAX_CONNECTOR_ABS_MM = 400.0 # hard cap regardless of spray width


def connect_passes(
    passes: list[PaintPass],
    mesh_data: MeshData,
    region_face_indices: np.ndarray,
    simplify_epsilon: float = 1.0,
    spray_width_mm: float = 100.0,
) -> list[Connection]:
    """Connect consecutive passes with straight-line air moves.

    Picks the endpoint pair (end→start or end→end) that minimises connector
    distance. When end→end is shorter, the next pass's points are reversed.
    Connectors longer than the threshold are skipped to suppress zigzag
    artefacts from noise fragments on complex meshes.
    """
    if len(passes) < 2:
        return []

    max_dist = min(spray_width_mm * _MAX_CONNECTOR_FACTOR, _MAX_CONNECTOR_ABS_MM)

    connections: list[Connection] = []
    conn_id = 0
    for i in range(len(passes) - 1):
        cur = passes[i]
        nxt = passes[i + 1]

        end_pt = cur.points[-1]

        dist_to_start = float(np.linalg.norm(end_pt - nxt.points[0]))
        dist_to_end   = float(np.linalg.norm(end_pt - nxt.points[-1]))

        # Skip connector if it would be absurdly long (noise fragment far away)
        best_dist = min(dist_to_start, dist_to_end)
        if best_dist > max_dist:
            continue

        if dist_to_end < dist_to_start:
            nxt.points    = nxt.points[::-1].copy()
            nxt.is_forward = not nxt.is_forward

        start_pt = nxt.points[0]
        connections.append(Connection(
            id=conn_id,
            from_pass_id=cur.id,
            to_pass_id=nxt.id,
            points=np.array([end_pt, start_pt], dtype=float),
            is_air_move=True,
        ))
        conn_id += 1

    return connections
