"""Straight-line air connectors between adjacent paint passes.

Connects every pass in execution order (including sub-index passes for holes).
Picks the nearest endpoint pair to minimise connector travel distance.
Skips connectors that would exceed the maximum allowed distance to avoid
wild zigzags caused by noise fragments on complex mesh geometry.
"""
from __future__ import annotations
import numpy as np
from app.path.path_model import PaintPass, Connection
from app.path.resampler import resample_arc

_MAX_CONNECTOR_FACTOR = 3.0   # connector distance must be ≤ this × spray_width_mm
_MAX_CONNECTOR_ABS_MM = 400.0 # hard cap regardless of spray width


def reorder_passes_nearest_neighbor(passes: list[PaintPass]) -> list[PaintPass]:
    """Reorder (and orient) passes via greedy nearest-neighbor walk, so
    connect_passes's connector lines follow actual spatial proximity instead of
    generation order.

    connect_passes only ever connects passes[i] to passes[i+1] — list position,
    not geometry. When the list order comes from "all of component A's passes,
    then all of component B's passes" (Mesh Flow's per-component generation,
    core_generator._region_components, ordered by panel area) that boundary has
    nothing to do with which panel is physically nearest — the connector at that
    seam can jump straight across the part. This is the standard greedy
    approximation to the shortest-Hamiltonian-path problem (open-path TSP): at
    each step, walk to whichever remaining pass's nearest endpoint is actually
    closest, re-measured with the same np.linalg.norm distance connect_passes
    already trusts locally, just searched over every remaining pass instead of
    only the next list entry. Not globally optimal (no polynomial algorithm is,
    for true TSP) — but it removes the specific defect of a connector crossing
    the part because of list order, which is the one observed in practice.
    O(n^2); fine for Mesh Flow's per-region pass counts (tens to low hundreds).
    """
    if len(passes) < 2:
        return passes

    remaining = list(passes)
    start_idx = min(range(len(remaining)), key=lambda i: remaining[i].slice_position)
    ordered = [remaining.pop(start_idx)]

    while remaining:
        cur_end = ordered[-1].points[-1]
        best_idx = 0
        best_dist = None
        best_reverse = False
        for i, p in enumerate(remaining):
            d_start = float(np.linalg.norm(cur_end - p.points[0]))
            d_end = float(np.linalg.norm(cur_end - p.points[-1]))
            d, reverse = (d_start, False) if d_start <= d_end else (d_end, True)
            if best_dist is None or d < best_dist:
                best_dist, best_idx, best_reverse = d, i, reverse

        nxt = remaining.pop(best_idx)
        if best_reverse:
            nxt.points = nxt.points[::-1].copy()
            if nxt.normals is not None:
                nxt.normals = nxt.normals[::-1].copy()
            if nxt.tangent is not None:
                nxt.tangent = nxt.tangent[::-1].copy()
            nxt.is_forward = not nxt.is_forward
        ordered.append(nxt)

    return ordered


def connect_passes(
    passes: list[PaintPass],
    spray_width_mm: float = 100.0,
    waypoint_spacing_mm: float = 0.0,
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
            nxt.points = nxt.points[::-1].copy()
            nxt.is_forward = not nxt.is_forward

        start_pt = nxt.points[0]
        pts = np.array([end_pt, start_pt], dtype=float)
        if waypoint_spacing_mm > 0.0:
            pts = resample_arc(pts, waypoint_spacing_mm)
        connections.append(Connection(
            id=conn_id,
            from_pass_id=cur.id,
            to_pass_id=nxt.id,
            points=pts,
            is_air_move=True,
        ))
        conn_id += 1

    return connections
