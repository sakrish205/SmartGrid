"""Sort unordered line segments into ordered polylines."""
from __future__ import annotations
from collections import defaultdict
import numpy as np


def stitch_segments(
    segments: np.ndarray,
    tolerance: float = 1e-4,   # 0.1 µm — safe for mm-scale meshes (was 1e-6, too tight)
) -> list[np.ndarray]:
    """Take (N, 2, 3) unordered segments, return list of ordered (M, 3) polylines.

    Multiple polylines are returned when segments form disconnected chains (holes).
    """
    if segments is None or len(segments) == 0:
        return []

    # Remove zero-length segments
    lengths = np.linalg.norm(segments[:, 1, :] - segments[:, 0, :], axis=1)
    segments = segments[lengths > 1e-12]
    if len(segments) == 0:
        return []

    n_segs = len(segments)

    # --- Step 1: Quantize endpoints to an integer grid ---
    # This turns floating-point proximity into exact key equality.
    scale = 1.0 / tolerance
    quant = (segments * scale).round().astype(np.int64)  # (N, 2, 3)

    # --- Step 2: Build node map (quantized tuple -> node id) ---
    point_to_node: dict[tuple, int] = {}
    node_to_coord: list[np.ndarray] = []

    def get_node(key: tuple, coord: np.ndarray) -> int:
        if key not in point_to_node:
            nid = len(point_to_node)
            point_to_node[key] = nid
            node_to_coord.append(coord)
        return point_to_node[key]

    seg_nodes: list[tuple[int, int]] = []
    for i in range(n_segs):
        key_a = tuple(quant[i, 0])
        key_b = tuple(quant[i, 1])
        node_a = get_node(key_a, segments[i, 0])
        node_b = get_node(key_b, segments[i, 1])
        seg_nodes.append((node_a, node_b))

    # --- Step 3: Build adjacency list ---
    # adj[node] = list of (neighbour_node, segment_index)
    adj: dict[int, list[tuple[int, int]]] = defaultdict(list)
    for seg_idx, (a, b) in enumerate(seg_nodes):
        if a != b:  # skip zero-length after quantisation
            adj[a].append((b, seg_idx))
            adj[b].append((a, seg_idx))

    # --- Step 4: Find chain starts (degree-1 nodes = open endpoints) ---
    degree = {nid: len(neighbours) for nid, neighbours in adj.items()}
    starts = [nid for nid, d in degree.items() if d == 1]

    # If no degree-1 nodes exist all chains are closed loops; start anywhere
    if not starts:
        starts = [next(iter(adj))]

    # --- Step 5: Walk chains ---
    visited_segs: set[int] = set()
    chains: list[list[int]] = []

    for start in starts:
        # Skip if all edges from this start are already consumed
        if all(seg_i in visited_segs for _, seg_i in adj[start]):
            continue

        chain: list[int] = [start]
        while True:
            current = chain[-1]
            prev = chain[-2] if len(chain) > 1 else -1
            moved = False
            for neighbour, seg_i in adj[current]:
                if seg_i not in visited_segs and neighbour != prev:
                    visited_segs.add(seg_i)
                    chain.append(neighbour)
                    moved = True
                    break
            if not moved:
                break

        if len(chain) >= 2:
            chains.append(chain)

    # --- Step 6: Handle any remaining unvisited segments (closed loops) ---
    for seg_i in range(n_segs):
        if seg_i in visited_segs:
            continue
        a, b = seg_nodes[seg_i]
        chain = [a]
        visited_segs.add(seg_i)
        current = b
        chain.append(current)
        while True:
            moved = False
            prev = chain[-2]
            for neighbour, si in adj[current]:
                if si not in visited_segs and neighbour != prev:
                    visited_segs.add(si)
                    chain.append(neighbour)
                    current = neighbour
                    moved = True
                    break
            if not moved:
                break
        if len(chain) >= 2:
            chains.append(chain)

    # --- Step 7: Convert node-id chains back to float coordinates ---
    result: list[np.ndarray] = []
    for chain in chains:
        pts = np.array([node_to_coord[nid] for nid in chain])
        result.append(pts)

    return result
