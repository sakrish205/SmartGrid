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

    # --- Steps 1-2: Quantize + assign node IDs via np.unique (vectorized) ---
    scale = 1.0 / tolerance
    quant = (segments * scale).round().astype(np.int64)  # (N, 2, 3)

    all_keys   = quant.reshape(-1, 3)        # (2N, 3) int64
    all_coords = segments.reshape(-1, 3)     # (2N, 3) float
    _, node_ids = np.unique(all_keys, axis=0, return_inverse=True)   # (2N,)
    _, first_occ = np.unique(node_ids, return_index=True)            # one coord per node
    node_to_coord = all_coords[first_occ]                            # (n_nodes, 3)
    seg_nodes_arr = node_ids.reshape(n_segs, 2)                      # (N, 2)

    # --- Step 3: Build adjacency list ---
    # adj[node] = list of (neighbour_node, segment_index)
    adj: dict[int, list[tuple[int, int]]] = defaultdict(list)
    for seg_idx in range(n_segs):
        a, b = int(seg_nodes_arr[seg_idx, 0]), int(seg_nodes_arr[seg_idx, 1])
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
        a, b = int(seg_nodes_arr[seg_i, 0]), int(seg_nodes_arr[seg_i, 1])
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
        result.append(node_to_coord[np.array(chain)])

    return result
