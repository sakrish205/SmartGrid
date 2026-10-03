"""First pass generation: plane-intersect the outer skin and offset by standoff."""
from __future__ import annotations
from collections import defaultdict

import numpy as np
import trimesh


def _chain_segments(lines: np.ndarray) -> list:
    """Order (N, 2, 3) segments into connected polylines (list of (M, 3) arrays).

    ponytail: O(N) dict lookup per step — fine for typical intersection-line
              sizes (<1000 segments). Upgrade to union-find if profiler says so.
    """
    if len(lines) == 0:
        return []
    scale = float(np.abs(lines).max() or 1.0)
    snap  = 1e-6 * scale

    def key(pt):
        return tuple((pt / snap).round().astype(np.int64))

    adj  = defaultdict(list)
    for i, seg in enumerate(lines):
        adj[key(seg[0])].append((i, 0))
        adj[key(seg[1])].append((i, 1))

    used   = np.zeros(len(lines), dtype=bool)
    chains = []
    for start in range(len(lines)):
        if used[start]:
            continue
        used[start] = True
        chain = [lines[start, 0], lines[start, 1]]
        while True:
            k = key(chain[-1])
            extended = False
            for seg_i, end in adj[k]:
                if used[seg_i]:
                    continue
                used[seg_i] = True
                chain.append(lines[seg_i, 1 - end])
                extended = True
                break
            if not extended:
                break
        chains.append(np.array(chain))
    return chains


def first_pass_line(
    mesh: trimesh.Trimesh,
    outer_face_ids: np.ndarray,
    standoff_mm: float,
    pitch_mm: float,
    direction: str = 'horizontal',
    up_axis: int = 2,
) -> np.ndarray:
    """Return (N, 3) gun-space waypoints for the first pass on the outer skin.

    1. Sub-mesh the outer skin faces.
    2. Place a plane at pitch_mm/2 from the lower edge along the slicing axis.
    3. Intersect that plane with the sub-mesh.
    4. For each intersection point find the nearest point on the FULL mesh
       and its face normal (full mesh gives accurate normals near skin edges).
    5. Offset each surface point outward by standoff_mm along its normal.

    Raises ValueError if the plane misses the outer skin entirely.
    Prints a warning and uses the longest segment if the intersection is
    disconnected (e.g. a hole in the panel).
    """
    print(f'[first_pass_line] outer_face_ids={len(outer_face_ids)} / {len(mesh.faces)} total faces')
    sub = mesh.submesh([outer_face_ids], append=True)

    if direction == 'horizontal':
        axis = up_axis
    elif direction == 'vertical':
        remaining = [i for i in range(3) if i != up_axis]
        extents   = [float(sub.vertices[:, i].max() - sub.vertices[:, i].min())
                     for i in remaining]
        axis = remaining[int(np.argmax(extents))]
    else:
        raise ValueError(f"direction must be 'horizontal' or 'vertical', got {direction!r}")

    lo        = float(sub.vertices[:, axis].min())
    hi        = float(sub.vertices[:, axis].max())
    plane_pos = lo + pitch_mm / 2.0

    if plane_pos > hi:
        raise ValueError(
            f"first_pass_line: first plane at {plane_pos:.1f} mm exceeds "
            f"outer-skin extent {lo:.1f}–{hi:.1f} mm along axis {axis}"
        )

    plane_nrm        = np.zeros(3)
    plane_nrm[axis]  = 1.0
    plane_orig       = np.zeros(3)
    plane_orig[axis] = plane_pos
    print(f'[first_pass_line] axis={axis}  lo={lo:.1f}  hi={hi:.1f}  plane_pos={plane_pos:.1f}')

    lines = trimesh.intersections.mesh_plane(sub, plane_nrm, plane_orig)

    if lines is None or len(lines) == 0:
        raise ValueError(
            f"first_pass_line: no intersection at axis-{axis} = {plane_pos:.1f} mm "
            f"(outer-skin extent {lo:.1f}–{hi:.1f} mm)"
        )

    chains = _chain_segments(lines)
    chain_lens = [len(c) for c in chains]
    print(f'[first_pass_line] chains={len(chains)}  lengths={chain_lens}')
    if len(chains) > 1:
        print(f'[first_pass_line] disconnected — using longest chain ({max(chain_lens)} pts)')
        chains = [max(chains, key=len)]

    line_pts            = chains[0]
    aim_pts, _, tri_ids = trimesh.proximity.closest_point(mesh, line_pts)
    normals             = mesh.face_normals[tri_ids]
    return aim_pts + standoff_mm * normals
