"""Collision detection for paint routes.

detect_collisions() returns {pass_id: 'collision' | 'near_miss'} using
trimesh.proximity (already installed — no new dep).

Hard collision  — point inside the mesh (mesh.contains, ray-cast)
Near miss       — closest surface point < standoff * 0.5

# ponytail: mesh.contains uses per-point ray casting — O(N*faces).
#           Replace with signed-distance field if performance matters on dense meshes.
"""
from __future__ import annotations
import numpy as np
import trimesh.proximity as _prox
from trimesh import Trimesh

from app.path.path_model import PaintRoute


def detect_collisions(
    routes: list[PaintRoute],
    mesh: Trimesh,
    standoff_mm: float,
) -> tuple[dict[int, str], float]:
    """Return ({pass_id: 'collision'|'near_miss'}, max_penetration_mm).

    max_penetration_mm is the deepest inside-point distance to the surface —
    used by the caller to suggest a minimum standoff correction.
    Near-miss check is skipped when standoff_mm == 0 (paths on surface by design).
    """
    flagged: dict[int, str] = {}
    near_threshold = standoff_mm * 0.5 if standoff_mm > 0 else 0.0
    max_depth = 0.0
    # When standoff is 0 paths sit on the surface by design — mesh.contains()
    # on surface points is a boundary case and produces false positives.
    skip_hard = (standoff_mm == 0.0)

    _CHUNK = 500   # ponytail: VTK ray-cast overflows C++ stack on large arrays; 500 pts is safe

    for route in routes:
        for p in route.passes:
            pts = p.points
            if len(pts) == 0:
                continue

            if skip_hard:
                pass  # on-surface paths never trigger hard collision
            else:
                # Chunk to avoid VTK stack overflow on large point arrays
                inside = np.zeros(len(pts), dtype=bool)
                for _start in range(0, len(pts), _CHUNK):
                    inside[_start:_start + _CHUNK] = mesh.contains(pts[_start:_start + _CHUNK])

            if not skip_hard and inside.any():
                in_pts = pts[inside]
                _d_arr = np.empty(len(in_pts))
                for _s in range(0, len(in_pts), _CHUNK):
                    _, _d_arr[_s:_s + _CHUNK], _ = _prox.closest_point(mesh, in_pts[_s:_s + _CHUNK])
                depth = float(_d_arr.max())
                if depth > 0.5:   # ignore surface-grazing false positives on open meshes
                    flagged[p.id] = 'collision'
                    max_depth = max(max_depth, depth)
                continue

            if near_threshold > 0:
                dists = np.empty(len(pts))
                for _start in range(0, len(pts), _CHUNK):
                    _, dists[_start:_start + _CHUNK], _ = _prox.closest_point(
                        mesh, pts[_start:_start + _CHUNK])
                if (dists < near_threshold).any():
                    flagged[p.id] = 'near_miss'

    return flagged, max_depth


def detect_overlaps(routes: list[PaintRoute]) -> dict[int, str]:
    """Flag passes from different regions whose points come within spacing_mm/2 of each other.

    Returns {pass_id: 'overlap'}.  Uses a KDTree per region pair — O(N log N).
    Only meaningful when routes contain passes from more than one region_id.
    """
    from scipy.spatial import cKDTree

    # Group passes by region_id (preserved per-pass even in merged routes)
    regions: dict[str, list] = {}
    spacing = 50.0
    for route in routes:
        if route.spacing_mm > 0:
            spacing = route.spacing_mm
        for p in route.passes:
            if len(p.points) > 0:
                regions.setdefault(p.region_id, []).append(p)

    region_keys = list(regions.keys())
    if len(region_keys) < 2:
        return {}

    threshold = spacing * 0.5
    flagged: dict[int, str] = {}

    for i in range(len(region_keys)):
        for j in range(i + 1, len(region_keys)):
            passes_a = regions[region_keys[i]]
            passes_b = regions[region_keys[j]]

            pts_b = np.vstack([p.points for p in passes_b])
            pid_b = np.concatenate([np.full(len(p.points), p.id, dtype=np.int64) for p in passes_b])
            tree_b = cKDTree(pts_b)

            for pa in passes_a:
                if pa.id in flagged:
                    continue
                dists, idxs = tree_b.query(pa.points, k=1)
                hit = np.where(dists < threshold)[0]
                if len(hit):
                    flagged[pa.id] = 'overlap'
                    flagged[int(pid_b[idxs[hit[0]]])] = 'overlap'

    return flagged
