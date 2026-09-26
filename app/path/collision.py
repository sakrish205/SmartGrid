"""Collision detection for paint routes.

detect_collisions() returns {pass_id: 'collision' | 'near_miss'} using
trimesh.proximity (already installed — no new dep).

Hard collision  — point inside the mesh (mesh.contains, ray-cast) — watertight only
Near miss       — closest surface point < (standoff - nozzle_radius) — sphere model
Open meshes     — auto-detected via mesh.is_watertight; falls back to closest_point only

# ponytail: mesh.contains uses per-point ray casting — O(N*faces).
#           Replace with signed-distance field if performance matters on dense meshes.
"""
from __future__ import annotations
import logging
import numpy as np
import trimesh.proximity as _prox
from trimesh import Trimesh

from app.path.path_model import PaintRoute

_log = logging.getLogger(__name__)


def detect_collisions(
    routes: list[PaintRoute],
    mesh: Trimesh,
    standoff_mm: float,
    nozzle_radius_mm: float = 0.0,
) -> tuple[dict[int, str], float]:
    """Return ({pass_id: 'collision'|'near_miss'}, max_penetration_mm).

    max_penetration_mm is the deepest inside-point distance to the surface —
    used by the caller to suggest a minimum standoff correction.
    Near-miss threshold = standoff_mm - nozzle_radius_mm (sphere model).
    Hard collision uses mesh.contains — only valid on watertight meshes.
    Open meshes fall back to closest_point-only with the same threshold.
    """
    flagged: dict[int, str] = {}
    max_depth = 0.0

    is_watertight = mesh.is_watertight
    if not is_watertight:
        _log.info('collision: open mesh — using closest_point fallback (no hard-collision check)')

    # Effective minimum distance: gun sphere must not intrude past this
    effective_min = standoff_mm - nozzle_radius_mm
    skip_hard = (standoff_mm == 0.0) or (not is_watertight)

    _CHUNK = 500   # ponytail: VTK ray-cast overflows C++ stack on large arrays; 500 pts is safe

    for route in routes:
        for p in route.passes:
            pts = p.points
            if len(pts) == 0:
                continue

            if not skip_hard:
                inside = np.zeros(len(pts), dtype=bool)
                for _s in range(0, len(pts), _CHUNK):
                    inside[_s:_s + _CHUNK] = mesh.contains(pts[_s:_s + _CHUNK])

                if inside.any():
                    in_pts = pts[inside]
                    _d_arr = np.empty(len(in_pts))
                    for _s in range(0, len(in_pts), _CHUNK):
                        _, _d_arr[_s:_s + _CHUNK], _ = _prox.closest_point(mesh, in_pts[_s:_s + _CHUNK])
                    depth = float(_d_arr.max())
                    if depth > 0.5:   # ignore surface-grazing false positives
                        flagged[p.id] = 'collision'
                        max_depth = max(max_depth, depth)
                    continue

            if effective_min > 0:
                dists = np.empty(len(pts))
                for _s in range(0, len(pts), _CHUNK):
                    _, dists[_s:_s + _CHUNK], _ = _prox.closest_point(mesh, pts[_s:_s + _CHUNK])
                if (dists < effective_min).any():
                    if p.id not in flagged:
                        flagged[p.id] = 'near_miss'

    return flagged, max_depth


def check_orientation(
    routes: list[PaintRoute],
    robot_profile,          # RobotProfile — imported locally to avoid circular dep
) -> dict[int, str]:
    """Return {pass_id: 'orientation_exceeded'} for passes where consecutive
    surface-normal angle change exceeds robot_profile.max_orientation_change_deg.

    Only applies to passes with normals (geodesic mode); others are skipped.
    """
    flagged: dict[int, str] = {}
    cos_limit = float(np.cos(np.radians(robot_profile.max_orientation_change_deg)))
    for route in routes:
        for p in route.passes:
            if p.normals is None or len(p.normals) < 2:
                continue
            dots = np.einsum('ij,ij->i', p.normals[:-1], p.normals[1:])
            dots = np.clip(dots, -1.0, 1.0)
            if (dots < cos_limit).any():
                flagged[p.id] = 'orientation_exceeded'
    return flagged


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
