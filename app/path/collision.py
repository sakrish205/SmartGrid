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
import trimesh
import trimesh.proximity as _prox

from app.path.path_model import PaintRoute


def detect_collisions(
    routes: list[PaintRoute],
    mesh: trimesh.Trimesh,
    standoff_mm: float,
) -> dict[int, str]:
    """Return {pass_id: 'collision'|'near_miss'} for flagged passes.

    Near-miss check is skipped when standoff_mm == 0 (paths on surface by design).
    """
    flagged: dict[int, str] = {}
    near_threshold = standoff_mm * 0.5 if standoff_mm > 0 else 0.0

    for route in routes:
        for p in route.passes:
            pts = p.points
            if len(pts) == 0:
                continue

            # Hard collision: any point inside the mesh
            inside = mesh.contains(pts)
            if inside.any():
                flagged[p.id] = 'collision'
                continue

            # Near miss: closest surface distance below threshold
            if near_threshold > 0:
                _, dists, _ = _prox.closest_point(mesh, pts)
                if (dists < near_threshold).any():
                    flagged[p.id] = 'near_miss'

    return flagged
