"""Export PaintRoute list as a standalone 3D mesh (.stl) — the path only,
not the source part. Every pass and connection is swept into a thin tube
so the toolpath is visible/reusable as ordinary mesh geometry in any
CAD/mesh tool.
"""
from __future__ import annotations

import numpy as np
import trimesh
from shapely.geometry import Point

from app.path.path_model import PaintRoute

_DEFAULT_TUBE_RADIUS_MM = 2.0


def _tube(points: np.ndarray, radius_mm: float) -> trimesh.Trimesh | None:
    """Sweep a small circular cross-section along points; None if too short to form a tube."""
    if points is None or len(points) < 2:
        return None
    poly = Point(0.0, 0.0).buffer(radius_mm, quad_segs=8)
    return trimesh.creation.sweep_polygon(poly, np.asarray(points, dtype=float))


def export_route_mesh(
    routes: list[PaintRoute],
    filepath: str,
    tube_radius_mm: float = _DEFAULT_TUBE_RADIUS_MM,
) -> None:
    """Write every pass and connection in `routes` as tube geometry to `filepath` (.stl)."""
    tubes = []
    for route in routes:
        for p in route.passes:
            t = _tube(p.points, tube_radius_mm)
            if t is not None:
                tubes.append(t)
        for c in route.connections:
            t = _tube(c.points, tube_radius_mm)
            if t is not None:
                tubes.append(t)

    if not tubes:
        raise ValueError('No path geometry to export — generate a path first.')

    mesh = trimesh.util.concatenate(tubes)
    mesh.export(filepath)
