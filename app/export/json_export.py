"""Export PaintRoute list to JSON."""
from __future__ import annotations
import json
from datetime import datetime

import numpy as np
from app.path.path_model import PaintRoute


class _NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        return super().default(obj)


def export_route_json(routes: list[PaintRoute], filepath: str) -> None:
    total_passes      = sum(r.total_passes for r in routes)
    total_connections = sum(len(r.connections) for r in routes)
    total_length      = sum(r.total_length_mm for r in routes)
    regions           = sorted({r.region_id for r in routes})
    directions        = sorted({p.direction for r in routes for p in r.passes})

    data = {
        "version": "1.1",
        "author": "Saketha Krishna B S",
        "generated_at": datetime.now().isoformat(timespec='seconds'),
        "summary": {
            "total_routes":       len(routes),
            "total_passes":       total_passes,
            "total_connections":  total_connections,
            "total_length_mm":    round(total_length, 3),
            "regions":            regions,
            "directions":         directions,
        },
        "routes": [_route_to_dict(r, idx) for idx, r in enumerate(routes)],
    }

    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, cls=_NumpyEncoder, indent=2)


def _route_to_dict(route: PaintRoute, route_index: int) -> dict:
    # Infer the single direction for this route (bbox routes are always one direction)
    dirs = {p.direction for p in route.passes}
    route_direction = dirs.pop() if len(dirs) == 1 else 'mixed'

    return {
        "route_index":      route_index,
        "region_id":        route.region_id,
        "direction":        route_direction,
        "unit":             route.unit,
        "spacing_mm":       route.spacing_mm,
        "total_passes":     route.total_passes,
        "total_length_mm":  round(route.total_length_mm, 3),
        "passes": [_pass_to_dict(p) for p in route.passes],
        "connections": [_conn_to_dict(c) for c in route.connections],
    }


def _pass_to_dict(p) -> dict:
    length = float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1))) if len(p.points) >= 2 else 0.0
    return {
        "id":             p.id,
        "region_id":      p.region_id,
        "direction":      p.direction,
        "is_forward":     p.is_forward,
        "sub_index":      p.sub_index,
        "slice_position": round(float(p.slice_position), 4),
        "length_mm":      round(length, 3),
        "start":          p.points[0].tolist(),
        "end":            p.points[-1].tolist(),
        "points":         p.points.tolist(),
    }


def _conn_to_dict(c) -> dict:
    length = float(np.sum(np.linalg.norm(np.diff(c.points, axis=0), axis=1))) if len(c.points) >= 2 else 0.0
    return {
        "id":           c.id,
        "from_pass_id": c.from_pass_id,
        "to_pass_id":   c.to_pass_id,
        "is_air_move":  c.is_air_move,
        "length_mm":    round(length, 3),
        "start":        c.points[0].tolist(),
        "end":          c.points[-1].tolist(),
        "points":       c.points.tolist(),
    }
