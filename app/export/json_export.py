"""Export PaintRoute list to JSON."""
from __future__ import annotations
import json
import numpy as np
from app.path.path_model import PaintRoute


class _NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        return super().default(obj)


def export_route_json(routes: list[PaintRoute], filepath: str) -> None:
    data = {
        "version": "1.0",
        "routes": [_route_to_dict(r) for r in routes],
    }
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, cls=_NumpyEncoder, indent=2)


def _route_to_dict(route: PaintRoute) -> dict:
    return {
        "region_id": route.region_id,
        "unit": route.unit,
        "spacing_mm": route.spacing_mm,
        "total_passes": route.total_passes,
        "total_length_mm": route.total_length_mm,
        "passes": [
            {
                "id": p.id,
                "direction": p.direction,
                "is_forward": p.is_forward,
                "sub_index": p.sub_index,
                "slice_position": p.slice_position,
                "points": p.points,
            }
            for p in route.passes
        ],
        "connections": [
            {
                "id": c.id,
                "from_pass_id": c.from_pass_id,
                "to_pass_id": c.to_pass_id,
                "is_air_move": c.is_air_move,
                "points": c.points,
            }
            for c in route.connections
        ],
    }
