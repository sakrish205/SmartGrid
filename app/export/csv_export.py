"""Export PaintRoute list to CSV."""
from __future__ import annotations
import csv
from app.path.path_model import PaintRoute

_FIELDS = [
    'segment_type', 'region', 'pass_id', 'is_forward', 'sub_index',
    'conn_id', 'is_air_move', 'pt_idx', 'x', 'y', 'z',
]


def export_route_csv(routes: list[PaintRoute], filepath: str) -> None:
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=_FIELDS, extrasaction='ignore')
        writer.writeheader()
        for route in routes:
            for p in route.passes:
                for i, pt in enumerate(p.points):
                    writer.writerow({
                        'segment_type': 'pass',
                        'region': route.region_id,
                        'pass_id': p.id,
                        'is_forward': p.is_forward,
                        'sub_index': p.sub_index,
                        'pt_idx': i,
                        'x': float(pt[0]),
                        'y': float(pt[1]),
                        'z': float(pt[2]),
                    })
            for c in route.connections:
                for i, pt in enumerate(c.points):
                    writer.writerow({
                        'segment_type': 'connection',
                        'region': route.region_id,
                        'conn_id': c.id,
                        'is_air_move': c.is_air_move,
                        'pt_idx': i,
                        'x': float(pt[0]),
                        'y': float(pt[1]),
                        'z': float(pt[2]),
                    })
