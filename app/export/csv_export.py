"""Export PaintRoute list to CSV.

One row = one 3D point. Passes and connections are written in route order.
Filter by `segment_type` to separate spraying from travel moves.
Filter by `direction` to separate horizontal from vertical passes.
"""
from __future__ import annotations
import csv

import numpy as np
from app.path.path_model import PaintRoute

_FIELDS = [
    'segment_type',   # 'pass' | 'connection'
    'route_index',    # which route (0 = first, 1 = second for crosshatch, etc.)
    'region',         # bbox face: TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT
    'direction',      # 'horizontal' | 'vertical' | '' (connections)
    'pass_id',        # unique pass number within the route
    'is_forward',     # True = forward sweep, False = reversed
    'sub_index',      # 0 = main pass; >1 = extra chain at same slice (holes)
    'conn_id',        # connection id (blank for passes)
    'is_air_move',    # True = gun off travel (blank for passes)
    'length_mm',      # total length of this pass/connection (written on pt_idx=0 only)
    'pt_idx',         # point index within the segment (0 = start, 1 = end, ...)
    'x', 'y', 'z',   # world coordinates in mm
]


def export_route_csv(routes: list[PaintRoute], filepath: str) -> None:
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=_FIELDS, extrasaction='ignore')
        writer.writeheader()

        for route_idx, route in enumerate(routes):
            for p in route.passes:
                seg_len = (
                    round(float(np.linalg.norm(p.points[-1] - p.points[0])), 3)
                    if len(p.points) >= 2 else 0.0
                )
                for i, pt in enumerate(p.points):
                    writer.writerow({
                        'segment_type': 'pass',
                        'route_index':  route_idx,
                        'region':       route.region_id,
                        'direction':    p.direction,
                        'pass_id':      p.id,
                        'is_forward':   p.is_forward,
                        'sub_index':    p.sub_index,
                        'conn_id':      '',
                        'is_air_move':  '',
                        'length_mm':    seg_len if i == 0 else '',
                        'pt_idx':       i,
                        'x': round(float(pt[0]), 4),
                        'y': round(float(pt[1]), 4),
                        'z': round(float(pt[2]), 4),
                    })

            for c in route.connections:
                seg_len = (
                    round(float(np.linalg.norm(c.points[-1] - c.points[0])), 3)
                    if len(c.points) >= 2 else 0.0
                )
                for i, pt in enumerate(c.points):
                    writer.writerow({
                        'segment_type': 'connection',
                        'route_index':  route_idx,
                        'region':       route.region_id,
                        'direction':    '',
                        'pass_id':      '',
                        'is_forward':   '',
                        'sub_index':    '',
                        'conn_id':      c.id,
                        'is_air_move':  c.is_air_move,
                        'length_mm':    seg_len if i == 0 else '',
                        'pt_idx':       i,
                        'x': round(float(pt[0]), 4),
                        'y': round(float(pt[1]), 4),
                        'z': round(float(pt[2]), 4),
                    })
