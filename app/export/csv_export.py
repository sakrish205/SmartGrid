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
    'segment_type',    # 'pass' | 'connection' | 'waypoint'
    'route_index',     # which route (0 = first, etc.)
    'region',          # bbox face: TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT
    'direction',       # 'horizontal' | 'vertical' | '' (connections)
    'pass_id',         # unique pass number within the route
    'is_forward',      # True = forward sweep, False = reversed
    'sub_index',       # 0 = main pass; >0 = extra chain at same slice (holes)
    'conn_id',         # connection id (blank for passes/waypoints)
    'is_air_move',     # True = gun off travel (blank for passes/waypoints)
    'sweep_direction', # 'CW' or 'CCW' — overall sweep direction for this route
    'length_mm',       # total length of this pass/connection (written on pt_idx=0 only)
    'arrow_direction', # unit vector 'dx,dy,dz' of travel direction (pt_idx=0 only, passes only)
    'pt_idx',          # point index within the segment (0 = start, 1 = end, ...)
    'x', 'y', 'z',    # world coordinates in mm
]


def export_route_csv(routes: list[PaintRoute], filepath: str) -> None:
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=_FIELDS, extrasaction='ignore')
        writer.writeheader()

        for route_idx, route in enumerate(routes):
            # Derive sweep direction from the first pass in this route
            first_pass = next((p for p in route.passes if p.sub_index == 0), None)
            sweep_dir = 'CW' if (first_pass is None or first_pass.is_forward) else 'CCW'

            for p in route.passes:
                seg_len = (
                    round(float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1))), 3)
                    if len(p.points) >= 2 else 0.0
                )
                arrow_dir_str = _arrow_dir(p.points) if len(p.points) >= 2 else ''

                for i, pt in enumerate(p.points):
                    writer.writerow({
                        'segment_type':    'pass',
                        'route_index':     route_idx,
                        'region':          route.region_id,
                        'direction':       p.direction,
                        'pass_id':         p.id,
                        'is_forward':      p.is_forward,
                        'sub_index':       p.sub_index,
                        'conn_id':         '',
                        'is_air_move':     '',
                        'sweep_direction': sweep_dir,
                        'length_mm':       seg_len if i == 0 else '',
                        'arrow_direction': arrow_dir_str if i == 0 else '',
                        'pt_idx':          i,
                        'x': round(float(pt[0]), 4),
                        'y': round(float(pt[1]), 4),
                        'z': round(float(pt[2]), 4),
                    })

            # Waypoint rows — pure TCP positions for OLP import.
            # Contains only pass_id + pt_idx + xyz; all other metadata lives in
            # the corresponding 'pass' rows above (join on route_index + pass_id).
            for p in route.passes:
                for i, pt in enumerate(p.points):
                    writer.writerow({
                        'segment_type': 'waypoint',
                        'route_index':  route_idx,
                        'pass_id':      p.id,
                        'pt_idx':       i,
                        'x': round(float(pt[0]), 4),
                        'y': round(float(pt[1]), 4),
                        'z': round(float(pt[2]), 4),
                    })

            for c in route.connections:
                seg_len = (
                    round(float(np.sum(np.linalg.norm(np.diff(c.points, axis=0), axis=1))), 3)
                    if len(c.points) >= 2 else 0.0
                )
                for i, pt in enumerate(c.points):
                    writer.writerow({
                        'segment_type':    'connection',
                        'route_index':     route_idx,
                        'region':          route.region_id,
                        'direction':       '',
                        'pass_id':         '',
                        'is_forward':      '',
                        'sub_index':       '',
                        'conn_id':         c.id,
                        'is_air_move':     c.is_air_move,
                        'sweep_direction': sweep_dir,
                        'length_mm':       seg_len if i == 0 else '',
                        'arrow_direction': '',
                        'pt_idx':          i,
                        'x': round(float(pt[0]), 4),
                        'y': round(float(pt[1]), 4),
                        'z': round(float(pt[2]), 4),
                    })


def _arrow_dir(points: np.ndarray) -> str:
    """Return normalized travel direction as 'dx,dy,dz' string, 4 dp."""
    d = np.asarray(points[1], dtype=float) - np.asarray(points[0], dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-9:
        return '0,0,0'
    d = d / n
    return f'{d[0]:.4f},{d[1]:.4f},{d[2]:.4f}'
