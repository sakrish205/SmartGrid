"""Export PaintRoute list to CSV for OLP software.

Rows are written in robot execution order:
  pass points → connector points → next pass points → …

Each row is one TCP waypoint. Key columns for OLP import:
  seq_id      – global sequence number (robot program line order)
  segment_type – 'pass' (spray move) or 'connection' (air move)
  is_spray    – True while gun is on, False for air travel
  x, y, z    – TCP position in mm

Metadata columns (region, direction, pass_id, etc.) allow grouping/filtering
in Excel or a preprocessing script.
"""
from __future__ import annotations
import csv

import numpy as np
from app.path.path_model import PaintRoute

_FIELDS = [
    'seq_id',           # global execution order (0, 1, 2, …)
    'segment_type',     # 'pass' | 'connection'
    'is_spray',         # True = spray gun ON, False = air travel
    'route_index',      # which route (0 = first, etc.)
    'region',           # TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT
    'direction',        # 'horizontal' | 'vertical' | '' for connections
    'pass_id',          # unique pass number within the route
    'sub_index',        # 0 = main pass; >0 = extra fragment at same slice (holes)
    'is_forward',       # True = forward sweep, False = reversed
    'conn_id',          # connection id (blank for passes)
    'is_air_move',      # True = gun off travel (blank for passes)
    'sweep_direction',  # 'CW' or 'CCW'
    'spray_nx',         # spray approach direction (unit normal, written once per route)
    'spray_ny',
    'spray_nz',
    'pass_dx',          # pass travel direction unit vector (written on pt_idx=0 only)
    'pass_dy',
    'pass_dz',
    'length_mm',        # segment total length (written on pt_idx=0 only)
    'pt_idx',           # point index within the segment
    'x', 'y', 'z',     # world coordinates in mm
]


def export_route_csv(
    routes: list[PaintRoute],
    filepath: str,
    show_waypoints: bool = True,
) -> None:
    """Export routes to CSV.

    show_waypoints=True  → all TCP waypoints per pass (full resampled path).
    show_waypoints=False → only start and end point per pass (minimal robot program).
    """
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=_FIELDS, extrasaction='ignore')
        writer.writeheader()

        seq_id = 0

        for route_idx, route in enumerate(routes):
            first_pass = next((p for p in route.passes if p.sub_index == 0), None)
            sweep_dir = 'CW' if (first_pass is None or first_pass.is_forward) else 'CCW'

            sn = route.spray_normal
            sn_row = (round(float(sn[0]), 6), round(float(sn[1]), 6), round(float(sn[2]), 6))

            conn_by_from = {c.from_pass_id: c for c in route.connections}

            for p in route.passes:
                # --- pass rows ---
                seg_len = (
                    round(float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1))), 3)
                    if len(p.points) >= 2 else 0.0
                )
                # Travel direction unit vector at pass start
                if len(p.points) >= 2:
                    d = p.points[1] - p.points[0]
                    dn = np.linalg.norm(d)
                    d = d / dn if dn > 1e-9 else d
                    pass_dir = (round(float(d[0]), 6), round(float(d[1]), 6), round(float(d[2]), 6))
                else:
                    pass_dir = ('', '', '')

                pts_to_write = (
                    p.points if show_waypoints
                    else p.points[[0, -1]]
                )
                for i, pt in enumerate(pts_to_write):
                    pt_label = i if show_waypoints else ([0, len(p.points) - 1][i])
                    writer.writerow({
                        'seq_id':          seq_id,
                        'segment_type':    'pass',
                        'is_spray':        True,
                        'route_index':     route_idx,
                        'region':          route.region_id,
                        'direction':       p.direction,
                        'pass_id':         p.id,
                        'sub_index':       p.sub_index,
                        'is_forward':      p.is_forward,
                        'conn_id':         '',
                        'is_air_move':     '',
                        'sweep_direction': sweep_dir,
                        'spray_nx':        sn_row[0] if i == 0 else '',
                        'spray_ny':        sn_row[1] if i == 0 else '',
                        'spray_nz':        sn_row[2] if i == 0 else '',
                        'pass_dx':         pass_dir[0] if i == 0 else '',
                        'pass_dy':         pass_dir[1] if i == 0 else '',
                        'pass_dz':         pass_dir[2] if i == 0 else '',
                        'length_mm':       seg_len if i == 0 else '',
                        'pt_idx':          pt_label,
                        'x': round(float(pt[0]), 4),
                        'y': round(float(pt[1]), 4),
                        'z': round(float(pt[2]), 4),
                    })
                    seq_id += 1

                # --- connector rows immediately after this pass ---
                conn = conn_by_from.get(p.id)
                if conn is not None:
                    c_len = (
                        round(float(np.sum(np.linalg.norm(np.diff(conn.points, axis=0), axis=1))), 3)
                        if len(conn.points) >= 2 else 0.0
                    )
                    for i, pt in enumerate(conn.points):
                        writer.writerow({
                            'seq_id':          seq_id,
                            'segment_type':    'connection',
                            'is_spray':        False,
                            'route_index':     route_idx,
                            'region':          route.region_id,
                            'direction':       '',
                            'pass_id':         '',
                            'sub_index':       '',
                            'is_forward':      '',
                            'conn_id':         conn.id,
                            'is_air_move':     conn.is_air_move,
                            'sweep_direction': sweep_dir,
                            'spray_nx': '', 'spray_ny': '', 'spray_nz': '',
                            'pass_dx':  '', 'pass_dy':  '', 'pass_dz':  '',
                            'length_mm':       c_len if i == 0 else '',
                            'pt_idx':          i,
                            'x': round(float(pt[0]), 4),
                            'y': round(float(pt[1]), 4),
                            'z': round(float(pt[2]), 4),
                        })
                        seq_id += 1
