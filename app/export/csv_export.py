"""Export PaintRoute list to a neutral CSV for OLP software.

Row order: pass points → connector points → next pass points → …
Each row is one TCP waypoint.

Neutral column layout (essential columns first):
  seq_id       – global execution order (robot program line order)
  Trigger      – ON = spray gun firing, OFF = air travel
  segment_type – 'pass' | 'connection'
  pass_id      – pass or connection number
  pt_idx       – point index within the segment
  X, Y, Z      – TCP position in mm
  NX, NY, NZ   – outward surface normal (tool approach = -N; OLP computes W/P/R)
  length_mm    – segment total length in mm (written on pt_idx=0 only)
  region       – face region (TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT)
  is_forward   – True = forward sweep, False = reversed (blank for connections)

OLP mapping guide (comment block in every export):
  RoboDK  → drag-drop: use X Y Z NX NY NZ (6 cols) as a curve import
  VC      → import script: seq_id + X Y Z + NX NY NZ + Trigger
  DELMIA  → APT macro: seq_id + X Y Z + NX NY NZ + Trigger
  Custom  → all columns available; length_mm useful for speed ramping

Waypoints exported:
  Custom interval OFF → raw slicer path points (mesh-density vertices)
  Custom interval ON  → resampled points at the set interval
  Endpoint dots (visual only) are not extra rows — they are the
  existing first/last points of every pass already in the data.
"""
from __future__ import annotations
import csv

import numpy as np
from app.path.path_model import PaintRoute, GenerationParams

_FIELDS = [
    'seq_id',        # global execution order (0, 1, 2, …)
    'Trigger',       # ON = spray gun firing, OFF = air travel
    'segment_type',  # 'pass' | 'connection'
    'pass_id',       # pass or connection id
    'pt_idx',        # point index within the segment
    'X', 'Y', 'Z',  # TCP position in mm (4 decimal places)
    'NX', 'NY', 'NZ',  # outward surface normal unit vector (6 dp)
    'length_mm',     # segment total length — written on pt_idx=0 only
    'region',        # face region label
    'is_forward',    # True/False for passes; blank for connections
]


def _write_metadata(f, params: GenerationParams) -> None:
    lines = [
        '# -- SmartGrid Toolpath Export ------------------------------------------',
        f'# software          : {params.software}',
        f'# generated_at      : {params.generated_at}',
        f'# source_file       : {params.source_file}',
        f'# path_mode         : {params.path_mode}',
        f'# regions           : {", ".join(params.regions)}',
        f'# up_axis           : {params.up_axis}',
        f'# spray_width_mm    : {params.spray_width_mm}',
        f'# standoff_mm       : {params.standoff_mm if params.standoff_mm else "off"}',
        (
            f'# waypoint_interval : {params.waypoint_spacing_mm} mm  (custom resampling active)'
            if params.waypoint_spacing_mm
            else '# waypoint_interval : off  (raw mesh-slicer points)'
        ),
        '#',
        '# OLP tool frame convention:',
        '#   NX/NY/NZ = outward surface normal  (gun approach direction = -NX/-NY/-NZ)',
        '#   Tool Z   = -NX/-NY/-NZ             (points INTO surface)',
        '#   Tool X   = pass travel direction   (derive from consecutive XYZ rows)',
        '#   Tool Y   = cross(Tool_Z, Tool_X)   (right-hand rule)',
        '#',
        '# OLP mapping:',
        '#   RoboDK drag-drop : X Y Z NX NY NZ  (6-col curve import)',
        '#   VC / DELMIA      : seq_id + X Y Z NX NY NZ + Trigger',
        '# -----------------------------------------------------------------------',
        '',
    ]
    for line in lines:
        f.write(line + '\n')


def export_route_csv(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
) -> None:
    """Export routes to neutral CSV.

    Always writes all path points (raw slicer pts or custom-resampled pts —
    whichever is stored in p.points at export time). No start/end-only mode.
    """
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        if params is not None:
            _write_metadata(f, params)

        writer = csv.DictWriter(f, fieldnames=_FIELDS, extrasaction='ignore')
        writer.writeheader()

        seq_id = 0

        for route in routes:
            sn = route.spray_normal
            nx = round(float(sn[0]), 6)
            ny = round(float(sn[1]), 6)
            nz = round(float(sn[2]), 6)

            conn_by_from = {c.from_pass_id: c for c in route.connections}

            for p in route.passes:
                seg_len = (
                    round(float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1))), 3)
                    if len(p.points) >= 2 else 0.0
                )
                for i, pt in enumerate(p.points):
                    writer.writerow({
                        'seq_id':       seq_id,
                        'Trigger':      'ON',
                        'segment_type': 'pass',
                        'pass_id':      p.id,
                        'pt_idx':       i,
                        'X': round(float(pt[0]), 4),
                        'Y': round(float(pt[1]), 4),
                        'Z': round(float(pt[2]), 4),
                        'NX': nx,
                        'NY': ny,
                        'NZ': nz,
                        'length_mm':  seg_len if i == 0 else '',
                        'region':     route.region_id,
                        'is_forward': p.is_forward,
                    })
                    seq_id += 1

                conn = conn_by_from.get(p.id)
                if conn is not None:
                    c_len = (
                        round(float(np.sum(np.linalg.norm(np.diff(conn.points, axis=0), axis=1))), 3)
                        if len(conn.points) >= 2 else 0.0
                    )
                    for i, pt in enumerate(conn.points):
                        writer.writerow({
                            'seq_id':       seq_id,
                            'Trigger':      'OFF',
                            'segment_type': 'connection',
                            'pass_id':      conn.id,
                            'pt_idx':       i,
                            'X': round(float(pt[0]), 4),
                            'Y': round(float(pt[1]), 4),
                            'Z': round(float(pt[2]), 4),
                            'NX': '', 'NY': '', 'NZ': '',
                            'length_mm':  c_len if i == 0 else '',
                            'region':     route.region_id,
                            'is_forward': '',
                        })
                        seq_id += 1
