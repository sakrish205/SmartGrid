"""OLP-specific exporters derived from the neutral CSV/JSON data.

Three thin converters — each reads p.points and route.spray_normal and writes
the exact format the target OLP expects.  No logic here; all path data is
already correct in the route objects.

RoboDK  — 6-col CSV, no header: X,Y,Z,NX,NY,NZ  (Utilities > Import Curve)
VC      — CSV with header: seq_id,X,Y,Z,NX,NY,NZ,Trigger
DELMIA  — APT text: GOTO/X,Y,Z,I,J,K  (I,J,K = tool-Z = -spray_normal)
           connector moves: RAPID/X,Y,Z  (travel, no orientation)
"""
from __future__ import annotations
import csv
import numpy as np
from datetime import datetime

from app.path.path_model import PaintRoute, GenerationParams


def _meta_header(fmt: str, params: GenerationParams | None) -> str:
    lines = [
        f'# SmartGrid OLP Export — {fmt}',
        f'# generated_at : {datetime.now().isoformat(timespec="seconds")}',
    ]
    if params:
        lines += [
            f'# source_file  : {params.source_file}',
            f'# path_mode    : {params.path_mode}',
            f'# regions      : {", ".join(params.regions)}',
        ]
    return '\n'.join(lines) + '\n'


def _pass_speeds(p, speeds_map: dict[int, np.ndarray] | None, fixed: float) -> np.ndarray:
    """Return per-waypoint speeds for pass p — array of len(p.points)."""
    if speeds_map is not None and p.id in speeds_map:
        return speeds_map[p.id]
    return np.full(len(p.points), fixed)


def export_robodk(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """6-col (auto) or 7-col (speed) CSV — drag-drop into RoboDK via Utilities > Import Curve.

    Passes only (Trigger ON).  No header row — RoboDK rejects files with one.
    NX/NY/NZ = outward surface normal; RoboDK uses it as the curve approach direction.
    """
    fixed = params.paint_speed_mmpm if params else 1000.0
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        # RoboDK curve-import rejects any non-numeric rows — no header written.
        writer = csv.writer(f)
        for route in routes:
            sn = route.spray_normal
            nx = round(float(sn[0]), 6)
            ny = round(float(sn[1]), 6)
            nz = round(float(sn[2]), 6)
            for p in route.passes:
                speeds = _pass_speeds(p, speeds_map, fixed)
                for i, pt in enumerate(p.points):
                    writer.writerow([
                        round(float(pt[0]), 4),
                        round(float(pt[1]), 4),
                        round(float(pt[2]), 4),
                        nx, ny, nz,
                        round(float(speeds[i]), 1),
                    ])


def export_vc(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """CSV for Visual Components import script.

    Header: seq_id,X,Y,Z,NX,NY,NZ,Trigger,speed_mmpm
    Passes = Trigger ON; connectors = Trigger OFF (no NX/NY/NZ on connectors).
    """
    _FIELDS = ['seq_id', 'X', 'Y', 'Z', 'NX', 'NY', 'NZ', 'Trigger', 'speed_mmpm']
    fixed = params.paint_speed_mmpm if params else 1000.0
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        f.write(_meta_header('VisualComponents', params))
        writer = csv.DictWriter(f, fieldnames=_FIELDS)
        writer.writeheader()
        seq = 0
        for route in routes:
            sn = route.spray_normal
            nx = round(float(sn[0]), 6)
            ny = round(float(sn[1]), 6)
            nz = round(float(sn[2]), 6)
            conn_by_from = {c.from_pass_id: c for c in route.connections}
            for p in route.passes:
                speeds = _pass_speeds(p, speeds_map, fixed)
                for i, pt in enumerate(p.points):
                    writer.writerow({
                        'seq_id': seq,
                        'X': round(float(pt[0]), 4),
                        'Y': round(float(pt[1]), 4),
                        'Z': round(float(pt[2]), 4),
                        'NX': nx, 'NY': ny, 'NZ': nz,
                        'Trigger': 'ON',
                        'speed_mmpm': round(float(speeds[i]), 1),
                    })
                    seq += 1
                conn = conn_by_from.get(p.id)
                if conn is not None:
                    for pt in conn.points:
                        writer.writerow({
                            'seq_id': seq,
                            'X': round(float(pt[0]), 4),
                            'Y': round(float(pt[1]), 4),
                            'Z': round(float(pt[2]), 4),
                            'NX': '', 'NY': '', 'NZ': '',
                            'Trigger': 'OFF',
                            'speed_mmpm': '',
                        })
                        seq += 1


def export_delmia_apt(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """APT file for DELMIA.

    GOTO/X,Y,Z,I,J,K  — spray passes (I,J,K = tool Z = -spray_normal)
    RAPID/X,Y,Z        — connector travel moves (no orientation)
    FEDRAT written only on speed change (dynamic) or once per pass (fixed).
    """
    fixed = params.paint_speed_mmpm if params else 1000.0
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(_meta_header('DELMIA APT', params))
        f.write('$$\n')
        f.write('PARTNO/SmartGrid_Toolpath\n')
        f.write('MACHIN/ROBOT\n')
        f.write('$$\n')

        for route in routes:
            sn = route.spray_normal
            ix = round(-float(sn[0]), 6)
            iy = round(-float(sn[1]), 6)
            iz = round(-float(sn[2]), 6)
            conn_by_from = {c.from_pass_id: c for c in route.connections}

            f.write(f'$$ Region: {route.region_id}\n')
            for p in route.passes:
                f.write(f'$$ Pass {p.id}  Trigger=ON\n')
                speeds = _pass_speeds(p, speeds_map, fixed)
                f.write('SPINDL/ON\n')
                prev_sp: float | None = None
                for i, pt in enumerate(p.points):
                    sp = round(float(speeds[i]), 1)
                    if sp != prev_sp:
                        f.write(f'FEDRAT/{sp:.1f},MMPM\n')
                        prev_sp = sp
                    x = round(float(pt[0]), 4)
                    y = round(float(pt[1]), 4)
                    z = round(float(pt[2]), 4)
                    f.write(f'GOTO/{x},{y},{z},{ix},{iy},{iz}\n')
                f.write('SPINDL/OFF\n')

                conn = conn_by_from.get(p.id)
                if conn is not None:
                    f.write(f'$$ Connection {conn.id}  Trigger=OFF\n')
                    for pt in conn.points:
                        x = round(float(pt[0]), 4)
                        y = round(float(pt[1]), 4)
                        z = round(float(pt[2]), 4)
                        f.write(f'RAPID/{x},{y},{z}\n')

        f.write('FINI\n')


def export_gcode(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """G-code for CNC/open robot controllers.

    G21 G90 G94  — mm, absolute, feed/min
    M8/M9        — spray gun on/off
    G1 F{sp} X Y Z on speed change; G1 X Y Z when speed unchanged (keeps file small).
    G0           — rapid connectors
    # ponytail: tool orientation (A,B,C) skipped — controller-specific, add post-processor when needed
    """
    fixed = params.paint_speed_mmpm if params else 1000.0

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write('%\n')
        f.write('O0001 (SMARTGRID TOOLPATH)\n')
        f.write('G21 G90 G94\n')
        if params:
            f.write(f'( source    : {params.source_file} )\n')
            f.write(f'( path_mode : {params.path_mode} )\n')
            f.write(f'( regions   : {", ".join(params.regions)} )\n')
            f.write(f'( generated : {params.generated_at} )\n')

        for route in routes:
            conn_by_from = {c.from_pass_id: c for c in route.connections}
            f.write(f'( Region: {route.region_id} )\n')
            for p in route.passes:
                f.write(f'( Pass {p.id} )\n')
                f.write('M8\n')
                speeds = _pass_speeds(p, speeds_map, fixed)
                prev_sp: float | None = None
                for i, pt in enumerate(p.points):
                    sp = round(float(speeds[i]), 1)
                    x = round(float(pt[0]), 4)
                    y = round(float(pt[1]), 4)
                    z = round(float(pt[2]), 4)
                    if sp != prev_sp:
                        f.write(f'G1 F{sp} X{x} Y{y} Z{z}\n')
                        prev_sp = sp
                    else:
                        f.write(f'G1 X{x} Y{y} Z{z}\n')
                f.write('M9\n')
                conn = conn_by_from.get(p.id)
                if conn is not None:
                    for pt in conn.points:
                        x = round(float(pt[0]), 4)
                        y = round(float(pt[1]), 4)
                        z = round(float(pt[2]), 4)
                        f.write(f'G0 X{x} Y{y} Z{z}\n')

        f.write('M30\n')
        f.write('%\n')
