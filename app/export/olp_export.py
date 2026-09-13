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


def export_robodk(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
) -> None:
    """6-col CSV — drag-drop into RoboDK via Utilities > Import Curve.

    Passes only (Trigger ON).  No header row — RoboDK rejects files with one.
    NX/NY/NZ = outward surface normal; RoboDK uses it as the curve approach direction.
    """
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        f.write(_meta_header('RoboDK', params))
        writer = csv.writer(f)
        for route in routes:
            sn = route.spray_normal
            nx = round(float(sn[0]), 6)
            ny = round(float(sn[1]), 6)
            nz = round(float(sn[2]), 6)
            for p in route.passes:
                for pt in p.points:
                    writer.writerow([
                        round(float(pt[0]), 4),
                        round(float(pt[1]), 4),
                        round(float(pt[2]), 4),
                        nx, ny, nz,
                    ])


def export_vc(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
) -> None:
    """CSV for Visual Components import script.

    Header: seq_id,X,Y,Z,NX,NY,NZ,Trigger
    Passes = Trigger ON; connectors = Trigger OFF (no NX/NY/NZ on connectors).
    """
    _FIELDS = ['seq_id', 'X', 'Y', 'Z', 'NX', 'NY', 'NZ', 'Trigger']
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
                for pt in p.points:
                    writer.writerow({
                        'seq_id': seq,
                        'X': round(float(pt[0]), 4),
                        'Y': round(float(pt[1]), 4),
                        'Z': round(float(pt[2]), 4),
                        'NX': nx, 'NY': ny, 'NZ': nz,
                        'Trigger': 'ON',
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
                        })
                        seq += 1


def export_delmia_apt(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
) -> None:
    """APT file for DELMIA.

    GOTO/X,Y,Z,I,J,K  — spray passes (I,J,K = tool Z = -spray_normal)
    RAPID/X,Y,Z        — connector travel moves (no orientation)
    """
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(_meta_header('DELMIA APT', params))
        f.write('$$\n')  # APT comment separator
        f.write('PARTNO/SmartGrid_Toolpath\n')
        f.write('MACHIN/ROBOT\n')
        f.write('$$\n')

        for route in routes:
            sn = route.spray_normal
            # tool Z points INTO surface = -spray_normal
            ix = round(-float(sn[0]), 6)
            iy = round(-float(sn[1]), 6)
            iz = round(-float(sn[2]), 6)
            conn_by_from = {c.from_pass_id: c for c in route.connections}

            f.write(f'$$ Region: {route.region_id}\n')
            for p in route.passes:
                f.write(f'$$ Pass {p.id}  Trigger=ON\n')
                f.write('SPINDL/ON\n')
                for pt in p.points:
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
