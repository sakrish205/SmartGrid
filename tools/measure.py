"""CLI measurement tool for SmartGrid Mesh Surface paths.

Usage:
    python tools/measure.py <mesh_file> --pitch <mm> --standoff <mm>
                            [--standoff-tol <mm>] [--up-axis {0,1,2}]
                            [--direction {horizontal,vertical}]

Loads the mesh, generates a route with the existing Mesh Surface generator
(current curve-tracing behaviour — unchanged), measures it with measure_route,
prints a summary table, and saves the full result as JSON next to the mesh file.

Example:
    python tools/measure.py models/bumper.stl --pitch 100 --standoff 250
"""
from __future__ import annotations
import argparse
import json
import os
import sys
import time

# make app imports work regardless of cwd
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from app.mesh.loader      import load_mesh
from app.mesh.preprocessor import preprocess
from app.mesh.regions      import classify_regions
from app.path.generator    import generate_route
from app.path.metrics      import measure_route


def _fmt(v, fmt='.1f') -> str:
    return f'{v:{fmt}}' if v is not None else 'n/a'


def main() -> None:
    ap = argparse.ArgumentParser(description='Measure SmartGrid Mesh Surface path quality.')
    ap.add_argument('mesh_file',   help='Path to STL, OBJ or STEP mesh')
    ap.add_argument('--mode',      default='mesh_surface',
                    help='Path mode (currently only mesh_surface supported)')
    ap.add_argument('--pitch',     type=float, required=True,  help='Spray pitch (mm)')
    ap.add_argument('--standoff',  type=float, required=True,  help='Standoff (mm)')
    ap.add_argument('--standoff-tol', dest='standoff_tol', type=float, default=5.0,
                    help='Standoff tolerance (mm, default 5)')
    ap.add_argument('--up-axis',   dest='up_axis', type=int,   default=2,
                    choices=[0, 1, 2], help='Up axis: 0=X 1=Y 2=Z (default 2)')
    ap.add_argument('--direction', default='horizontal',
                    choices=['horizontal', 'vertical'])
    args = ap.parse_args()

    if not os.path.isfile(args.mesh_file):
        print(f'Error: file not found: {args.mesh_file}', file=sys.stderr)
        sys.exit(1)

    print(f'Loading {args.mesh_file}…')
    mesh = load_mesh(args.mesh_file)
    data = preprocess(mesh, args.mesh_file, up_axis=args.up_axis)

    print(f'  {len(mesh.faces):,} faces  bbox extents: '
          f'{data.bbox_extents[0]:.0f} × {data.bbox_extents[1]:.0f} × '
          f'{data.bbox_extents[2]:.0f} mm')

    region_faces = classify_regions(data)
    non_empty    = {k: v for k, v in region_faces.items() if len(v) > 0}
    print(f'  regions: {", ".join(f"{k}({len(v):,})" for k, v in non_empty.items())}')

    all_passes: list[np.ndarray] = []
    gen_start = time.perf_counter()

    for region_id, face_ids in sorted(non_empty.items()):
        print(f'  generating {region_id}…')
        route = generate_route(
            data, region_id, face_ids,
            spray_width_mm=args.pitch,
            waypoint_spacing_mm=args.pitch / 5.0,
            direction=args.direction,
        )
        for p in route.passes:
            if len(p.points) >= 2:
                all_passes.append(p.points)

    gen_time = time.perf_counter() - gen_start
    print(f'  {len(all_passes)} passes generated in {gen_time:.2f} s')

    if not all_passes:
        print('No passes generated — check mesh, up-axis and pitch settings.')
        sys.exit(1)

    print('Measuring…')
    m = measure_route(
        mesh, all_passes,
        pitch_mm=args.pitch,
        standoff_mm=args.standoff,
        standoff_tol_mm=args.standoff_tol,
        gen_time_s=gen_time,
    )

    # ---- summary table ----
    g   = m['global']
    thi = m['thickness']

    print()
    print('=' * 60)
    print('  MESH SURFACE MEASUREMENT SUMMARY')
    print('=' * 60)
    print(f'  Pitch              : {args.pitch} mm')
    print(f'  Standoff target    : {args.standoff} ± {args.standoff_tol} mm')
    print(f'  Direction          : {args.direction}')
    print(f'  Generation time    : {gen_time:.2f} s')
    print(f'  Total passes       : {g["total_passes"]}')
    print(f'  Total waypoints    : {g["total_waypoints"]}')
    print()
    print('  SPACING (surface, vs pitch)')
    print(f'    % OK             : {_fmt(g["pct_ok"])} %')
    print(f'    % overlap        : {_fmt(g["pct_overlap"])} %')
    print(f'    % gap            : {_fmt(g["pct_gap"])} %')
    print()
    print('  STANDOFF')
    print(f'    out-of-tolerance : {g["count_out_of_tol"]} samples')
    print()
    print('  CROSSINGS')
    print(f'    count            : {len(m["crossings"])}')
    if m['crossings']:
        for c in m['crossings'][:5]:
            print(f'    passes ({c["pass_a"]},{c["pass_b"]})  '
                  f'dist={c["dist_mm"]:.2f} mm')
        if len(m['crossings']) > 5:
            print(f'    … and {len(m["crossings"]) - 5} more')
    print()
    print('  THICKNESS (Gaussian, sigma=pitch/2)')
    print(f'    mean             : {_fmt(thi["mean"], ".3f")}')
    print(f'    min              : {_fmt(thi["min"],  ".3f")}')
    print(f'    max              : {_fmt(thi["max"],  ".3f")}')
    print(f'    CV               : {_fmt(thi["cv"],   ".3f")}')
    print(f'    % uncovered      : {_fmt(thi["pct_uncovered"])} %')
    print()
    print('  COLLISIONS')
    print(f'    waypoints inside : {g["total_waypoints_inside"]}')
    print()

    # per-pass table
    print(f'  {"Pass":>4}  {"Wpts":>5}  '
          f'{"Spc_min":>7}  {"Spc_mean":>8}  {"Spc_max":>7}  '
          f'{"Spd_OOT":>7}  {"Inside":>6}')
    print('  ' + '-' * 56)
    for i, r in enumerate(m['passes']):
        sp = r['spacing']
        sd = r['standoff']
        co = r['collision']
        print(f'  {i:>4}  {r["waypoint_count"]:>5}  '
              f'{_fmt(sp["min_mm"]):>7}  {_fmt(sp["mean_mm"]):>8}  '
              f'{_fmt(sp["max_mm"]):>7}  '
              f'{sd["count_out_of_tol"]:>7}  {co["waypoints_inside"]:>6}')
    print('=' * 60)

    # ---- save JSON ----
    base   = os.path.splitext(args.mesh_file)[0]
    out_path = base + '_metrics.json'

    # numpy types are not JSON-serialisable — convert to native Python
    def _default(obj):
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        raise TypeError(f'Not serialisable: {type(obj)}')

    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(m, f, indent=2, default=_default)
    print(f'Full metrics saved → {out_path}')


if __name__ == '__main__':
    main()
