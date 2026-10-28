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
import math
import os
import re
from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation

from app.path.path_model import PaintRoute, GenerationParams


def _part_identifier(params: GenerationParams | None, max_len: int = 16, fallback: str = 'PART') -> str:
    """Short identifier derived from the source mesh filename (e.g. SETOBJECT(B05) in
    Aleta_Ptras.tip, wobjMazdaMX5 in Exercise_2.mod) — real OEM references name the
    work object after the actual part/job, never a generic placeholder."""
    if not params or not params.source_file:
        return fallback
    name = os.path.splitext(os.path.basename(params.source_file))[0]
    name = re.sub(r'[^A-Za-z0-9_]', '_', name).strip('_').upper()
    return name[:max_len] or fallback


def _route_preset_numbers(routes: list[PaintRoute]) -> dict[int, int]:
    """1-based zone/preset number per route, in route order.

    S1_100.LS switches Preset[n] per zone and Aleta_Ptras.tip switches its
    SETBRUSH pattern id per zone — both real, OEM-confirmed signals that a
    single fixed value (Preset[1] / pattern 1) for the whole program doesn't
    carry. PaintRoute doesn't expose a real spray-pattern id yet, so the
    honestly-derivable substitute is route order itself: one region = one
    route = one real physical zone already, which is exactly the boundary a
    preset/pattern change happens at. Not a fabricated number — route index.
    """
    return {id(route): i + 1 for i, route in enumerate(routes)}


def _corner_cnt(points: np.ndarray, idx: int) -> int:
    """Continuous-path blend tolerance (Fanuc CNT) for waypoint idx.

    S1_100.LS tightens CNT (75/80/85) at its sharper direction changes and
    leaves CNT100 on straight runs — a real, geometry-derived choice (a wide
    blend radius overshoots a sharp corner). Computed from the actual turn
    angle between the incoming and outgoing segments, not fabricated.
    """
    if idx <= 0 or idx >= len(points) - 1:
        return 100
    v1 = points[idx] - points[idx - 1]
    v2 = points[idx + 1] - points[idx]
    n1, n2 = float(np.linalg.norm(v1)), float(np.linalg.norm(v2))
    if n1 < 1e-9 or n2 < 1e-9:
        return 100
    cos_a = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
    angle_deg = math.degrees(math.acos(cos_a))
    if angle_deg > 60:
        return 50
    if angle_deg > 30:
        return 75
    if angle_deg > 10:
        return 90
    return 100


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


def _get_normal(route: 'PaintRoute', p, pt_idx: int) -> tuple[float, float, float]:
    """Per-waypoint surface normal if available, else route spray_normal."""
    if p.normals is not None and pt_idx < len(p.normals):
        n = p.normals[pt_idx]
    else:
        n = route.spray_normal
    return round(float(n[0]), 6), round(float(n[1]), 6), round(float(n[2]), 6)


def _get_tangent(p, pt_idx: int) -> tuple[float, float, float] | None:
    """Per-waypoint TCP X-axis (corrected path tangent). None when not computed."""
    if p.tangent is not None and pt_idx < len(p.tangent):
        t = p.tangent[pt_idx]
        return round(float(t[0]), 6), round(float(t[1]), 6), round(float(t[2]), 6)
    return None


def _get_tcp_orientation(
    route: 'PaintRoute',
    p,
    pt_idx: int,
    points: np.ndarray | None = None,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Compute orthonormal TCP orientation frame at waypoint pt_idx.

    Returns:
        quaternion_wxyz: (q1, q2, q3, q4) where q1 is scalar w (ABB convention).
        fanuc_wpr: (W, P, R) in degrees (extrinsic XYZ).
        durr_abc: (A, B, C) in degrees.
    """
    if p is not None and getattr(p, 'normals', None) is not None and pt_idx < len(p.normals):
        n = np.asarray(p.normals[pt_idx], dtype=float)
    elif hasattr(route, 'spray_normal') and route.spray_normal is not None:
        n = np.asarray(route.spray_normal, dtype=float)
    else:
        n = np.array([0.0, 0.0, 1.0])

    n_len = np.linalg.norm(n)
    n = n / n_len if n_len > 1e-9 else np.array([0.0, 0.0, 1.0])

    # Tool approach Z points into the part surface
    tz = -n

    tx = None
    if p is not None and getattr(p, 'tangent', None) is not None and pt_idx < len(p.tangent):
        t_cand = np.asarray(p.tangent[pt_idx], dtype=float)
        if np.linalg.norm(t_cand) > 1e-6:
            tx = t_cand / np.linalg.norm(t_cand)

    if tx is None:
        pts = points if points is not None else (p.points if p is not None else None)
        if pts is not None and len(pts) > 1:
            if pt_idx < len(pts) - 1:
                diff = pts[pt_idx + 1] - pts[pt_idx]
            else:
                diff = pts[pt_idx] - pts[pt_idx - 1]
            diff_len = np.linalg.norm(diff)
            if diff_len > 1e-6:
                tx = diff / diff_len

    if tx is None:
        ref = np.array([0.0, 0.0, 1.0]) if abs(tz[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        tx = np.cross(tz, ref)
        tx = tx / (np.linalg.norm(tx) + 1e-9)

    # Gram-Schmidt: ensure tx is orthogonal to tz
    tx = tx - np.dot(tx, tz) * tz
    tx_len = np.linalg.norm(tx)
    if tx_len < 1e-6:
        ref = np.array([0.0, 1.0, 0.0]) if abs(tz[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
        tx = np.cross(tz, ref)
        tx_len = np.linalg.norm(tx)
    tx = tx / tx_len

    # ty completes right-handed triad
    ty = np.cross(tz, tx)
    ty = ty / (np.linalg.norm(ty) + 1e-9)

    R = np.column_stack([tx, ty, tz])
    rot = Rotation.from_matrix(R)
    q_xyzw = rot.as_quat()
    # ABB RAPID expects [w, x, y, z]
    q_wxyz = (round(float(q_xyzw[3]), 6), round(float(q_xyzw[0]), 6),
              round(float(q_xyzw[1]), 6), round(float(q_xyzw[2]), 6))
    eulers = rot.as_euler('xyz', degrees=True)
    wpr = (round(float(eulers[0]), 3), round(float(eulers[1]), 3), round(float(eulers[2]), 3))
    abc = (round(float(eulers[0]), 3), round(float(eulers[1]), 3), round(float(eulers[2]), 3))
    return q_wxyz, wpr, abc


def export_robodk(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """6-col or 7-col CSV — drag-drop into RoboDK via Utilities > Import Curve.

    No header row — RoboDK rejects non-numeric lines.
    6-col: X,Y,Z,NX,NY,NZ  (when no speeds_map)
    7-col: X,Y,Z,NX,NY,NZ,speed_mmpm  (when speeds_map provided)
    NX/NY/NZ = outward surface normal; RoboDK uses it as the curve approach direction.
    """
    fixed = params.paint_speed_mmpm if params and params.paint_speed_mmpm else 1000.0
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        for route in routes:
            for p in route.passes:
                speeds = _pass_speeds(p, speeds_map, fixed)
                for i, pt in enumerate(p.points):
                    nx, ny, nz = _get_normal(route, p, i)
                    row = [
                        round(float(pt[0]), 4),
                        round(float(pt[1]), 4),
                        round(float(pt[2]), 4),
                        nx, ny, nz,
                    ]
                    if speeds_map is not None:
                        row.append(round(float(speeds[i]), 1))
                    writer.writerow(row)


def export_vc(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """CSV for Visual Components import script.

    Header: seq_id,X,Y,Z,NX,NY,NZ,TX,TY,TZ,Trigger,speed_mmpm
    TX/TY/TZ = TCP X-axis (corrected path tangent) from Noether createTransform.
    Empty when normals/tangent not computed (e.g. BBox mode).
    Passes = Trigger ON; connectors = Trigger OFF.
    """
    _FIELDS = ['seq_id', 'X', 'Y', 'Z', 'NX', 'NY', 'NZ', 'TX', 'TY', 'TZ', 'Trigger', 'speed_mmpm']
    fixed = params.paint_speed_mmpm if params else 1000.0
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=_FIELDS)
        writer.writeheader()
        seq = 0
        for route in routes:
            conn_by_from = {c.from_pass_id: c for c in route.connections}
            for p in route.passes:
                speeds = _pass_speeds(p, speeds_map, fixed)
                for i, pt in enumerate(p.points):
                    nx, ny, nz = _get_normal(route, p, i)
                    tang = _get_tangent(p, i)
                    tx, ty, tz = tang if tang else ('', '', '')
                    writer.writerow({
                        'seq_id': seq,
                        'X': round(float(pt[0]), 4),
                        'Y': round(float(pt[1]), 4),
                        'Z': round(float(pt[2]), 4),
                        'NX': nx, 'NY': ny, 'NZ': nz,
                        'TX': tx, 'TY': ty, 'TZ': tz,
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
                            'TX': '', 'TY': '', 'TZ': '',
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
                    nx, ny, nz = _get_normal(route, p, i)
                    x = round(float(pt[0]), 4)
                    y = round(float(pt[1]), 4)
                    z = round(float(pt[2]), 4)
                    tang = _get_tangent(p, i)
                    if tang:
                        tx, ty, tz = tang
                        # TCP frame: Z=-normal (tool axis), X=tangent (path dir) — post-processor hint
                        f.write(f'GOTO/{x},{y},{z},{-nx},{-ny},{-nz} $$ TX={tx},{ty},{tz}\n')
                    else:
                        f.write(f'GOTO/{x},{y},{z},{-nx},{-ny},{-nz}\n')
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


def export_abb_rapid(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """ABB RAPID (.mod) program matching Exercise_2.mod OEM standard.

    Generates:
      - CONST robtarget declarations with unit quaternions [w, x, y, z]
      - PERS wobjdata named after the source part (e.g. wobjMazdaMX5 in the reference)
      - PROC runJob() calling one PROC per region (mvTOP/mvFRONT/... — Exercise_2.mod
        splits motion into mvHood/mvSideA/... one real body zone per PROC, not one
        flat list; route.region_id is the same real zone boundary)
      - One PaintL/MoveL motion PROC per route
    """
    fixed = params.paint_speed_mmpm if params else 1000.0
    wobj_name = 'wobj' + _part_identifier(params, max_len=20, fallback='Part')

    targets: list[tuple[str, tuple[float, float, float], tuple[float, float, float, float]]] = []
    # One motion list per route (region) — the real zone boundary a real program splits on.
    route_procs: list[tuple[str, list[tuple[str, str, int]]]] = []

    target_idx = 10
    used_proc_names: set[str] = set()
    for route in routes:
        proc_name = 'mv' + re.sub(r'[^A-Za-z0-9]', '', route.region_id).capitalize()
        if proc_name in used_proc_names:
            proc_name += str(len(used_proc_names))
        used_proc_names.add(proc_name)

        moves: list[tuple[str, str, int]] = []
        conn_by_from = {c.from_pass_id: c for c in route.connections}
        for p in route.passes:
            speeds = _pass_speeds(p, speeds_map, fixed)
            for i, pt in enumerate(p.points):
                t_name = f'p{target_idx}'
                target_idx += 10
                q_wxyz, _, _ = _get_tcp_orientation(route, p, i)
                targets.append((t_name, (round(float(pt[0]), 3), round(float(pt[1]), 3), round(float(pt[2]), 3)), q_wxyz))
                sp_mmps = int(round(float(speeds[i]) / 60.0)) if float(speeds[i]) > 0 else 500
                moves.append(('paint', t_name, sp_mmps))

            conn = conn_by_from.get(p.id)
            if conn is not None:
                for i, pt in enumerate(conn.points):
                    t_name = f'p{target_idx}'
                    target_idx += 10
                    q_wxyz, _, _ = _get_tcp_orientation(route, None, i, points=conn.points)
                    targets.append((t_name, (round(float(pt[0]), 3), round(float(pt[1]), 3), round(float(pt[2]), 3)), q_wxyz))
                    moves.append(('move', t_name, 1500))
        route_procs.append((proc_name, moves))

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write('%%%\n')
        f.write('  VERSION:1\n')
        f.write('  LANGUAGE:ENGLISH\n')
        f.write('%%%\n\n')
        f.write('MODULE M1\n')
        f.write('  ! Targets defined at standoff with unit quaternions [q1, q2, q3, q4] = [w, x, y, z]\n')
        for t_name, (x, y, z), (q1, q2, q3, q4) in targets:
            f.write(
                f'  CONST robtarget {t_name}:=[[{x:.3f},{y:.3f},{z:.3f}],'
                f'[{q1:.6f},{q2:.6f},{q3:.6f},{q4:.6f}],'
                f'[0,0,0,0],[9E+009,9E+009,9E+009,9E+009,9E+009,9E+009]];\n'
            )
        f.write('\n')
        f.write(f'  PERS wobjdata {wobj_name}:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];\n\n')
        f.write('  PROC runJob()\n')
        for proc_name, _moves in route_procs:
            f.write(f'    {proc_name};\n')
        f.write('  ENDPROC\n\n')
        for proc_name, moves in route_procs:
            f.write(f'  PROC {proc_name}()\n')
            for move_type, t_name, sp in moves:
                if move_type == 'paint':
                    f.write(f'    PaintL {t_name},v{sp},z10,tool0\\wobj:={wobj_name};\n')
                else:
                    f.write(f'    MoveL {t_name},v{sp},z50,tool0\\wobj:={wobj_name};\n')
            f.write('  ENDPROC\n\n')
        f.write('ENDMODULE\n')


def export_fanuc_ls(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """Fanuc TP / Karel ASCII format (.LS) matching S1_100.LS OEM standard.

    Generates:
      - /PROG, /ATTR, /APPL headers
      - /MN motion sequence with Gun=ON on pass start, Gun=OFF on pass end
      - /POS Cartesian positions (X,Y,Z in mm) and Euler angles (W,P,R in degrees)
    """
    fixed = params.paint_speed_mmpm if params else 1000.0
    preset_by_route = _route_preset_numbers(routes)

    points_data: list[tuple[int, tuple[float, float, float], tuple[float, float, float]]] = []
    mn_lines: list[str] = []

    pt_num = 1
    mn_line_num = 1

    cycle_time_sec = 0.0
    gun_on_time_sec = 0.0
    preset_gun_on_sec: dict[int, float] = {n: 0.0 for n in preset_by_route.values()}
    prev_xyz: np.ndarray | None = None

    for route in routes:
        preset_n = preset_by_route[id(route)]
        mn_lines.append(f'   {mn_line_num}:  Preset[{preset_n}] ;')
        mn_line_num += 1

        conn_by_from = {c.from_pass_id: c for c in route.connections}
        for p in route.passes:
            speeds = _pass_speeds(p, speeds_map, fixed)
            n_pts = len(p.points)
            for i, pt in enumerate(p.points):
                _, wpr, _ = _get_tcp_orientation(route, p, i)
                xyz = np.array([float(pt[0]), float(pt[1]), float(pt[2])])
                points_data.append((pt_num, (xyz[0], xyz[1], xyz[2]), wpr))
                sp_mmps = int(round(float(speeds[i]) / 60.0)) if float(speeds[i]) > 0 else 500

                if prev_xyz is not None and sp_mmps > 0:
                    seg_sec = float(np.linalg.norm(xyz - prev_xyz)) / sp_mmps
                    cycle_time_sec += seg_sec
                    gun_on_time_sec += seg_sec
                    preset_gun_on_sec[preset_n] += seg_sec
                prev_xyz = xyz

                if i == 0:
                    trigger_str = ' Gun=ON'
                elif i == n_pts - 1:
                    trigger_str = ' Gun=OFF'
                else:
                    trigger_str = ''

                cnt = _corner_cnt(p.points, i)
                mn_lines.append(f'   {mn_line_num}:L P[{pt_num}] {sp_mmps}mm/sec CNT{cnt}{trigger_str}    ;')
                mn_line_num += 1
                pt_num += 1

            conn = conn_by_from.get(p.id)
            if conn is not None:
                for i, pt in enumerate(conn.points):
                    _, wpr, _ = _get_tcp_orientation(route, None, i, points=conn.points)
                    xyz = np.array([float(pt[0]), float(pt[1]), float(pt[2])])
                    points_data.append((pt_num, (xyz[0], xyz[1], xyz[2]), wpr))
                    if prev_xyz is not None:
                        cycle_time_sec += float(np.linalg.norm(xyz - prev_xyz)) / 25.0   # 1500mm/min travel
                    prev_xyz = xyz
                    mn_lines.append(f'   {mn_line_num}:L P[{pt_num}] 1500mm/sec CNT100    ;')
                    mn_line_num += 1
                    pt_num += 1

    now = datetime.now()
    date_str = now.strftime('%y-%m-%d')
    time_str = now.strftime('%H:%M:%S')
    prog_name = os.path.splitext(os.path.basename(filepath))[0].upper().replace(' ', '_')[:16] or 'SMARTGRID'

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(f'/PROG  {prog_name}\t  Process\n')
        f.write('/ATTR\n')
        f.write('OWNER\t\t= MNEDITOR;\n')
        f.write('COMMENT\t\t= "SmartGrid OLP";\n')
        f.write('PROG_SIZE\t= 0;\n')
        f.write(f'CREATE\t\t= DATE {date_str}  TIME {time_str};\n')
        f.write(f'MODIFIED\t= DATE {date_str}  TIME {time_str};\n')
        f.write(f'FILE_NAME\t= {prog_name};\n')
        f.write('VERSION\t\t= 0;\n')
        f.write(f'LINE_COUNT\t= {len(mn_lines)};\n')
        f.write('MEMORY_SIZE\t= 0;\n')
        f.write('PROTECT\t\t= READ_WRITE;\n')
        f.write('TCD:  STACK_SIZE\t= 0,\n')
        f.write('      TASK_PRIORITY\t= 50,\n')
        f.write('      TIME_SLICE\t= 0,\n')
        f.write('      BUSY_LAMP_OFF\t= 0,\n')
        f.write('      ABORT_REQUEST\t= 0,\n')
        f.write('      PAUSE_REQUEST\t= 0;\n')
        f.write('DEFAULT_GROUP\t= 1,*,*,*,*;\n')
        f.write('CONTROL_CODE\t= 00000000 00000000;\n')
        f.write('/APPL\n\n')
        f.write('AUTO_SINGULARITY_HEADER;\n')
        f.write('  ENABLE_SINGULARITY_AVOIDANCE   : FALSE;\n')
        f.write('PAINT_PROCESS;\n')
        f.write(f'  LAST_CYCLE_TIME\t: {cycle_time_sec:.1f} sec;\n')
        f.write(f'  LAST_GUN_ON_TIME\t: {gun_on_time_sec:.1f} sec;\n')
        f.write('  DEFAULT_USER_FRAME\t: 1;\n')
        f.write('  DEFAULT_TOOL_FRAME\t: 1;\n')
        f.write('  START_DELAY\t\t: 0;\n')
        f.write(f'  LAST_GUN_OFF_LINE\t: {mn_line_num - 1};\n')
        f.write(f'  LAST_PROCESSED_DATE\t: DATE {date_str} TIME {time_str};\n\n')
        for n in range(1, 41):
            sec = preset_gun_on_sec.get(n, 0.0)
            f.write(f'  PRESET_#{n}_GUN_ON_TIME   : {sec / 60.0:.3f} min;\n')
        f.write('\nLINE_TRACK;\n')
        f.write('  LINE_TRACK_SCHEDULE_NUMBER      : 0;\n')
        f.write('  LINE_TRACK_BOUNDARY_NUMBER      : 0;\n')
        f.write('  CONTINUE_TRACK_AT_PROG_END      : FALSE;\n\n')
        f.write('/MN\n')
        for line in mn_lines:
            f.write(line + '\n')
        f.write('/POS\n')
        for p_idx, (x, y, z), (w, p, r) in points_data:
            f.write(f'P[{p_idx}]{{\n')
            f.write('   GP1:\n')
            f.write("\tUF : 1, UT : 1,\t\tCONFIG : 'N U T, 0, 0, 0',\n")
            f.write(f'\tX = {x:10.3f}  mm,\tY = {y:10.3f}  mm,\tZ = {z:10.3f}  mm,\n')
            f.write(f'\tW = {w:10.3f} deg,\tP = {p:10.3f} deg,\tR = {r:10.3f} deg\n')
            f.write('};\n')


def export_durr(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """Dürr EcoScreen trajectory (.tid & .tip) matching Aleta_Ptras standard.

    Writes companion pair:
      - .tid: Point table (P1 : POINT := (Object, X, Y, Z, A, B, C, 0, 0, 0)),
        PT{n} trigger-position points, and TRIGGERFRAME declarations.
      - .tip: Motion and brush commands (LIN(P1), SETBRUSH/GUN each with their
        own PT/TR pair, GunOff emitted only after the LIN that reaches it).
    """
    fixed = params.paint_speed_mmpm if params else 1000.0
    preset_by_route = _route_preset_numbers(routes)

    base_path, _ = os.path.splitext(filepath)
    tid_path = base_path + '.tid'
    tip_path = base_path + '.tip'

    points_data: list[tuple[int, tuple[float, float, float], tuple[float, float, float]]] = []
    # ponytail: PT trigger-position == the P point it fires at (no gun-delay offset
    # modeled); upgrade once spray-gun response time / lead distance is known.
    trigger_points: list[tuple[int, tuple[float, float, float], tuple[float, float, float]]] = []
    tip_lines: list[str] = [
        'RSELECT("R11")',
        '  //',
        '  SETTOOL(ECOBELL3E_200)',
        f'  SETOBJECT({_part_identifier(params)})',
        '  //',
        '  OVERLAP(NODEC50)',
        '  ACC(A3500)',
        '  VEL(V1000)',
        '  //',
    ]

    pt_num = 1
    tr_num = 1

    for route in routes:
        # Aleta_Ptras.tip marks each distinct body zone with a comment (//A_Pillar) —
        # route.region_id is the same real zone boundary SmartGrid already tracks.
        tip_lines.append(f'  //{route.region_id}')
        brush_pattern = preset_by_route[id(route)]
        conn_by_from = {c.from_pass_id: c for c in route.connections}
        for p in route.passes:
            speeds = _pass_speeds(p, speeds_map, fixed)
            n_pts = len(p.points)
            for i, pt in enumerate(p.points):
                _, _, abc = _get_tcp_orientation(route, p, i)
                coords = (float(pt[0]), float(pt[1]), float(pt[2]))
                points_data.append((pt_num, coords, abc))
                sp_mmps = int(round(float(speeds[i]) / 60.0)) if float(speeds[i]) > 0 else 500

                if i == 0:
                    tip_lines.append(f'  LIN(P{pt_num})')
                    tip_lines.append(f'  VEL(V{sp_mmps})')
                    trigger_points.append((tr_num, coords, abc))
                    # Pattern id derived from route order (region boundary) — see
                    # _route_preset_numbers; was hardcoded to 1 for every zone before.
                    tip_lines.append(f'  SETBRUSH(GUN1, {brush_pattern}, PT{tr_num}, TR{tr_num})')
                    tr_num += 1
                    trigger_points.append((tr_num, coords, abc))
                    tip_lines.append(f'  GUN(GUN1, GunOn, PT{tr_num}, TR{tr_num})')
                    tr_num += 1
                elif i == n_pts - 1:
                    tip_lines.append(f'  LIN(P{pt_num})')
                    trigger_points.append((tr_num, coords, abc))
                    tip_lines.append(f'  GUN(GUN1, GunOff, PT{tr_num}, TR{tr_num})')
                    tr_num += 1
                    tip_lines.append('  VEL(V1000)')
                else:
                    tip_lines.append(f'  LIN(P{pt_num})')

                pt_num += 1

            conn = conn_by_from.get(p.id)
            if conn is not None:
                for i, pt in enumerate(conn.points):
                    _, _, abc = _get_tcp_orientation(route, None, i, points=conn.points)
                    points_data.append((pt_num, (float(pt[0]), float(pt[1]), float(pt[2])), abc))
                    tip_lines.append(f'  LIN(P{pt_num})')
                    pt_num += 1

    with open(tid_path, 'w', encoding='utf-8') as f:
        f.write('// POINT\n')
        for p_idx, (x, y, z), (a, b, c) in points_data:
            f.write(f'P{p_idx} : POINT := (Object, {x:.3f}, {y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f}, 0, 0, 0)\n')
        for tr_idx, (x, y, z), (a, b, c) in trigger_points:
            f.write(f'PT{tr_idx} : POINT := (Object, {x:.3f}, {y:.3f}, {z:.3f}, {a:.3f}, {b:.3f}, {c:.3f}, 0, 0, 0)\n')
        f.write('\n// TRIGGERFRAME\n')
        for tr_idx in range(1, tr_num):
            f.write(f'TR{tr_idx} : TRIGGERFRAME := (Object, 0, 0, 0, 0)\n')
        f.write('//--------------------------------------------\n')

    with open(tip_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(tip_lines) + '\n')


def export_cad_dxf(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict[int, np.ndarray] | None = None,
) -> None:
    """Export 3D toolpath curves as AutoCAD DXF 3D polylines.

    Creates layers:
      - PAINT_PASSES (Blue) for spray passes at standoff depth.
      - AIR_CONNECTORS (Red) for air transfer moves.
    """
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write('0\nSECTION\n2\nHEADER\n0\nENDSEC\n')
        f.write('0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n2\n')
        f.write('0\nLAYER\n2\nPAINT_PASSES\n70\n0\n62\n5\n6\nCONTINUOUS\n')
        f.write('0\nLAYER\n2\nAIR_CONNECTORS\n70\n0\n62\n1\n6\nCONTINUOUS\n')
        f.write('0\nENDTAB\n0\nENDSEC\n')
        f.write('0\nSECTION\n2\nENTITIES\n')

        def _write_polyline(pts: np.ndarray, layer: str):
            if len(pts) < 2:
                return
            f.write('0\nPOLYLINE\n8\n' + layer + '\n66\n1\n70\n8\n')
            for pt in pts:
                f.write(
                    f'0\nVERTEX\n8\n{layer}\n'
                    f'10\n{float(pt[0]):.4f}\n'
                    f'20\n{float(pt[1]):.4f}\n'
                    f'30\n{float(pt[2]):.4f}\n'
                    '70\n32\n'
                )
            f.write('0\nSEQEND\n8\n' + layer + '\n')

        for route in routes:
            conn_by_from = {c.from_pass_id: c for c in route.connections}
            for p in route.passes:
                _write_polyline(p.points, 'PAINT_PASSES')
                conn = conn_by_from.get(p.id)
                if conn is not None:
                    _write_polyline(conn.points, 'AIR_CONNECTORS')

        f.write('0\nENDSEC\n0\nEOF\n')
