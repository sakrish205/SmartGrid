"""Durr EcoScreen (.tid + .tip) reference-program parser.

Confirmed real-file behavior this parser is built against: `.tip`'s own LIN(...)
call order IS the execution order (unlike ABB/FANUC, no reordering needed here).
SETBRUSH/GUN(GunOn) are separate modal directives with their own PT/TR indices,
not a single combined event. VEL/ACC are modal state changes with no point
argument. WAIT_PAINTPOSITION(n) is a non-motion timed wait. Point declarations
use either `(Object, ...)` (Cartesian) or `(Axis, ...)` (joint-space, e.g. one
point in xpos_test.tid) as the first field — frame_type is read from this, not
assumed. Many declared P/PT points are legitimately never referenced by any
LIN/SETBRUSH/GUN call (confirmed: ~97 of 175 in Aleta_Ptras, all 23 PT + 26 TR
in xpos_test) — these are kept in unreferenced_source_ids, never dropped.

ponytail: regex-based line scanner over a fixed set of known directives
(LIN/VEL/ACC/SETBRUSH/GUN/WAIT_PAINTPOSITION/SETTOOL/SETOBJECT/OVERLAP/RSELECT/
RDESELECT). An unrecognized directive is recorded as a parse_warning rather
than silently ignored or guessed at.
"""
from __future__ import annotations
import re

import numpy as np

from app.reference.reference_model import (
    ProcessState, ReferenceFrameType, ReferenceProgram, ReferenceWaypoint,
)

_POINT_RE = re.compile(
    r'^(?P<name>\w+)\s*:\s*POINT\s*:=\s*\(\s*(?P<frame>Object|Axis)\s*,\s*(?P<vals>[^)]*)\)'
)
_LIN_RE = re.compile(r'^\s*LIN\((?P<name>\w+)\)')
_VEL_RE = re.compile(r'^\s*VEL\((?P<name>\w+)\)')
_ACC_RE = re.compile(r'^\s*ACC\((?P<name>\w+)\)')
_SETBRUSH_RE = re.compile(r'^\s*SETBRUSH\(\s*\w+\s*,\s*(?P<pattern>\d+)\s*,\s*(?P<pt>\w+)\s*,\s*(?P<tr>\w+)\s*\)')
_GUN_RE = re.compile(r'^\s*GUN\(\s*\w+\s*,\s*Gun(?P<state>On|Off)\s*,\s*(?P<pt>\w+)\s*,\s*(?P<tr>\w+)\s*\)')
_WAIT_RE = re.compile(r'^\s*WAIT_PAINTPOSITION\((?P<ms>\d+)\)')
_SKIP_RE = re.compile(r'^\s*(SETTOOL|SETOBJECT|OVERLAP|RSELECT|RDESELECT)\(')
_COMMENT_RE = re.compile(r'^\s*//')


def _parse_points(tid_text: str) -> dict[str, tuple[np.ndarray, np.ndarray, 'ReferenceFrameType']]:
    points: dict[str, tuple[np.ndarray, np.ndarray, ReferenceFrameType]] = {}
    for line in tid_text.splitlines():
        m = _POINT_RE.match(line)
        if not m:
            continue
        vals = [float(v) for v in m.group('vals').split(',')]
        frame = ReferenceFrameType.OBJECT if m.group('frame') == 'Object' else ReferenceFrameType.AXIS
        points[m.group('name')] = (np.array(vals[0:3]), np.array(vals[3:6]), frame)
    return points


def parse(tid_path: str, tip_path: str) -> ReferenceProgram:
    with open(tid_path, encoding='utf-8') as f:
        points = _parse_points(f.read())
    with open(tip_path, encoding='utf-8') as f:
        tip_lines = f.read().splitlines()

    warnings: list[str] = []
    waypoints: list[ReferenceWaypoint] = []
    referenced: set[str] = set()
    state = ProcessState(gun_on=None)
    exec_index = 0

    def _lookup(name: str) -> tuple[np.ndarray | None, np.ndarray | None, ReferenceFrameType]:
        entry = points.get(name)
        if entry is None:
            warnings.append(f"reference to undeclared point '{name}'")
            return None, None, ReferenceFrameType.OBJECT
        pos, ori, frame = entry
        return pos, ori, frame

    for line_no, line in enumerate(tip_lines):
        if _COMMENT_RE.match(line) or _SKIP_RE.match(line) or not line.strip():
            continue

        m = _LIN_RE.match(line)
        if m:
            referenced.add(m.group('name'))
            pos, ori, frame = _lookup(m.group('name'))
            waypoints.append(ReferenceWaypoint(
                source_id=m.group('name'), exec_index=exec_index,
                position=pos, orientation=ori, frame_type=frame,
                move_type='LIN', process_state=state.copy(), speed_ref=None,
                is_motion=True, raw_line_refs=[line_no],
            ))
            exec_index += 1
            continue

        m = _VEL_RE.match(line) or _ACC_RE.match(line)
        if m:
            label = 'VEL' if line.strip().startswith('VEL') else 'ACC'
            state = state.copy()
            state.modal_notes.append(f'{label}({m.group("name")})')
            waypoints.append(ReferenceWaypoint(
                source_id=f'{label}({m.group("name")})', exec_index=exec_index,
                position=None, orientation=None, frame_type=ReferenceFrameType.OBJECT,
                move_type=label, process_state=state.copy(), speed_ref=m.group('name'),
                is_motion=False, raw_line_refs=[line_no],
            ))
            exec_index += 1
            continue

        m = _SETBRUSH_RE.match(line)
        if m:
            referenced.add(m.group('pt'))
            referenced.add(m.group('tr'))
            state = state.copy()
            state.setbrush_index = int(m.group('pattern'))
            waypoints.append(ReferenceWaypoint(
                source_id=f'SETBRUSH[{m.group("pt")},{m.group("tr")}]', exec_index=exec_index,
                position=None, orientation=None, frame_type=ReferenceFrameType.OBJECT,
                move_type='SETBRUSH', process_state=state.copy(), speed_ref=None,
                is_motion=False, raw_line_refs=[line_no],
            ))
            exec_index += 1
            continue

        m = _GUN_RE.match(line)
        if m:
            referenced.add(m.group('pt'))
            referenced.add(m.group('tr'))
            state = state.copy()
            state.gun_on = (m.group('state') == 'On')
            waypoints.append(ReferenceWaypoint(
                source_id=f'GUN[{m.group("pt")},{m.group("tr")}]', exec_index=exec_index,
                position=None, orientation=None, frame_type=ReferenceFrameType.OBJECT,
                move_type='GUN', process_state=state.copy(), speed_ref=None,
                is_motion=False, raw_line_refs=[line_no],
            ))
            exec_index += 1
            continue

        m = _WAIT_RE.match(line)
        if m:
            waypoints.append(ReferenceWaypoint(
                source_id=f'WAIT_PAINTPOSITION({m.group("ms")})', exec_index=exec_index,
                position=None, orientation=None, frame_type=ReferenceFrameType.OBJECT,
                move_type='WAIT', process_state=state.copy(), speed_ref=None,
                is_motion=False, raw_line_refs=[line_no],
            ))
            exec_index += 1
            continue

        warnings.append(f"line {line_no}: unrecognized .tip directive skipped: {line.strip()!r}")

    unreferenced = sorted(set(points) - referenced)

    return ReferenceProgram(
        source_file=f'{tid_path} + {tip_path}',
        controller='DURR',
        waypoints=waypoints,
        unreferenced_source_ids=unreferenced,
        dead_procedures=[],
        parse_warnings=warnings,
    )
