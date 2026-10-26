"""Fanuc TP/Karel (.LS) reference-program parser.

Confirmed real-file behavior this parser is built against (S1_100.LS): `/MN`
already lists moves in actual execution order (proven: it calls P[32],P[33],
P[42],P[34],P[35]... while `/POS` declares P[1]..P[63] strictly ascending —
so /MN's own line order IS the exec order, unlike ABB/DURR where this had to
be reconstructed). `Preset[n]` lines are modal process-state changes with no
point argument (non-monotonic — Preset[3] recurs); bare `DO[n]=ON/OFF` lines
carry no point reference either; `Gun=ON`/`Gun=OFF` are inline suffixes on
motion lines, modal (toggled once each in the sample, held until changed).

ponytail: regex-based line scanner over /MN and /POS only — does not interpret
CONFIG turn-flags, UF/UT frame indices beyond capturing them opaquely, or
non-L/J move types (circular/C moves). Upgrade path: extend _MOTION_RE if a
future sample uses them.
"""
from __future__ import annotations
import re

import numpy as np

from app.reference.reference_model import (
    ProcessState, ReferenceFrameType, ReferenceProgram, ReferenceWaypoint,
)

_POS_BLOCK_RE = re.compile(
    r'P\[(?P<id>\d+)\]\{\s*GP1:\s*'
    r"UF\s*:\s*(?P<uf>\d+)\s*,\s*UT\s*:\s*(?P<ut>\d+)\s*,\s*CONFIG\s*:\s*'(?P<config>[^']*)'\s*,\s*"
    r'X\s*=\s*(?P<x>[-\d.]+)\s*mm\s*,\s*Y\s*=\s*(?P<y>[-\d.]+)\s*mm\s*,\s*Z\s*=\s*(?P<z>[-\d.]+)\s*mm\s*,\s*'
    r'W\s*=\s*(?P<w>[-\d.]+)\s*deg\s*,\s*P\s*=\s*(?P<p>[-\d.]+)\s*deg\s*,\s*R\s*=\s*(?P<r>[-\d.]+)\s*deg',
    re.DOTALL,
)
_MOTION_RE = re.compile(
    r'^\s*\d+:\s*(?P<move>[JLC])\s*P\[(?P<id>\d+)\]\s*(?P<speed>[\d.]+%|\d+mm/sec)\s*CNT\d+'
    r'(?P<gun>\s*Gun=(?:ON|OFF))?\s*;'
)
_PRESET_RE = re.compile(r'^\s*\d+:\s*Preset\[(?P<n>\d+)\]\s*;')
_DO_RE = re.compile(r'^\s*\d+:\s*DO\[(?P<n>\d+)\]\s*=\s*(?P<val>ON|OFF)\s*;')


def _parse_positions(text: str) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    positions: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for m in _POS_BLOCK_RE.finditer(text):
        pos = np.array([float(m.group('x')), float(m.group('y')), float(m.group('z'))])
        wpr = np.array([float(m.group('w')), float(m.group('p')), float(m.group('r'))])
        positions[int(m.group('id'))] = (pos, wpr)
    return positions


def parse(path: str) -> ReferenceProgram:
    with open(path, encoding='utf-8') as f:
        text = f.read()
    lines = text.splitlines()

    mn_start = text.find('/MN')
    pos_start = text.find('/POS')
    if mn_start == -1 or pos_start == -1:
        return ReferenceProgram(
            source_file=path, controller='FANUC', waypoints=[],
            parse_warnings=['missing /MN or /POS section'],
        )
    mn_lines = text[mn_start:pos_start].splitlines()
    positions = _parse_positions(text[pos_start:])

    warnings: list[str] = []
    waypoints: list[ReferenceWaypoint] = []
    referenced: set[int] = set()
    state = ProcessState(gun_on=None)
    exec_index = 0

    for line_no, line in enumerate(mn_lines):
        m_preset = _PRESET_RE.match(line)
        if m_preset:
            state = state.copy()
            state.modal_notes.append(f'Preset[{m_preset.group("n")}]')
            waypoints.append(ReferenceWaypoint(
                source_id=f'Preset[{m_preset.group("n")}]', exec_index=exec_index,
                position=None, orientation=None, frame_type=ReferenceFrameType.OBJECT,
                move_type='PRESET', process_state=state.copy(), speed_ref=None,
                is_motion=False, raw_line_refs=[line_no],
            ))
            exec_index += 1
            continue

        m_do = _DO_RE.match(line)
        if m_do:
            state = state.copy()
            state.digital_outputs[int(m_do.group('n'))] = (m_do.group('val') == 'ON')
            waypoints.append(ReferenceWaypoint(
                source_id=f'DO[{m_do.group("n")}]', exec_index=exec_index,
                position=None, orientation=None, frame_type=ReferenceFrameType.OBJECT,
                move_type='DO', process_state=state.copy(), speed_ref=None,
                is_motion=False, raw_line_refs=[line_no],
            ))
            exec_index += 1
            continue

        m_motion = _MOTION_RE.match(line)
        if m_motion:
            if m_motion.group('gun'):
                state = state.copy()
                state.gun_on = 'ON' in m_motion.group('gun')
            pid = int(m_motion.group('id'))
            referenced.add(pid)
            pos_wpr = positions.get(pid)
            if pos_wpr is None:
                warnings.append(f"line {line_no}: motion references undeclared P[{pid}]")
            waypoints.append(ReferenceWaypoint(
                source_id=f'P[{pid}]', exec_index=exec_index,
                position=pos_wpr[0] if pos_wpr else None,
                orientation=pos_wpr[1] if pos_wpr else None,
                frame_type=ReferenceFrameType.OBJECT,
                move_type=m_motion.group('move'),
                process_state=state.copy(),
                speed_ref=m_motion.group('speed'),
                is_motion=True,
                raw_line_refs=[line_no],
            ))
            exec_index += 1
            continue

        stripped = line.strip()
        after_num = re.sub(r'^\d+:\s*', '', stripped)
        if stripped and stripped != '/MN' and after_num not in ('', ';') and not after_num.startswith('!'):
            warnings.append(f"line {line_no}: unrecognized /MN line skipped: {stripped!r}")

    unreferenced = sorted(f'P[{pid}]' for pid in (set(positions) - referenced))

    return ReferenceProgram(
        source_file=path,
        controller='FANUC',
        waypoints=waypoints,
        unreferenced_source_ids=unreferenced,
        dead_procedures=[],
        parse_warnings=warnings,
    )
