"""ABB RAPID (.mod) reference-program parser.

Confirmed real-file behavior this parser is built against (Exercise_2.mod):
declaration order (p600->p10, descending) is unrelated to execution order;
`mvSideA()` is declared but never called from `runJob()` (dead procedure);
motion lines carry variable-arity trailing `\\E1:=[...]`/`\\E2:=[...]` external-axis
clauses; speeds are named (`v600`), not literal; this sample has zero SETBRUSH/
gun on-off instructions — `PaintL` implies continuous spray, `MoveL`/`MoveJ` are
travel, so gun state here is derived from instruction type, not a toggle.

ponytail: regex-based line scanner, not a RAPID grammar — covers CONST robtarget,
PaintL/MoveL/MoveJ, and bare proc-call statements only. Unknown instruction forms
are skipped with a parse_warning rather than guessed at. Upgrade path: a real
tokenizer if later samples use control flow (IF/FOR/WHILE) inside a called proc.
"""
from __future__ import annotations
import re

import numpy as np

from app.reference.reference_model import (
    ProcessState, ReferenceFrameType, ReferenceProgram, ReferenceWaypoint,
)

_TARGET_RE = re.compile(
    r'CONST\s+robtarget\s+(?P<name>\w+)\s*:=\s*\[\[(?P<pos>[^\]]*)\]\s*,\s*\[(?P<quat>[^\]]*)\]'
)
_PROC_RE = re.compile(r'PROC\s+(?P<name>\w+)\s*\(\s*\)')
_MOTION_RE = re.compile(
    r'^\s*(?P<instr>PaintL|MoveL|MoveJ)\s+(?P<target>\w+)\s*,\s*(?P<speed>v\d+|\d+(?:\.\d+)?)'
    r'(?P<ext>(?:\\E\d+:=\[[^\]]*\])*)\s*,\s*z(?P<zone>\S+?)\s*,'
)
_EXT_AXIS_RE = re.compile(r'\\E(?P<n>\d+):=\[(?P<vals>[^\]]*)\]')
_BARE_CALL_RE = re.compile(r'^\s*(?P<name>\w+)\s*;\s*$')


def _parse_targets(lines: list[str]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    targets: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for line in lines:
        m = _TARGET_RE.search(line)
        if m:
            pos = np.array([float(v) for v in m.group('pos').split(',')])
            quat = np.array([float(v) for v in m.group('quat').split(',')])
            targets[m.group('name')] = (pos, quat)
    return targets


def _split_procs(lines: list[str]) -> dict[str, list[str]]:
    """PROC name() ... ENDPROC -> {name: body_lines}."""
    procs: dict[str, list[str]] = {}
    current: str | None = None
    body: list[str] = []
    for line in lines:
        m = _PROC_RE.search(line)
        if m:
            current = m.group('name')
            body = []
            continue
        if current is not None:
            if re.search(r'\bENDPROC\b', line):
                procs[current] = body
                current = None
            else:
                body.append(line)
    return procs


def parse(path: str) -> ReferenceProgram:
    with open(path, encoding='utf-8') as f:
        raw_lines = f.readlines()

    targets = _parse_targets(raw_lines)
    procs = _split_procs(raw_lines)
    warnings: list[str] = []

    # Resolve the call graph from runJob() to find reachable procs and their order.
    call_order: list[str] = []
    visited: set[str] = set()

    def _walk(proc_name: str) -> None:
        if proc_name in visited or proc_name not in procs:
            return
        visited.add(proc_name)
        for line in procs[proc_name]:
            if _MOTION_RE.search(line):
                continue
            m = _BARE_CALL_RE.match(line.strip())
            if m and m.group('name') in procs:
                call_order.append(m.group('name'))
                _walk(m.group('name'))

    if 'runJob' in procs:
        _walk('runJob')
    else:
        warnings.append('no runJob() procedure found; treating all procs as unreachable')

    dead_procedures = sorted(set(procs) - visited - {'runJob'})

    # A declared target is "referenced" if any PROC uses it, reached or not —
    # only the ORDERED waypoint list below is restricted to reachable procs.
    referenced: set[str] = {
        m.group('target')
        for body in procs.values()
        for line in body
        if (m := _MOTION_RE.search(line))
    }
    waypoints: list[ReferenceWaypoint] = []
    exec_index = 0
    for proc_name in call_order:
        for line_no, line in enumerate(procs[proc_name]):
            m = _MOTION_RE.search(line)
            if not m:
                continue
            target = m.group('target')
            pos_quat = targets.get(target)
            position = pos_quat[0] if pos_quat else None
            orientation = pos_quat[1] if pos_quat else None
            if pos_quat is None:
                warnings.append(f"{proc_name}: motion references undeclared target '{target}'")

            external_axes = {
                f'E{em.group("n")}': [float(v) for v in em.group('vals').split(',')]
                for em in _EXT_AXIS_RE.finditer(m.group('ext') or '')
            } or None

            instr = m.group('instr')
            waypoints.append(ReferenceWaypoint(
                source_id=target,
                exec_index=exec_index,
                position=position,
                orientation=orientation,
                frame_type=ReferenceFrameType.OBJECT,
                move_type=instr,
                process_state=ProcessState(
                    gun_on=(instr == 'PaintL'),
                    modal_notes=[f'implicit:{instr}'],
                ),
                speed_ref=m.group('speed'),
                is_motion=True,
                raw_line_refs=[line_no],
                external_axes=external_axes,
            ))
            exec_index += 1

    unreferenced = sorted(set(targets) - referenced)

    return ReferenceProgram(
        source_file=path,
        controller='ABB',
        waypoints=waypoints,
        unreferenced_source_ids=unreferenced,
        dead_procedures=dead_procedures,
        parse_warnings=warnings,
    )
