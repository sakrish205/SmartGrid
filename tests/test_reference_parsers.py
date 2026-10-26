"""Golden-fixture tests for the Mesh Flow Phase 1 reference-OLP parsers.

`olp review details/` is gitignored (client-supplied sample programs, not
committed) — these tests skip gracefully wherever it isn't present locally,
rather than depending on it being checked in.
"""
from __future__ import annotations
import os

import pytest

from app.reference.parsers import abb_parser, durr_parser, fanuc_parser

_REF_DIR = os.path.join(os.path.dirname(__file__), '..', 'olp review details')
_ABB_PATH = os.path.join(_REF_DIR, 'Exercise_2.mod')
_FANUC_PATH = os.path.join(_REF_DIR, 'Fanuc_Ref', 'S1_100.LS')
_DURR_ALETA_TID = os.path.join(_REF_DIR, 'Durr_Ref', 'B05.tt', 'Aleta_Ptras.tid')
_DURR_ALETA_TIP = os.path.join(_REF_DIR, 'Durr_Ref', 'B05.tt', 'Aleta_Ptras.tip')
_DURR_XPOS_TID = os.path.join(_REF_DIR, 'Durr_Ref', 'B05.tt', 'xpos_test.tid')
_DURR_XPOS_TIP = os.path.join(_REF_DIR, 'Durr_Ref', 'B05.tt', 'xpos_test.tip')

_requires_abb = pytest.mark.skipif(not os.path.exists(_ABB_PATH), reason='olp review details/ not present locally')
_requires_fanuc = pytest.mark.skipif(not os.path.exists(_FANUC_PATH), reason='olp review details/ not present locally')
_requires_durr = pytest.mark.skipif(not os.path.exists(_DURR_ALETA_TID), reason='olp review details/ not present locally')
_requires_durr_xpos = pytest.mark.skipif(not os.path.exists(_DURR_XPOS_TID), reason='olp review details/ not present locally')


@_requires_abb
def test_abb_exercise2_exec_order_and_dead_procedure():
    prog = abb_parser.parse(_ABB_PATH)

    # Only mvHood() is reachable from runJob(); mvSideA() is declared but dead.
    assert prog.dead_procedures == ['mvSideA']

    # Exercise_2.mod has no SETBRUSH/gun toggles — every PaintL implies gun on.
    assert len(prog.waypoints) == 24
    assert all(w.move_type == 'PaintL' for w in prog.waypoints)
    assert all(w.process_state.gun_on is True for w in prog.waypoints)

    # Execution order must be the actual mvHood() call sequence, not target-name order.
    ids = [w.source_id for w in prog.waypoints]
    assert ids[:4] == ['p220', 'p230', 'p210', 'p10']

    # Every declared target is used somewhere (mvHood or the dead mvSideA) — none orphaned.
    assert prog.unreferenced_source_ids == []


@_requires_fanuc
def test_fanuc_s1_100_exec_order_and_modal_state():
    prog = fanuc_parser.parse(_FANUC_PATH)

    motion = [w for w in prog.waypoints if w.is_motion]
    assert len(motion) == 63

    # Proven irregularity: /MN calls P[33] then P[42] then P[34] — NOT numeric order.
    ids = [w.source_id for w in motion]
    assert ids.index('P[42]') < ids.index('P[34]')
    assert ids.index('P[33]') < ids.index('P[42]')

    # Gun toggled exactly once ON (first move) and once OFF (near the end); modal in between.
    assert motion[0].process_state.gun_on is True
    gun_off_idx = next(i for i, w in enumerate(motion) if w.process_state.gun_on is False)
    assert motion[gun_off_idx - 1].process_state.gun_on is True  # held modally until the toggle

    # Preset[n] lines are modal, carry no position, and Preset[3] recurs (non-monotonic).
    presets = [w for w in prog.waypoints if w.move_type == 'PRESET']
    preset_ns = [w.source_id for w in presets]
    assert preset_ns.count('Preset[3]') == 2
    assert all(w.position is None for w in presets)

    # Bare digital-output line has no point reference.
    do_lines = [w for w in prog.waypoints if w.move_type == 'DO']
    assert len(do_lines) == 1
    assert do_lines[0].position is None

    assert prog.unreferenced_source_ids == []


@_requires_durr
def test_durr_aleta_orphaned_points_and_trigger_pairing():
    prog = durr_parser.parse(_DURR_ALETA_TID, _DURR_ALETA_TIP)

    motion = [w for w in prog.waypoints if w.is_motion]
    assert len(motion) == 90  # LIN(P1) .. LIN(P90)
    assert [w.source_id for w in motion[:3]] == ['P1', 'P2', 'P3']

    # ~97 of 175 declared P-points, plus any unused PT/TR, are never referenced.
    assert len(prog.unreferenced_source_ids) > 80

    # SETBRUSH and the following GUN(GunOn) use distinct PT/TR pairs.
    setbrush = next(w for w in prog.waypoints if w.move_type == 'SETBRUSH')
    gun_on = next(w for w in prog.waypoints if w.move_type == 'GUN' and w.process_state.gun_on)
    assert setbrush.source_id != gun_on.source_id


@_requires_durr_xpos
def test_durr_xpos_test_only_five_executed():
    prog = durr_parser.parse(_DURR_XPOS_TID, _DURR_XPOS_TIP)

    motion = [w for w in prog.waypoints if w.is_motion]
    assert [w.source_id for w in motion] == ['P110', 'P1102', 'P89', 'P0', 'P1101']

    # The one Axis-frame point (P85) is declared but never executed in this file.
    assert 'P85' in prog.unreferenced_source_ids
    assert not any(w.source_id == 'P85' for w in prog.waypoints)

    # 93 declared P-points + 23 PT = 116 POINT declarations (TR entries are a
    # separate TRIGGERFRAME table, not POINTs); only 5 are referenced.
    assert len(prog.unreferenced_source_ids) == 116 - 5
