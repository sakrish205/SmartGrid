"""Mesh Flow Phase 8 exit-gate tests: the rename mechanism's safe default,
and that it never breaks loading an existing project."""
from __future__ import annotations
import pytest

import app.meshflow.legacy_aliases as legacy_aliases
from app.meshflow.legacy_aliases import RenamePolicy, display_label_for, resolve_path_mode_for_load
from app.meshflow.open_decisions import OpenDecisionBlocked


def test_default_policy_is_label_only():
    assert legacy_aliases.ACTIVE_RENAME_POLICY is RenamePolicy.LABEL_ONLY


def test_resolve_path_mode_for_load_always_succeeds_under_default_policy():
    # Regression-proofs the "don't break old projects" requirement directly —
    # no setup, no monkeypatch, just the module's real default.
    assert resolve_path_mode_for_load('Mesh Surface') == 'Mesh Surface'
    assert resolve_path_mode_for_load('Mesh Flow') == 'Mesh Flow'


def test_version_flag_policy_raises_open_decision_blocked(monkeypatch):
    monkeypatch.setattr(legacy_aliases, 'ACTIVE_RENAME_POLICY', RenamePolicy.VERSION_FLAG)
    with pytest.raises(OpenDecisionBlocked) as exc_info:
        resolve_path_mode_for_load('Mesh Surface')
    assert exc_info.value.decision_no == 6


def test_migrate_policy_round_trips_correctly(monkeypatch):
    monkeypatch.setattr(legacy_aliases, 'ACTIVE_RENAME_POLICY', RenamePolicy.MIGRATE)
    assert resolve_path_mode_for_load('Mesh Surface') == 'Mesh Flow'
    assert resolve_path_mode_for_load('Mesh Flow') == 'Mesh Flow'   # already-current value untouched


def test_dual_write_policy_is_passthrough(monkeypatch):
    monkeypatch.setattr(legacy_aliases, 'ACTIVE_RENAME_POLICY', RenamePolicy.DUAL_WRITE)
    assert resolve_path_mode_for_load('Mesh Surface') == 'Mesh Surface'
    assert resolve_path_mode_for_load('Mesh Flow') == 'Mesh Flow'


def test_display_label_for_is_identity_until_the_rename_is_actually_applied():
    # Deliberately not wired up yet — see legacy_aliases.py's bottom-of-file
    # note for why. Mesh Flow still coexists as a separate beta mode.
    assert display_label_for('Mesh Surface') == 'Mesh Surface'
    assert display_label_for('Mesh Flow') == 'Mesh Flow'
