"""Mesh Flow Phase 3 exit-gate test: the feature-flag dispatch leaf in ribbon.py.

Runs headless (offscreen Qt platform) so it works without a display.
"""
from __future__ import annotations
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PySide6.QtWidgets import QApplication

import app.ui.ribbon as ribbon_module
from app.ui.ribbon import SmartRibbon


@pytest.fixture(scope='module')
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


def test_get_path_target_returns_mesh_flow_when_checked(qapp, monkeypatch):
    monkeypatch.setattr(ribbon_module, 'MESH_FLOW_UI_ENABLED', True)
    rb = SmartRibbon()
    rb._mesh_flow_radio.setChecked(True)
    assert rb.get_path_target() == 'mesh_flow'


def test_mesh_flow_radio_visibility_follows_flag_disabled(qapp, monkeypatch):
    monkeypatch.setattr(ribbon_module, 'MESH_FLOW_UI_ENABLED', False)
    rb = SmartRibbon()
    assert rb._mesh_flow_radio.isVisible() is False


def test_mesh_flow_radio_visibility_follows_flag_enabled(qapp, monkeypatch):
    monkeypatch.setattr(ribbon_module, 'MESH_FLOW_UI_ENABLED', True)
    rb = SmartRibbon()
    rb.show()
    assert rb._mesh_flow_radio.isVisible() is True


def test_default_path_target_is_unaffected_by_mesh_flow_flag(qapp, monkeypatch):
    monkeypatch.setattr(ribbon_module, 'MESH_FLOW_UI_ENABLED', True)
    rb = SmartRibbon()
    assert rb.get_path_target() == 'bbox'   # bbox is still checked by default
