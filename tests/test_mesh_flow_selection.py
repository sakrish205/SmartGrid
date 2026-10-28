"""Mesh Flow region-selection exit-gate tests: selection is INVERTED relative
to Boundary Box/Face Grid — the region(s) the user clicks are EXCLUDED (e.g. a
hanger/mount point), and generation runs on the complement of the fixed
6-region universe (TOP/BOTTOM/FRONT/REAR/LEFT/RIGHT).

Drives MainWindow._generate_mesh_flow directly against a minimal fake window
object (duck-typed) instead of constructing a full MainWindow, so this stays
fast and isolated from viewer/mesh-loading machinery; _MeshFlowWorker itself
is stubbed so no real region/route generation runs.
"""
from __future__ import annotations
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PySide6.QtWidgets import QApplication, QMessageBox, QWidget

import app.main_window as main_window_module
from app.main_window import MainWindow, _ALL_REGIONS


@pytest.fixture(scope='module')
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


class _FakeRibbon:
    def get_waypoint_spacing_mm(self) -> float:
        return 2.0

    def get_standoff_mm(self) -> float:
        return 15.0

    def get_cover_all(self) -> bool:
        return False

    def set_generating(self, generating: bool) -> None:
        pass


class _FakeViewer:
    def clear_face_grid_planes(self) -> None:
        pass

    def show_bbox(self, flag: bool) -> None:
        pass


class _FakeWorker:
    """Stand-in for _MeshFlowWorker: captures constructor args, never runs."""
    instances: list['_FakeWorker'] = []

    def __init__(self, mesh_data, selected_region_ids, spray_mm, waypoint_spacing_mm=0.0, standoff_mm=0.0, cover_all=False):
        self.selected_region_ids = selected_region_ids
        self.standoff_mm = standoff_mm
        self.finished = types.SimpleNamespace(connect=lambda *_a, **_k: None)
        self.error = types.SimpleNamespace(connect=lambda *_a, **_k: None)
        _FakeWorker.instances.append(self)

    def start(self) -> None:
        pass


def _make_fake_window(selected_regions: set[str]) -> QWidget:
    win = QWidget()
    win._selected_regions = set(selected_regions)
    win._ribbon = _FakeRibbon()
    win._viewer = _FakeViewer()
    win._model = types.SimpleNamespace(data=object())
    win._face_grid_planes_cache = None
    win._adaptive_grid_cache = None
    win._worker = None
    win._on_route_ready = lambda *_a: None
    win._on_route_error = lambda *_a: None
    win._on_mesh_flow_route_ready = lambda *_a: None
    win.statusBar = lambda: types.SimpleNamespace(showMessage=lambda *_a: None)
    return win


@pytest.fixture(autouse=True)
def _reset_fake_worker(monkeypatch):
    _FakeWorker.instances = []
    monkeypatch.setattr(main_window_module, '_MeshFlowWorker', _FakeWorker)
    monkeypatch.setattr(QMessageBox, 'warning', staticmethod(lambda *a, **k: None))


def test_excluding_one_region_generates_on_the_complement(qapp):
    win = _make_fake_window({'BOTTOM'})
    MainWindow._generate_mesh_flow(win, spray_mm=10.0)
    assert len(_FakeWorker.instances) == 1
    assert _FakeWorker.instances[0].selected_region_ids == sorted(_ALL_REGIONS - {'BOTTOM'})


def test_excluding_two_regions_generates_on_the_remaining_four(qapp):
    win = _make_fake_window({'FRONT', 'LEFT'})
    MainWindow._generate_mesh_flow(win, spray_mm=10.0)
    assert _FakeWorker.instances[0].selected_region_ids == sorted(_ALL_REGIONS - {'FRONT', 'LEFT'})


def test_no_exclusion_selected_blocks_generation(qapp):
    win = _make_fake_window(set())
    MainWindow._generate_mesh_flow(win, spray_mm=10.0)
    assert _FakeWorker.instances == []


def test_excluding_every_region_blocks_generation(qapp):
    win = _make_fake_window(set(_ALL_REGIONS))
    MainWindow._generate_mesh_flow(win, spray_mm=10.0)
    assert _FakeWorker.instances == []


def test_generation_forwards_the_ribbon_standoff_value(qapp):
    win = _make_fake_window({'BOTTOM'})
    MainWindow._generate_mesh_flow(win, spray_mm=10.0)
    assert _FakeWorker.instances[0].standoff_mm == 15.0


def test_dropped_regions_are_warned_about_and_routes_still_forwarded(qapp, monkeypatch):
    warnings = []
    monkeypatch.setattr(QMessageBox, 'warning',
                         staticmethod(lambda *a, **k: warnings.append(a)))
    received = []
    win = _make_fake_window({'BOTTOM'})
    win._on_route_ready = lambda routes: received.append(routes)

    MainWindow._on_mesh_flow_route_ready(win, (['route-stub'], ['REAR', 'LEFT']))

    assert len(warnings) == 1
    assert 'LEFT' in warnings[0][2] and 'REAR' in warnings[0][2]
    assert received == [['route-stub']]


def test_no_dropped_regions_means_no_warning(qapp, monkeypatch):
    warnings = []
    monkeypatch.setattr(QMessageBox, 'warning',
                         staticmethod(lambda *a, **k: warnings.append(a)))
    received = []
    win = _make_fake_window({'BOTTOM'})
    win._on_route_ready = lambda routes: received.append(routes)

    MainWindow._on_mesh_flow_route_ready(win, (['route-stub'], []))

    assert warnings == []
    assert received == [['route-stub']]
