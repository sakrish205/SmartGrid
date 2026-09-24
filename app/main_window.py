"""Top-level window — ribbon layout, no sidebar."""
from __future__ import annotations
import json
import math
import os
import pathlib
import traceback
from dataclasses import replace as _dc_replace
from typing import Optional

import numpy as np

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFileDialog, QMessageBox, QDialog,
    QDialogButtonBox, QRadioButton, QButtonGroup,
    QLabel, QScrollArea, QFrame,
    QComboBox, QDoubleSpinBox, QTextBrowser,
)
from PySide6.QtCore import Qt, QThread, Signal, QEvent, QTimer, QUrl
from PySide6.QtGui import QAction, QDragEnterEvent, QDropEvent, QDesktopServices

# Heavy imports (trimesh, pyvista, pyvistaqt) are deferred to first use
# so the window appears before the 2-3 s cold-start load completes.
from app.path.path_model import PaintRoute, GenerationParams
from app.ui.ribbon import SmartRibbon
from app.export.json_export import export_route_json
from app.export.csv_export import export_route_csv
from app.export.olp_export import export_robodk, export_vc, export_delmia_apt, export_gcode
from app.ui.view_settings_dialog import ViewSettingsDialog, DEFAULTS as _COLOR_DEFAULTS


_SIDE_REGIONS = frozenset({'FRONT', 'REAR', 'LEFT', 'RIGHT'})
_TB_REGIONS   = frozenset({'TOP', 'BOTTOM'})


def _group_for_mesh(selected_regions, model):
    """Combine side regions into one group and top/bottom into another."""
    groups = []
    for bucket in (_SIDE_REGIONS, _TB_REGIONS):
        members = sorted(selected_regions & bucket)
        if not members:
            continue
        arrays = [np.asarray(model.get_region_faces(r), dtype=np.int64) for r in members]
        arrays = [a for a in arrays if len(a) > 0]
        if not arrays:
            continue
        name = '+'.join(members) if len(members) > 1 else members[0]
        groups.append((name, np.unique(np.concatenate(arrays))))
    return groups


def _group_for_face_grid(selected_regions, model):
    """Combine all selected regions into a single group."""
    members = sorted(selected_regions)
    arrays = [np.asarray(model.get_region_faces(r), dtype=np.int64) for r in members]
    arrays = [a for a in arrays if len(a) > 0]
    if not arrays:
        return []
    name = '+'.join(members) if len(members) > 1 else members[0]
    return [(name, np.unique(np.concatenate(arrays)))]


def _detect_unit(max_extent: float) -> str:
    if max_extent > 100:
        return 'mm'
    if max_extent > 1:
        return 'cm'
    return 'm'


# ---------------------------------------------------------------------------
# Dynamic speed — one place to tune all auto-speed behaviour
# ---------------------------------------------------------------------------

_SPEED_CURVE = {
    'max_mmpm':        2000.0,   # speed on perfectly straight segments
    'min_mmpm':         300.0,   # floor at maximum curvature
    'angle_threshold':   30.0,   # degrees — at or above this angle → min speed
}


def _compute_dynamic_speeds(points: np.ndarray) -> np.ndarray:
    """Per-waypoint speed derived from local turning angle. Straight=fast, curved=slow."""
    n = len(points)
    max_s = _SPEED_CURVE['max_mmpm']
    min_s = _SPEED_CURVE['min_mmpm']
    if n < 3:
        return np.full(n, max_s)

    threshold_rad = np.radians(_SPEED_CURVE['angle_threshold'])

    v1 = points[1:-1] - points[:-2]     # (n-2, 3) — incoming vectors
    v2 = points[2:]   - points[1:-1]    # (n-2, 3) — outgoing vectors

    n1 = np.linalg.norm(v1, axis=1, keepdims=True)
    n2 = np.linalg.norm(v2, axis=1, keepdims=True)

    valid = (n1.ravel() > 1e-9) & (n2.ravel() > 1e-9)
    cos_a = np.ones(n - 2)
    cos_a[valid] = np.clip(
        np.sum((v1[valid] / n1[valid]) * (v2[valid] / n2[valid]), axis=1),
        -1.0, 1.0,
    )
    angles = np.arccos(cos_a)
    t = np.minimum(angles / threshold_rad, 1.0)
    mid_speeds = max_s - t * (max_s - min_s)

    speeds = np.empty(n)
    speeds[0]    = max_s
    speeds[-1]   = max_s
    speeds[1:-1] = mid_speeds
    return speeds


# ---------------------------------------------------------------------------
# Export OLP dialog
# ---------------------------------------------------------------------------

class _OlpExportDialog(QDialog):
    def __init__(self, formats: dict, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle('Export OLP')
        self.setFixedWidth(360)

        vl = QVBoxLayout(self)
        vl.setSpacing(8)

        vl.addWidget(QLabel('Select OLP format:'))
        self._fmt_combo = QComboBox()
        self._fmt_combo.addItems(list(formats.keys()))
        vl.addWidget(self._fmt_combo)

        vl.addWidget(QLabel('Spray Speed:'))

        max_s = int(_SPEED_CURVE['max_mmpm'])
        min_s = int(_SPEED_CURVE['min_mmpm'])
        self._auto_radio   = QRadioButton(
            f'Auto  ({min_s}–{max_s} mm/min)  — varies with path curvature')
        self._custom_radio = QRadioButton('Custom')
        self._auto_radio.setChecked(True)

        self._speed_spin = QDoubleSpinBox()
        self._speed_spin.setRange(1.0, 100_000.0)
        self._speed_spin.setDecimals(0)
        self._speed_spin.setSingleStep(100.0)
        self._speed_spin.setValue(1000.0)
        self._speed_spin.setSuffix('  mm/min')
        self._speed_spin.setMinimumWidth(120)
        self._speed_spin.setEnabled(False)

        custom_row = QHBoxLayout()
        custom_row.addWidget(self._custom_radio)
        custom_row.addWidget(self._speed_spin)
        custom_row.addStretch()

        self._custom_radio.toggled.connect(self._speed_spin.setEnabled)

        vl.addWidget(self._auto_radio)
        vl.addLayout(custom_row)

        btns = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        vl.addSpacing(4)
        vl.addWidget(btns)

    def values(self):
        """Returns (fmt_label: str, speed_mode: str, custom_speed: float | None)."""
        mode = 'custom' if self._custom_radio.isChecked() else 'auto'
        return (
            self._fmt_combo.currentText(),
            mode,
            self._speed_spin.value() if mode == 'custom' else None,
        )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _offset_route_by_standoff(route: 'PaintRoute', mesh, standoff_mm: float) -> 'PaintRoute':
    """Shift every waypoint outward along the nearest mesh face normal."""
    import trimesh.proximity as _prox
    from app.path.path_model import PaintPass, Connection, PaintRoute as _PR

    def _offset_pts(pts: np.ndarray) -> np.ndarray:
        if len(pts) == 0:
            return pts
        _, _, face_ids = _prox.closest_point(mesh, pts)
        normals = mesh.face_normals[face_ids]
        return pts + normals * standoff_mm

    new_passes = []
    for p in route.passes:
        new_passes.append(PaintPass(
            id=p.id, region_id=p.region_id, direction=p.direction,
            points=_offset_pts(p.points),
            is_forward=p.is_forward, sub_index=p.sub_index,
            slice_position=p.slice_position,
        ))
    new_conns = []
    for c in route.connections:
        new_conns.append(Connection(
            id=c.id, from_pass_id=c.from_pass_id, to_pass_id=c.to_pass_id,
            points=_offset_pts(c.points),
            is_air_move=c.is_air_move,
        ))
    total_length = sum(
        float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1)))
        for p in new_passes if len(p.points) >= 2
    )
    return _PR(
        region_id=route.region_id, passes=new_passes, connections=new_conns,
        unit=route.unit, spacing_mm=route.spacing_mm,
        total_passes=route.total_passes, total_length_mm=total_length,
        spray_normal=route.spray_normal.copy(),
    )


# ---------------------------------------------------------------------------
# Background workers
# ---------------------------------------------------------------------------

class _LoadWorker(QThread):
    finished = Signal(object)
    progress = Signal(str)
    error    = Signal(str)

    def __init__(self, filepath: str, up_axis: int) -> None:
        super().__init__()
        self._filepath = filepath
        self._up_axis  = up_axis

    def run(self) -> None:
        try:
            from models.mesh_model import MeshModel as _MM
            self.progress.emit(f'Reading {os.path.basename(self._filepath)}...')
            model = _MM()
            model.load(self._filepath, up_axis=self._up_axis)
            self.progress.emit('Finalising...')
            self.finished.emit(model)
        except Exception as exc:
            self.error.emit(f'{type(exc).__name__}: {exc}\n{traceback.format_exc()}')


class _PathWorker(QThread):
    finished = Signal(object)
    error    = Signal(str)

    def __init__(
        self,
        mesh_data,
        pairs: list,
        spray_mm: float,
        waypoint_spacing_mm: float = 0.0,
        standoff_mm: float = 0.0,
        direction: str = 'horizontal',
    ) -> None:
        super().__init__()
        self._mesh_data        = mesh_data
        self._pairs            = pairs
        self._spray_mm         = spray_mm
        self._waypoint_spacing = waypoint_spacing_mm
        self._standoff_mm      = standoff_mm
        self._direction        = direction

    def run(self) -> None:
        try:
            from app.path import generator as _gen
            routes = []
            mesh = self._mesh_data.trimesh_mesh
            for region_id, face_indices in self._pairs:
                route = _gen.generate_route(
                    self._mesh_data,
                    region_id=region_id,
                    region_face_indices=face_indices,
                    spray_width_mm=self._spray_mm,
                    waypoint_spacing_mm=self._waypoint_spacing,
                    direction=self._direction,
                )
                if self._standoff_mm > 0.0:
                    route = _offset_route_by_standoff(route, mesh, self._standoff_mm)
                routes.append(route)
            self.finished.emit(routes)
        except Exception as exc:
            self.error.emit(f'{type(exc).__name__}: {exc}\n{traceback.format_exc()}')


class _CollisionWorker(QThread):
    finished = Signal(object)  # tuple[dict[int, str], float]  (collision_ids, suggested_standoff)
    error    = Signal(str)

    def __init__(self, routes, mesh, standoff_mm: float) -> None:
        super().__init__()
        self._routes   = routes
        self._mesh     = mesh
        self._standoff = standoff_mm

    def run(self) -> None:
        try:
            from app.path.collision import detect_collisions
            collision_ids, max_depth = detect_collisions(self._routes, self._mesh, self._standoff)
            suggested = 0.0
            if collision_ids:
                has_hard = max_depth > 0
                raw = (self._standoff + max_depth + 2.0) if has_hard else (self._standoff * 2.0 + 2.0)
                suggested = math.ceil(raw / 5.0) * 5.0
            self.finished.emit((collision_ids, suggested))
        except Exception as exc:
            self.error.emit(f'{type(exc).__name__}: {exc}\n{traceback.format_exc()}')


# ---------------------------------------------------------------------------
# Up-axis dialog
# ---------------------------------------------------------------------------

class _UpAxisDialog(QDialog):
    def __init__(self, filename: str, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle('Open Mesh')
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.addWidget(QLabel(f'<b>{os.path.basename(filename)}</b>'))
        layout.addWidget(QLabel('Which axis is UP in this file?'))
        self._group = QButtonGroup(self)
        self._btns: list[QRadioButton] = []
        for i, label in enumerate(['X  (axis 0)', 'Y  (axis 1)', 'Z  (axis 2)  —  default']):
            btn = QRadioButton(label)
            self._group.addButton(btn, i)
            layout.addWidget(btn)
            self._btns.append(btn)
        self._btns[2].setChecked(True)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        buttons.accepted.connect(self.accept)
        layout.addWidget(buttons)

    def up_axis(self) -> int:
        return self._group.checkedId()


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class _ViewerImportWorker(QThread):
    """Warm pyvista/VTK and trimesh in a background thread.

    Imports are pure Python/VTK — no Qt objects created — so this is
    safe off the main thread. Only QtInteractor instantiation (widget
    creation) must remain on the main thread.
    """
    done = Signal()

    def run(self) -> None:
        import app.ui.viewer      # pyvista/VTK cold-load (~1-3 s)
        import models.mesh_model  # trimesh cold-load (~0.5-1 s)
        self.done.emit()


class MainWindow(QMainWindow):

    _MENUBAR_STYLE = (
        'QMenuBar{background:#f0f0f0;color:#1f1f1f;font-size:12px;'
        '  font-family:"Segoe UI",Arial;border-bottom:1px solid #b0b0b0;}'
        'QMenuBar::item{padding:4px 10px;background:transparent;border-radius:0px;}'
        'QMenuBar::item:selected{background:#e5e5e5;border:1px solid #b0b0b0;}'
        'QMenuBar::item:pressed{background:#d0d0d0;}'
        'QMenu{background:#ffffff;color:#1f1f1f;font-size:12px;'
        '  font-family:"Segoe UI",Arial;border:1px solid #b0b0b0;border-radius:0px;}'
        'QMenu::item{padding:5px 24px 5px 12px;border-radius:0px;}'
        'QMenu::item:selected{background:#0078d4;color:#ffffff;}'
        'QMenu::separator{height:1px;background:#d0d0d0;margin:2px 0;}'
    )

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('SmartGrid  —  3D Surface Grid & Pitch Mapper')
        self.resize(1600, 860)
        self.setAcceptDrops(True)
        self.setStyleSheet(
            'QMainWindow{background:#e8e8e8;}'
            'QStatusBar{background:#f0f0f0;color:#333;font-size:11px;'
            '  font-family:"Segoe UI",Arial;border-top:1px solid #b0b0b0;'
            '  padding:0 4px;}'
            'QStatusBar::item{border:none;}'
        )
        self.menuBar().setStyleSheet(self._MENUBAR_STYLE)

        self._model              = None          # set after heavy imports load
        self._viewer             = None          # set after viewer deferred init
        self._selected_regions:  set[str]        = set()
        self._current_routes:    list[PaintRoute] = []
        self._collision_ids:     dict[int, str]   = {}
        self._last_params:       GenerationParams | None = None
        self._worker:        Optional[QThread] = None
        self._load_worker:   Optional[QThread] = None
        self._coll_worker:   Optional[QThread] = None
        self._zombie_workers: list = []  # coll workers dropped while still running
        self._current_colors: dict[str, str]  = self._load_view_settings()
        self._face_grid_planes_cache: tuple | None = None
        self._adaptive_grid_cache:   tuple | None = None

        self._build_ui()
        self._build_menus()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        # Central widget: ribbon on top, viewer below
        central = QWidget()
        self.setCentralWidget(central)
        vl = QVBoxLayout(central)
        vl.setContentsMargins(0, 0, 0, 0)
        vl.setSpacing(0)

        from app.ui.ribbon import RIBBON_H
        self._ribbon = SmartRibbon()
        ribbon_scroll = QScrollArea()
        ribbon_scroll.setWidget(self._ribbon)
        ribbon_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        ribbon_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        ribbon_scroll.setWidgetResizable(True)
        ribbon_scroll.setFixedHeight(RIBBON_H + 4)
        ribbon_scroll.setFrameShape(QFrame.Shape.NoFrame)
        ribbon_scroll.setStyleSheet('QScrollArea{background:transparent;border:none;}')
        vl.addWidget(ribbon_scroll)

        # Placeholder shown while heavy imports load in the background
        self._viewer_placeholder = QLabel('Loading viewer…')
        self._viewer_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._viewer_placeholder.setStyleSheet(
            'color:#666;font-size:13px;font-family:"Segoe UI",Arial;'
            'background:#e8e8e8;')
        self._viewer_vl = vl                  # saved so _init_viewer can splice in
        vl.addWidget(self._viewer_placeholder, stretch=1)

        # Initial state — ribbon enabled after viewer is ready
        self._ribbon.set_model_loaded(False)
        self.statusBar().showMessage('Starting up…')

        # Signals that do not touch the viewer — safe to wire now
        self._ribbon.open_requested.connect(self._open_file)
        self._ribbon.region_toggled.connect(self._on_region_shortcut)
        self._ribbon.select_mode_changed.connect(self._on_select_mode_changed)
        self._ribbon.generate_requested.connect(self._on_generate)
        self._ribbon.clear_requested.connect(self._clear_paths)
        self._ribbon.sweep_changed.connect(self._on_sweep_changed)
        self._ribbon.export_json.connect(self._export_json)
        self._ribbon.export_csv.connect(self._export_csv)
        self._ribbon.export_olp.connect(self._export_olp)
        self._ribbon.view_settings_req.connect(self._open_view_settings)

        # Defer heavy imports (trimesh, pyvista, pyvistaqt) to after first paint
        QTimer.singleShot(0, self._init_viewer)

    def _init_viewer(self) -> None:
        """Kick off background import worker; main thread stays live throughout."""
        self._import_worker = _ViewerImportWorker()
        self._import_worker.done.connect(self._create_viewer)
        self._import_worker.start()

    def _create_viewer(self) -> None:
        """Main-thread only: imports already warm — only QtInteractor blocks (~1-2 s)."""
        from app.ui.viewer import MeshViewer    # instant — already in sys.modules
        from models.mesh_model import MeshModel # instant — already in sys.modules
        # Only create a blank model if no file was loaded while viewer was initialising
        if self._model is None:
            self._model = MeshModel()
        self._viewer = MeshViewer()             # QtInteractor OpenGL init — main thread required

        # Replace placeholder with the real viewer
        idx = self._viewer_vl.indexOf(self._viewer_placeholder)
        self._viewer_vl.insertWidget(idx, self._viewer, stretch=1)
        self._viewer_vl.removeWidget(self._viewer_placeholder)
        self._viewer_placeholder.deleteLater()
        self._viewer_placeholder = None

        self._viewer.installEventFilter(self)

        # Wire the remaining viewer-dependent signals
        self._ribbon.grid_changed.connect(self._update_grid)
        self._ribbon.arrows_changed.connect(self._refresh_route_display)

        # Apply persisted view colours to the now-ready viewer
        self._on_colors_changed(self._current_colors)

        if self._model.data is not None:
            # File was loaded before the viewer finished initialising — apply it now
            self._apply_loaded_model()
        else:
            self.statusBar().showMessage('Ready — open an STL or OBJ file to begin.')

    def _build_menus(self) -> None:
        mb = self.menuBar()

        file_menu = mb.addMenu('File')
        open_act = QAction('Open STL / OBJ / STEP...', self)
        open_act.setShortcut('Ctrl+O')
        open_act.triggered.connect(self._open_file)
        file_menu.addAction(open_act)
        file_menu.addSeparator()
        file_menu.addAction('Exit', self.close)

        view_menu = mb.addMenu('View')
        view_menu.addAction('Fit All\tCtrl+Home', lambda: self._viewer and self._viewer.fit_all())
        view_menu.addAction('Top View',    lambda: self._on_view_set('top'))
        view_menu.addAction('Bottom View', lambda: self._on_view_set('bottom'))
        view_menu.addAction('Front View',  lambda: self._on_view_set('front'))
        view_menu.addAction('Rear View',   lambda: self._on_view_set('rear'))
        view_menu.addAction('Left View',   lambda: self._on_view_set('left'))
        view_menu.addAction('Right View',  lambda: self._on_view_set('right'))
        view_menu.addSeparator()
        view_menu.addAction('Rotate Left  90',  lambda: self._viewer and self._viewer.roll_view(-90))
        view_menu.addAction('Rotate Right 90',  lambda: self._viewer and self._viewer.roll_view(+90))
        view_menu.addSeparator()
        view_menu.addAction('View Settings...', self._open_view_settings)

        path_menu = mb.addMenu('Path')
        gen_act = QAction('Generate', self)
        gen_act.setShortcuts(['Ctrl+G', 'F5'])
        gen_act.triggered.connect(self._on_generate)
        path_menu.addAction(gen_act)
        path_menu.addAction('Flip Direction',   self._flip_direction)
        path_menu.addSeparator()
        path_menu.addAction('Clear Paths', self._clear_paths)

        export_menu = mb.addMenu('Export')
        export_menu.addAction('Export JSON...', self._export_json)
        export_menu.addAction('Export CSV...',  self._export_csv)
        export_menu.addAction('Export OLP...',  self._export_olp)

        help_menu = mb.addMenu('Help')
        help_menu.addAction('How to Use SmartGrid...', self._show_help)
        help_menu.addSeparator()
        help_menu.addAction('GitHub Repository',
            lambda: QDesktopServices.openUrl(QUrl('https://github.com/sakrish205/SmartGrid')))
        help_menu.addSeparator()
        help_menu.addAction('About SmartGrid...', self._show_about)

    # ------------------------------------------------------------------
    # View helpers
    # ------------------------------------------------------------------

    def _on_view_set(self, direction: str) -> None:
        if self._viewer is None:
            return
        self._viewer.set_view(direction)

    # ------------------------------------------------------------------
    # Select mode
    # ------------------------------------------------------------------

    def _on_select_mode_changed(self, active: bool) -> None:
        if self._viewer is None:
            return
        self._viewer.set_select_mode(active)
        if active:
            self.statusBar().showMessage(
                'Select Faces — click a bounding box face. Camera rotation suspended.')
        else:
            self.statusBar().showMessage('Navigate — drag to rotate, scroll to zoom.')

    def eventFilter(self, obj, event) -> bool:
        return super().eventFilter(obj, event)

    # ------------------------------------------------------------------
    # Settings persistence
    # ------------------------------------------------------------------

    _SETTINGS_FILE = pathlib.Path(__file__).parent.parent / 'settings.json'

    def _load_view_settings(self) -> dict[str, str]:
        colors = dict(_COLOR_DEFAULTS)
        try:
            saved = json.loads(self._SETTINGS_FILE.read_text(encoding='utf-8'))
            for key in colors:
                if key in saved:
                    colors[key] = str(saved[key])
        except Exception:
            pass
        return colors

    def _save_view_settings(self) -> None:
        try:
            self._SETTINGS_FILE.write_text(
                json.dumps(self._current_colors, indent=2), encoding='utf-8')
        except Exception:
            pass

    def closeEvent(self, event) -> None:
        self._save_view_settings()
        super().closeEvent(event)

    # ------------------------------------------------------------------
    # View settings dialog
    # ------------------------------------------------------------------

    def _open_view_settings(self) -> None:
        dlg = ViewSettingsDialog(self._current_colors, self)
        dlg.colors_changed.connect(self._on_colors_changed)
        if dlg.exec():
            self._current_colors = dlg.colors
            self._save_view_settings()
        # On cancel: ViewSettingsDialog emits colors_changed with the original
        # colors, which _on_colors_changed already handles — no extra call needed.

    def _on_colors_changed(self, colors: dict[str, str]) -> None:
        if self._viewer is None:
            return
        self._viewer.apply_colors(colors)
        if self._current_routes:
            self._viewer.show_route(
                self._current_routes,
                show_arrows=self._ribbon.is_show_arrows(),
                show_waypoints=self._ribbon.is_show_waypoints(),
                collision_ids=self._collision_ids,
            )

    # ------------------------------------------------------------------
    # Sweep direction
    # ------------------------------------------------------------------

    def _on_sweep_changed(self) -> None:
        if self._current_routes:
            self._on_generate()

    def _flip_direction(self) -> None:
        self._ribbon.flip_sweep_direction()


    # ------------------------------------------------------------------
    # File loading
    # ------------------------------------------------------------------

    def _open_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, 'Open Mesh File', '',
            'Mesh Files (*.stl *.obj *.step *.stp);;All Files (*)')
        if path:
            self._load(path)

    _MESH_EXTS = ('.stl', '.obj', '.step', '.stp')

    def dragEnterEvent(self, event: QDragEnterEvent) -> None:
        if event.mimeData().hasUrls():
            if any(u.toLocalFile().lower().endswith(self._MESH_EXTS)
                   for u in event.mimeData().urls()):
                event.acceptProposedAction()

    def dropEvent(self, event: QDropEvent) -> None:
        for url in event.mimeData().urls():
            path = url.toLocalFile()
            if path.lower().endswith(self._MESH_EXTS):
                self._load(path)
                break

    def _load(self, filepath: str) -> None:
        if self._load_worker or self._worker:
            return
        dlg = _UpAxisDialog(filepath, self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        up_axis = dlg.up_axis()

        self._selected_regions.clear()
        self._current_routes = []
        # Reset region buttons
        for region in ('TOP', 'BOTTOM', 'FRONT', 'REAR', 'LEFT', 'RIGHT'):
            self._ribbon.set_region_checked(region, False)
        self._ribbon.set_model_loaded(False)
        self.statusBar().showMessage(f'Reading {os.path.basename(filepath)}...')

        worker = _LoadWorker(filepath, up_axis)
        worker.progress.connect(self.statusBar().showMessage)
        worker.finished.connect(self._on_load_ready)
        worker.error.connect(self._on_load_error)
        self._load_worker = worker
        worker.start()

    def _on_load_ready(self, model) -> None:
        if self._load_worker:
            self._load_worker.deleteLater()
            self._load_worker = None
        self._model = model
        if self._viewer is None:
            return  # viewer still initialising; _create_viewer will call _apply_loaded_model
        self._apply_loaded_model()

    def _apply_loaded_model(self) -> None:
        """Apply self._model to the viewer and ribbon. Both must be ready."""
        model = self._model
        n_faces = len(model.data.trimesh_mesh.faces)
        self._viewer.load_mesh(model.data)
        self._viewer.enable_bbox_clicking(self._on_bbox_region_clicked)

        self._ribbon.set_model_loaded(True)
        self._ribbon.set_path_exists(False)
        self._ribbon.set_select_mode(False)
        self._viewer.set_select_mode(False)
        self._ribbon.update_mesh_stats(n_faces)

        self._ribbon.clear_stats()
        self._viewer.show_stats_text([
            'MESH',
            f'Triangles  {n_faces:,}',
        ])

        b = model.data.pyvista_mesh.bounds
        max_extent = max(b[1]-b[0], b[3]-b[2], b[5]-b[4])
        detected = _detect_unit(max_extent)
        self._ribbon.set_unit(detected)

        self.statusBar().showMessage(
            f'Loaded: {os.path.basename(model.data.source_path)}'
            f'  |  {n_faces:,} triangles  |  Unit: {detected} (auto-detected)'
            f'  —  click a bounding box face to select a region.'
        )

    def _on_load_error(self, message: str) -> None:
        if self._load_worker:
            self._load_worker.deleteLater()
            self._load_worker = None
        QMessageBox.critical(self, 'Load error', message)
        self.statusBar().showMessage('Load failed.')

    # ------------------------------------------------------------------
    # Bbox region selection
    # ------------------------------------------------------------------

    def _on_bbox_region_clicked(self, region: str) -> None:
        if self._viewer is None:
            return
        if region in self._selected_regions:
            self._selected_regions.discard(region)
            self._viewer.highlight_bbox_region(region, False)
            self._ribbon.set_region_checked(region, False)
        else:
            self._selected_regions.add(region)
            self._viewer.highlight_bbox_region(region, True)
            self._ribbon.set_region_checked(region, True)
        n = len(self._selected_regions)
        self.statusBar().showMessage(
            f'{n} region(s) selected: {", ".join(sorted(self._selected_regions))}'
            if n else 'Click a bounding box face to select it.'
        )
        self._update_grid()

    def _on_region_shortcut(self, region_id: str, checked: bool) -> None:
        if self._model is None or self._model.data is None:
            return
        if self._viewer is None:
            return
        if checked:
            self._selected_regions.add(region_id)
        else:
            self._selected_regions.discard(region_id)
        self._viewer.highlight_bbox_region(region_id, checked)
        n = len(self._selected_regions)
        self.statusBar().showMessage(
            f'{n} region(s) selected: {", ".join(sorted(self._selected_regions))}'
            if n else 'Click a bounding box face to select it.'
        )
        self._update_grid()

    # ------------------------------------------------------------------
    # Path generation
    # ------------------------------------------------------------------

    def _build_params(self, path_target: str, spray_mm: float) -> GenerationParams:
        """Snapshot all UI settings into a GenerationParams for export metadata."""
        submode = self._ribbon.get_face_grid_submode()
        if path_target == 'bbox':
            mode = 'Boundary Box'
        elif path_target == 'face_grid' and submode == 'shadow':
            mode = 'Face Grid / Adaptive'
        elif path_target == 'face_grid' and submode == 'mesh_standoff':
            mode = 'Face Grid / Conform'
        else:
            mode = 'Mesh Surface'

        up_labels = {0: 'X', 1: 'Y', 2: 'Z'}
        src = ''
        if self._model.data is not None and self._model.data.source_path:
            src = os.path.basename(self._model.data.source_path)

        return GenerationParams(
            source_file         = src,
            path_mode           = mode,
            regions             = sorted(self._selected_regions),
            up_axis             = up_labels.get(self._model.data.up_axis, '?'),
            spray_width_mm      = round(spray_mm, 4),
            standoff_mm         = round(self._ribbon.get_standoff_mm(), 4),
            waypoint_spacing_mm = round(self._ribbon.get_waypoint_spacing_mm(), 4),
            direction           = self._ribbon.get_direction(),
            sweep               = 'CCW' if self._ribbon.is_direction_flipped() else 'CW',
            paint_speed_mmpm    = _SPEED_CURVE['max_mmpm'],   # overridden at OLP export time
        )

    def _on_generate(self) -> None:
        if self._model is None or not self._model.is_loaded:
            return
        if self._worker:  # non-None = in flight (running OR results pending delivery)
            return
        spray_mm    = self._ribbon.get_spray_width_mm()
        path_target = self._ribbon.get_path_target()
        self._last_params = self._build_params(path_target, spray_mm)
        if path_target == 'bbox':
            self._generate_bbox(spray_mm)
        elif path_target == 'face_grid':
            self._generate_face_grid(spray_mm)
        else:
            self._generate_mesh(spray_mm)

    def _generate_bbox(self, spray_mm: float) -> None:
        if not self._selected_regions:
            QMessageBox.warning(self, 'No selection',
                'Click a bounding box face to select it, then generate.')
            return
        from app.path import bbox_generator as _bbox_gen
        bounds    = self._model.data.pyvista_mesh.bounds
        up        = self._model.data.up_axis
        direction = self._ribbon.get_direction()
        v_mm      = self._ribbon.get_v_width_mm() or spray_mm
        offset    = 1 if self._ribbon.is_direction_flipped() else 0
        wpt_mm    = self._ribbon.get_waypoint_spacing_mm()
        routes: list[PaintRoute] = []
        for region in sorted(self._selected_regions):
            try:
                standoff = self._ribbon.get_standoff_mm()
                if direction in ('horizontal', 'both'):
                    routes.append(_bbox_gen.generate_bbox_route(
                        region, bounds, spray_mm, up,
                        direction='horizontal', direction_offset=offset,
                        waypoint_spacing_mm=wpt_mm, standoff_mm=standoff))
                if direction in ('vertical', 'both'):
                    routes.append(_bbox_gen.generate_bbox_route(
                        region, bounds, v_mm, up,
                        direction='vertical', direction_offset=offset,
                        waypoint_spacing_mm=wpt_mm, standoff_mm=standoff))
            except Exception as exc:
                QMessageBox.critical(self, 'Generation error',
                    f'{type(exc).__name__}: {exc}\n{traceback.format_exc()}')
                return
        self._face_grid_planes_cache = None
        self._adaptive_grid_cache    = None
        self._viewer.clear_face_grid_planes()
        self._viewer.show_bbox(True)
        self._on_route_ready(routes)

    def _generate_face_grid(self, spray_mm: float) -> None:
        """Face Grid: shadow projection or mesh-surface standoff."""
        if not self._selected_regions:
            QMessageBox.warning(self, 'No selection',
                'Select a face region first, then generate.')
            return
        if self._ribbon.get_face_grid_submode() == 'mesh_standoff':
            self._generate_face_grid_mesh(spray_mm)
        else:
            self._generate_face_grid_flat(spray_mm)

    def _generate_face_grid_flat(self, spray_mm: float) -> None:
        """Depth-Adaptive: H×V intersection-grid passes (background worker)."""
        from app.path import face_grid_generator as _fg_gen
        data     = self._model.data
        mesh     = data.trimesh_mesh
        up       = data.up_axis
        offset   = 1 if self._ribbon.is_direction_flipped() else 0
        wpt_mm   = self._ribbon.get_waypoint_spacing_mm()
        standoff = self._ribbon.get_standoff_mm()
        direction = self._ribbon.get_direction()

        pairs = _group_for_face_grid(self._selected_regions, self._model)
        if not pairs:
            QMessageBox.warning(self, 'No faces', 'Selected regions have no classified faces.')
            return

        class _AdaptiveWorker(QThread):
            finished = Signal(object)   # list of (route, grid_pts, ref_corners)
            error    = Signal(str)
            def __init__(self, pairs, mesh, up, spray_mm, offset, wpt_mm, standoff, direction):
                super().__init__()
                self._pairs, self._mesh = pairs, mesh
                self._up, self._spray = up, spray_mm
                self._offset, self._wpt = offset, wpt_mm
                self._standoff, self._direction = standoff, direction
            def run(self):
                try:
                    results = []
                    for region, faces in self._pairs:
                        route, grid_pts = _fg_gen.generate_adaptive_grid_route(
                            region, faces, self._mesh, self._up,
                            spray_width_mm=self._spray,
                            direction_offset=self._offset,
                            waypoint_spacing_mm=self._wpt,
                            standoff_mm=self._standoff,
                            direction=self._direction,
                        )
                        rc = _fg_gen.get_face_grid_plane_corners(
                            region, faces, self._mesh, self._up, standoff_mm=0.0,
                        )
                        results.append((route, grid_pts, rc))
                    self.finished.emit(results)
                except Exception as exc:
                    import traceback as _tb
                    self.error.emit(f'{type(exc).__name__}: {exc}\n{_tb.format_exc()}')

        def _on_adaptive_done(results):
            self._viewer.show_bbox(False)
            routes   = [r for r, _, _ in results]
            adaptive = [(rc, gp) for _, gp, rc in results]
            self._on_route_ready(routes)
            self._face_grid_planes_cache = None
            self._adaptive_grid_cache    = (adaptive, spray_mm)
            show_grid = self._ribbon.is_show_grid()
            for i, (rc, gp) in enumerate(adaptive):
                self._viewer.show_adaptive_grid(
                    rc, gp,
                    show_grid=show_grid,
                    clear=(i == 0),
                    suffix=f'_{i}' if i > 0 else '',
                )
            self._ribbon.set_generating(False)

        worker = _AdaptiveWorker(pairs, mesh, up, spray_mm, offset, wpt_mm, standoff, direction)
        worker.finished.connect(_on_adaptive_done)
        worker.error.connect(self._on_route_error)
        self._viewer.show_bbox(False)
        self._ribbon.set_generating(True)
        self.statusBar().showMessage('Generating adaptive paths…')
        self._worker = worker
        worker.start()

    def _generate_face_grid_mesh(self, spray_mm: float) -> None:
        """Conform: tilted-basis cutting planes + trimesh intersection + uniform standoff."""
        from app.path import face_grid_generator as _fg_gen
        data     = self._model.data
        mesh     = data.trimesh_mesh
        up       = data.up_axis
        standoff = self._ribbon.get_standoff_mm()
        offset   = 1 if self._ribbon.is_direction_flipped() else 0
        wpt_mm   = self._ribbon.get_waypoint_spacing_mm()

        pairs = _group_for_face_grid(self._selected_regions, self._model)
        if not pairs:
            QMessageBox.warning(self, 'No faces', 'Selected regions have no classified faces.')
            return

        # Build ref/spray plane corners for each selected region
        plane_pairs = []
        for i, (region, faces) in enumerate(pairs):
            rc = _fg_gen.get_face_grid_plane_corners(
                region, faces, mesh, up,
                standoff_mm=0.0,
            ) if i == 0 else None
            sc = _fg_gen.get_face_grid_plane_corners(
                region, faces, mesh, up,
                standoff_mm=standoff,
            )
            plane_pairs.append((rc, sc))

        self._face_grid_planes_cache = (plane_pairs, spray_mm * 0.85)
        self._adaptive_grid_cache    = None
        show_grid = self._ribbon.is_show_grid()
        for i, (rc, sc) in enumerate(plane_pairs):
            self._viewer.show_face_grid_planes(
                rc, sc,
                step_spacing=spray_mm,
                show_grid=show_grid,
                clear=(i == 0),
                suffix=f'_{i}' if i > 0 else '',
            )
        self._viewer.show_bbox(False)

        self._ribbon.set_generating(True)
        self.statusBar().showMessage('Generating conform paths…')

        class _ConformWorker(QThread):
            finished = Signal(object)
            error    = Signal(str)
            def __init__(self, pairs, mesh, up, spray_mm, standoff_mm, offset, wpt_mm, direction):
                super().__init__()
                self._pairs, self._mesh = pairs, mesh
                self._up, self._spray = up, spray_mm
                self._standoff, self._offset, self._wpt = standoff_mm, offset, wpt_mm
                self._direction = direction
            def run(self):
                try:
                    routes = []
                    for region, faces in self._pairs:
                        r = _fg_gen.generate_conform_route(
                            region, faces, self._mesh, self._up,
                            spray_width_mm=self._spray,
                            direction_offset=self._offset,
                            waypoint_spacing_mm=self._wpt,
                            standoff_mm=self._standoff,
                            direction=self._direction,
                        )
                        routes.append(r)
                    self.finished.emit(routes)
                except Exception as exc:
                    import traceback as _tb
                    self.error.emit(f'{type(exc).__name__}: {exc}\n{_tb.format_exc()}')

        worker = _ConformWorker(pairs, mesh, up, spray_mm, standoff, offset, wpt_mm,
                                self._ribbon.get_direction())
        worker.finished.connect(self._on_route_ready)
        worker.error.connect(self._on_route_error)
        self._worker = worker
        worker.start()

    def _generate_mesh(self, spray_mm: float) -> None:
        pairs = _group_for_mesh(self._selected_regions, self._model)
        if not pairs:
            QMessageBox.warning(self, 'No selection',
                'Select bounding box regions first.')
            return
        wpt_mm = self._ribbon.get_waypoint_spacing_mm()
        self._ribbon.set_generating(True)
        self.statusBar().showMessage('Generating mesh paths...')
        worker = _PathWorker(self._model.data, pairs, spray_mm,
                             waypoint_spacing_mm=wpt_mm,
                             standoff_mm=self._ribbon.get_standoff_mm(),
                             direction=self._ribbon.get_direction())
        self._face_grid_planes_cache = None
        self._adaptive_grid_cache    = None
        self._viewer.clear_face_grid_planes()
        self._viewer.show_bbox(True)
        worker.finished.connect(self._on_route_ready)
        worker.error.connect(self._on_route_error)
        self._worker = worker
        worker.start()

    def _on_route_ready(self, routes: list[PaintRoute]) -> None:
        self._ribbon.set_generating(False)
        if self._worker:
            try:
                self._worker.finished.disconnect()
                self._worker.error.disconnect()
            except RuntimeError:
                pass
            self._worker.deleteLater()
            self._worker = None
        if len(routes) > 1:
            from app.path import bbox_generator as _bbox_gen
            routes = [_bbox_gen.merge_routes(routes)]
        self._current_routes  = routes
        from app.path.collision import detect_overlaps
        self._collision_ids   = detect_overlaps(routes)

        # Show paths immediately — collision highlights added after background check
        self._viewer.show_route(
            routes,
            show_arrows=self._ribbon.is_show_arrows(),
            show_waypoints=self._ribbon.is_show_waypoints(),
            collision_ids=self._collision_ids,
        )
        self._ribbon.update_route_stats(routes, self._ribbon.current_unit)
        self._ribbon.set_path_exists(bool(routes))
        from app.path.path_model import UNIT_TO_MM
        unit    = self._ribbon.current_unit
        factor  = UNIT_TO_MM.get(unit, 1.0)
        total_passes = sum(r.total_passes for r in routes)
        total_conns  = sum(len(r.connections) for r in routes)
        total_mm     = sum(r.total_length_mm for r in routes)
        spacing_mm   = routes[0].spacing_mm if routes else 0.0
        n_faces = len(self._model.data.trimesh_mesh.faces)
        self._viewer.show_stats_text([
            'MESH',
            f'Triangles  {n_faces:,}',
            '',
            'PATH',
            f'Passes       {total_passes}',
            f'Connections  {total_conns}',
            f'Length       {total_mm / factor:.2f} {unit}',
            f'Spacing      {spacing_mm / factor:.2f} {unit}',
        ])
        empty_regions = [r.region_id for r in routes if r.total_passes == 0]
        if empty_regions:
            QMessageBox.warning(
                self, 'No passes generated',
                f"The following regions produced 0 passes:\n  {', '.join(empty_regions)}\n\n"
                "Try reducing the spray width or check that the correct up-axis was selected.",
            )
        is_face_grid = (self._last_params is not None and
                        'Face Grid' in self._last_params.path_mode)
        self.statusBar().showMessage(
            f'Path generation complete  —  {total_passes} passes, {total_conns} connections.'
            + ('' if is_face_grid else '  Checking collisions…'))
        self._update_grid()

        # Spawn background collision check (skip for face grid — standoff is built-in)
        if self._model and self._model.data and not is_face_grid:
            if self._coll_worker:
                try:
                    self._coll_worker.finished.disconnect()
                except RuntimeError:
                    pass
                self._zombie_workers.append(self._coll_worker)  # keep alive until thread finishes
                self._coll_worker = None
            self._coll_worker = _CollisionWorker(
                routes, self._model.data.trimesh_mesh, self._ribbon.get_standoff_mm())
            self._coll_worker.finished.connect(self._on_collision_ready)
            self._coll_worker.error.connect(self._on_collision_error)
            self._coll_worker.start()

    def _on_collision_ready(self, result) -> None:
        self._zombie_workers = [w for w in self._zombie_workers if w.isRunning()]
        collision_ids, suggested = result
        if self._coll_worker:
            self._coll_worker.deleteLater()
            self._coll_worker = None
        # Merge: collision/near_miss take priority; keep existing overlap flags
        self._collision_ids = {**self._collision_ids, **collision_ids}
        if self._current_routes and self._collision_ids:
            self._viewer.show_route(
                self._current_routes,
                show_arrows=self._ribbon.is_show_arrows(),
                show_waypoints=self._ribbon.is_show_waypoints(),
                collision_ids=self._collision_ids,
            )
        n_coll = sum(1 for v in collision_ids.values() if v == 'collision')
        n_near = sum(1 for v in collision_ids.values() if v == 'near_miss')
        total_passes = sum(r.total_passes for r in self._current_routes)
        total_conns  = sum(len(r.connections) for r in self._current_routes)
        coll_suffix = ''
        if n_coll or n_near:
            coll_suffix = (f'  ⚠ {n_coll} collision(s), {n_near} near-miss(es)'
                           f' — shown red/orange  |  Suggested standoff: {suggested:.0f} mm')
        self.statusBar().showMessage(
            f'Path generation complete  —  {total_passes} passes, {total_conns} connections.{coll_suffix}')

    def _on_collision_error(self, msg: str) -> None:
        self._zombie_workers = [w for w in self._zombie_workers if w.isRunning()]
        if self._coll_worker:
            self._coll_worker.deleteLater()
            self._coll_worker = None
        total_passes = sum(r.total_passes for r in self._current_routes) if self._current_routes else 0
        total_conns  = sum(len(r.connections) for r in self._current_routes) if self._current_routes else 0
        self.statusBar().showMessage(
            f'Path generation complete  —  {total_passes} passes, {total_conns} connections.'
            '  ⚠ Collision check failed (see details).')
        QMessageBox.warning(self, 'Collision check failed',
            'The collision check could not complete — the path has been generated '
            'but collisions have not been verified.\n\n'
            f'Reason: {msg.splitlines()[0]}')

    def _refresh_route_display(self) -> None:
        if self._viewer is None or not self._current_routes:
            return
        self._viewer.show_route(
            self._current_routes,
            show_arrows=self._ribbon.is_show_arrows(),
            show_waypoints=self._ribbon.is_show_waypoints(),
            collision_ids=self._collision_ids,
        )

    def _on_route_error(self, message: str) -> None:
        self._ribbon.set_generating(False)
        if self._worker:
            try:
                self._worker.finished.disconnect()
                self._worker.error.disconnect()
            except RuntimeError:
                pass
            self._worker.deleteLater()
            self._worker = None
        if self._coll_worker:
            try:
                self._coll_worker.finished.disconnect()
            except RuntimeError:
                pass
            self._zombie_workers.append(self._coll_worker)
            self._coll_worker = None
        QMessageBox.critical(self, 'Generation error', message)
        self.statusBar().showMessage('Generation failed.')

    def _clear_paths(self) -> None:
        self._face_grid_planes_cache = None
        self._adaptive_grid_cache    = None
        if self._coll_worker:
            try:
                self._coll_worker.finished.disconnect()
            except RuntimeError:
                pass
            self._zombie_workers.append(self._coll_worker)
            self._coll_worker = None
        if self._viewer is None:
            return
        self._viewer.clear_route()
        self._viewer.clear_face_grid_planes()
        self._viewer.clear_bbox_grid()
        self._viewer.show_bbox(True)        # restore bbox cage if hidden by Face Grid
        self._current_routes = []
        self._ribbon.clear_stats()
        self._viewer.clear_stats_text()
        self._ribbon.set_path_exists(False)
        self.statusBar().showMessage('Paths cleared.')

    def _update_grid(self) -> None:
        if self._viewer is None:
            return
        # Adaptive mode: redraw curved intersection grid from cache
        if self._adaptive_grid_cache is not None:
            adaptive_list, _ = self._adaptive_grid_cache
            show_grid = self._ribbon.is_show_grid()
            for i, (rc, gp) in enumerate(adaptive_list):
                self._viewer.show_adaptive_grid(
                    rc, gp,
                    show_grid=show_grid,
                    clear=(i == 0),
                    suffix=f'_{i}' if i > 0 else '',
                )
            return
        # Flat/conform mode: redraw planes/grid from cache
        if self._face_grid_planes_cache is not None:
            plane_pairs, spc = self._face_grid_planes_cache
            show_grid = self._ribbon.is_show_grid()
            for i, (rc, sc) in enumerate(plane_pairs):
                self._viewer.show_face_grid_planes(
                    rc, sc,
                    step_spacing=spc,
                    show_grid=show_grid,
                    clear=(i == 0),
                    suffix=f'_{i}' if i > 0 else '',
                )
            return

        # Bbox / mesh mode: draw grid on the bounding-box face
        self._viewer.clear_bbox_grid()
        if not self._ribbon.is_show_grid():
            return
        if self._model is None or self._model.data is None:
            return
        bounds = tuple(self._model.data.pyvista_mesh.bounds)
        h_mm = self._ribbon.get_spray_width_mm() * 0.85  # match bbox_generator step_spacing
        v_mm = (self._ribbon.get_v_width_mm() or self._ribbon.get_spray_width_mm()) * 0.85
        up   = self._model.data.up_axis
        for region in self._selected_regions:
            self._viewer.show_bbox_grid(region, bounds, up, h_mm, v_mm)

    # ------------------------------------------------------------------
    # Help / About
    # ------------------------------------------------------------------

    def _show_help(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle('How to Use SmartGrid')
        dlg.setMinimumSize(560, 480)
        vl = QVBoxLayout(dlg)
        vl.setContentsMargins(16, 16, 16, 12)

        tb = QTextBrowser()
        tb.setOpenExternalLinks(True)
        tb.setStyleSheet('font-family:"Segoe UI",Arial;font-size:12px;')
        tb.setHtml('''
<h2 style="margin-top:0">SmartGrid — Quick Start</h2>

<h3>1 · Open a mesh</h3>
<p>Drag an <b>STL / OBJ / STEP</b> file onto the window, or use
<b>File › Open STL / OBJ / STEP…</b> (Ctrl+O).<br>
When prompted, pick the axis that points <em>up</em> in your file (Z is the
most common default).</p>

<h3>2 · Select a region</h3>
<p>After loading, a transparent bounding-box cage appears around the mesh.
<b>Click any face of the cage</b> to select that surface region
(Top, Bottom, Front, Rear, Left, Right). You can also toggle regions using the
checkboxes in the ribbon. Multiple regions may be selected simultaneously.</p>

<h3>3 · Choose a path mode</h3>
<ul>
  <li><b>Boundary Box</b> – flat parallel passes projected from the bounding box.
      Fast, useful for roughly flat surfaces.</li>
  <li><b>Adaptive</b> – a single H×V grid fitted to the actual surface geometry.
      Best for curved or multi-face selections.</li>
  <li><b>Conform</b> – tilted cutting planes follow the surface normal; paths
      hug the mesh. Use with a standoff distance.</li>
  <li><b>Mesh Surface</b> – axis-aligned slices through the mesh. Good for
      organic shapes with complex topology.</li>
</ul>

<h3>4 · Tune parameters</h3>
<p>Adjust <b>Spray Width</b>, <b>Standoff</b>, <b>Waypoint Spacing</b>,
<b>Direction</b> (H / V / Both), and <b>Sweep</b> direction in the ribbon.
Press <b>Generate</b> (Ctrl+G / F5) to compute the toolpath.</p>

<h3>5 · Inspect the result</h3>
<p><b>Blue</b> passes = forward; <b>Green</b> passes = reverse;
<b>Red</b> = collision with mesh; <b>Orange</b> = near-miss.<br>
Toggle <b>Show Grid</b>, <b>Arrows</b>, and <b>Waypoints</b> in the ribbon for
additional visual feedback.</p>

<h3>6 · Export</h3>
<p>Use the <b>Export</b> menu (or ribbon buttons) to save the toolpath as
<b>JSON</b>, <b>CSV</b>, or one of the supported OLP formats
(RoboDK, Visual Components, DELMIA APT, G-code).</p>

<hr>
<p style="color:#555;font-size:11px">
For updates and full documentation visit the
<a href="https://github.com/sakrish205/SmartGrid">GitHub repository</a>.
</p>
''')
        vl.addWidget(tb)

        btns = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        btns.rejected.connect(dlg.reject)
        btns.accepted.connect(dlg.accept)
        # Close button maps to rejected; wire accepted too for safety
        btns.button(QDialogButtonBox.StandardButton.Close).clicked.connect(dlg.accept)
        vl.addWidget(btns)
        dlg.exec()

    def _show_about(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle('About SmartGrid')
        dlg.setFixedWidth(400)
        vl = QVBoxLayout(dlg)
        vl.setContentsMargins(20, 20, 20, 16)
        vl.setSpacing(8)

        tb = QTextBrowser()
        tb.setOpenExternalLinks(True)
        tb.setFrameShape(QFrame.Shape.NoFrame)
        tb.setStyleSheet('font-family:"Segoe UI",Arial;font-size:12px;background:transparent;')
        tb.setHtml('''
<h2 style="margin:0 0 4px 0">SmartGrid  <span style="font-weight:normal;font-size:13px;color:#555">v1.5</span></h2>
<p style="margin:0 0 12px 0;color:#555">3D Surface Grid &amp; Pitch Mapper</p>
<p>Robotic spray-paint toolpath planner for STL, OBJ and STEP meshes.<br>
Supports Boundary Box, Adaptive, Conform and Mesh Surface path modes
with JSON / CSV / OLP export.</p>
<p><a href="https://github.com/sakrish205/SmartGrid">github.com/sakrish205/SmartGrid</a></p>
''')
        tb.setMaximumHeight(160)
        vl.addWidget(tb)

        btns = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        btns.button(QDialogButtonBox.StandardButton.Close).clicked.connect(dlg.accept)
        vl.addWidget(btns)
        dlg.exec()

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------

    def _export_json(self) -> None:
        if not self._current_routes:
            QMessageBox.warning(self, 'Nothing to export', 'Generate a path first.')
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Export JSON', '', 'JSON (*.json)')
        if not path:
            return
        if not path.lower().endswith('.json'):
            path += '.json'
        self.statusBar().showMessage('Exporting toolpath...')
        try:
            export_route_json(
                self._current_routes, path,
                params=self._last_params,
            )
            self.statusBar().showMessage(f'Exported: {path}')
        except Exception as exc:
                QMessageBox.critical(self, 'Export error', str(exc))
                self.statusBar().showMessage('Export failed.')

    def _export_csv(self) -> None:
        if not self._current_routes:
            QMessageBox.warning(self, 'Nothing to export', 'Generate a path first.')
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Export CSV', '', 'CSV (*.csv)')
        if not path:
            return
        if not path.lower().endswith('.csv'):
            path += '.csv'
        self.statusBar().showMessage('Exporting toolpath...')
        try:
            export_route_csv(
                self._current_routes, path,
                params=self._last_params,
            )
            self.statusBar().showMessage(f'Exported: {path}')
        except Exception as exc:
            QMessageBox.critical(self, 'Export error', str(exc))
            self.statusBar().showMessage('Export failed.')

    _OLP_FORMATS = {
        'RoboDK (6-col curve CSV)':  ('robodk', 'CSV (*.csv)',        '.csv'),
        'Visual Components (CSV)':   ('vc',     'CSV (*.csv)',        '.csv'),
        'DELMIA (APT text)':         ('delmia', 'APT (*.apt)',        '.apt'),
        'G-code (CNC / Robot)':      ('gcode',  'G-code (*.nc)',      '.nc'),
    }

    def _export_olp(self) -> None:
        if not self._current_routes:
            QMessageBox.warning(self, 'Nothing to export', 'Generate a path first.')
            return

        dlg = _OlpExportDialog(self._OLP_FORMATS, parent=self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return

        fmt_label, speed_mode, custom_speed = dlg.values()
        fmt_key, file_filter, ext = self._OLP_FORMATS[fmt_label]

        path, _ = QFileDialog.getSaveFileName(self, f'Export {fmt_label}', '', file_filter)
        if not path:
            return
        if not path.lower().endswith(ext):
            path += ext

        # Build per-pass speeds map (auto) or None (custom → fixed)
        if speed_mode == 'auto':
            speeds_map: dict[int, np.ndarray] = {
                p.id: _compute_dynamic_speeds(p.points)
                for route in self._current_routes
                for p in route.passes
            }
        else:
            speeds_map = None

        # Params with correct fixed speed for metadata / custom mode
        params = self._last_params
        if params is not None:
            fixed = custom_speed if speed_mode == 'custom' else _SPEED_CURVE['max_mmpm']
            params = _dc_replace(params, paint_speed_mmpm=fixed)

        self.statusBar().showMessage('Exporting OLP...')
        try:
            fn = {'robodk': export_robodk, 'vc': export_vc, 'delmia': export_delmia_apt,
                  'gcode': export_gcode}[fmt_key]
            fn(self._current_routes, path, params=params, speeds_map=speeds_map)
            self.statusBar().showMessage(f'Exported: {path}')
        except Exception as exc:
            QMessageBox.critical(self, 'Export error', str(exc))
            self.statusBar().showMessage('Export failed.')
