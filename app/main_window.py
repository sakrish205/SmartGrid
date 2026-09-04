"""Top-level window — ribbon layout, no sidebar."""
from __future__ import annotations
import os
import traceback
from typing import Optional

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout,
    QFileDialog, QMessageBox, QDialog,
    QDialogButtonBox, QRadioButton, QButtonGroup,
    QLabel, QVBoxLayout as QVBox,
)
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QAction, QDragEnterEvent, QDropEvent, QFont

from models.mesh_model import MeshModel
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintRoute
from app.path import generator as _generator
from app.path import bbox_generator as _bbox_generator
from app.ui.viewer import MeshViewer
from app.ui.ribbon import SmartRibbon
from app.export.json_export import export_route_json
from app.export.csv_export import export_route_csv
from app.ui.view_settings_dialog import ViewSettingsDialog, DEFAULTS as _COLOR_DEFAULTS


def _detect_unit(max_extent: float) -> str:
    if max_extent > 100:
        return 'mm'
    if max_extent > 1:
        return 'cm'
    return 'm'


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
        mesh_data: MeshData,
        pairs: list,
        spray_mm: float,
        waypoint_spacing_mm: float = 0.0,
    ) -> None:
        super().__init__()
        self._mesh_data          = mesh_data
        self._pairs              = pairs
        self._spray_mm           = spray_mm
        self._waypoint_spacing   = waypoint_spacing_mm

    def run(self) -> None:
        try:
            routes = []
            for region_id, face_indices in self._pairs:
                routes.append(_generator.generate_route(
                    self._mesh_data,
                    region_id=region_id,
                    region_face_indices=face_indices,
                    spray_width_mm=self._spray_mm,
                    waypoint_spacing_mm=self._waypoint_spacing,
                ))
            self.finished.emit(routes)
        except Exception as exc:
            self.error.emit(f'{type(exc).__name__}: {exc}\n{traceback.format_exc()}')


# ---------------------------------------------------------------------------
# Up-axis dialog
# ---------------------------------------------------------------------------

class _UpAxisDialog(QDialog):
    def __init__(self, filename: str, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle('Open Mesh')
        layout = QVBox(self)
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
        self.resize(1400, 860)
        self.setAcceptDrops(True)
        self.setStyleSheet(
            'QMainWindow{background:#e8e8e8;}'
            'QStatusBar{background:#f0f0f0;color:#333;font-size:11px;'
            '  font-family:"Segoe UI",Arial;border-top:1px solid #b0b0b0;'
            '  padding:0 4px;}'
            'QStatusBar::item{border:none;}'
        )
        self.menuBar().setStyleSheet(self._MENUBAR_STYLE)

        self._model              = MeshModel()
        self._selected_regions:  set[str]         = set()
        self._current_routes:    list[PaintRoute] = []
        self._worker:        Optional[QThread] = None
        self._load_worker:   Optional[QThread] = None
        self._current_colors: dict[str, str]  = dict(_COLOR_DEFAULTS)

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

        # Ribbon
        self._ribbon = SmartRibbon()
        vl.addWidget(self._ribbon)

        # 3-D viewport — takes all remaining space
        self._viewer = MeshViewer()
        vl.addWidget(self._viewer, stretch=1)

        # Initial state
        self._ribbon.set_model_loaded(False)
        self.statusBar().showMessage('Ready — open an STL or OBJ file to begin.')

        # Wire ribbon signals → window handlers
        self._ribbon.open_requested.connect(self._open_file)
        self._ribbon.view_fit.connect(self._viewer.fit_all)
        self._ribbon.view_set.connect(self._on_view_set)
        self._ribbon.grid_changed.connect(self._update_grid)
        self._ribbon.arrows_changed.connect(self._refresh_route_display)
        self._ribbon.region_toggled.connect(self._on_region_shortcut)
        self._ribbon.select_mode_changed.connect(self._on_select_mode_changed)
        self._ribbon.generate_requested.connect(self._on_generate)
        self._ribbon.clear_requested.connect(self._clear_paths)
        self._ribbon.sweep_changed.connect(self._on_sweep_changed)
        self._ribbon.waypoints_changed.connect(self._refresh_route_display)
        self._ribbon.export_json.connect(self._export_json)
        self._ribbon.export_csv.connect(self._export_csv)
        self._ribbon.view_settings_req.connect(self._open_view_settings)

    def _build_menus(self) -> None:
        mb = self.menuBar()

        file_menu = mb.addMenu('File')
        open_act = QAction('Open STL / OBJ...', self)
        open_act.setShortcut('Ctrl+O')
        open_act.triggered.connect(self._open_file)
        file_menu.addAction(open_act)
        file_menu.addSeparator()
        file_menu.addAction('Exit', self.close)

        view_menu = mb.addMenu('View')
        view_menu.addAction('Fit All\tCtrl+Home', lambda: self._viewer.fit_all())
        view_menu.addAction('Top View',    lambda: self._on_view_set('top'))
        view_menu.addAction('Bottom View', lambda: self._on_view_set('bottom'))
        view_menu.addAction('Front View',  lambda: self._on_view_set('front'))
        view_menu.addAction('Rear View',   lambda: self._on_view_set('rear'))
        view_menu.addAction('Left View',   lambda: self._on_view_set('left'))
        view_menu.addAction('Right View',  lambda: self._on_view_set('right'))
        view_menu.addSeparator()
        view_menu.addAction('Rotate Left  90',  lambda: self._viewer.roll_view(-90))
        view_menu.addAction('Rotate Right 90',  lambda: self._viewer.roll_view(+90))
        view_menu.addSeparator()
        view_menu.addAction('View Settings...', self._open_view_settings)

        path_menu = mb.addMenu('Path')
        path_menu.addAction('Generate\tCtrl+G', self._on_generate)
        path_menu.addAction('Flip Direction',   self._flip_direction)
        path_menu.addSeparator()
        path_menu.addAction('Clear Paths', self._clear_paths)

        export_menu = mb.addMenu('Export')
        export_menu.addAction('Export JSON...', self._export_json)
        export_menu.addAction('Export CSV...',  self._export_csv)

    # ------------------------------------------------------------------
    # View helpers
    # ------------------------------------------------------------------

    def _on_view_set(self, direction: str) -> None:
        self._viewer.set_view(direction)

    # ------------------------------------------------------------------
    # Select mode
    # ------------------------------------------------------------------

    def _on_select_mode_changed(self, active: bool) -> None:
        self._viewer.set_select_mode(active)
        if active:
            self.statusBar().showMessage(
                'Select Faces — click a bounding box face. Camera rotation suspended.')
        else:
            self.statusBar().showMessage('Navigate — drag to rotate, scroll to zoom.')

    # ------------------------------------------------------------------
    # View settings
    # ------------------------------------------------------------------

    def _open_view_settings(self) -> None:
        dlg = ViewSettingsDialog(self._current_colors, self)
        dlg.colors_changed.connect(self._on_colors_changed)
        if dlg.exec():
            self._current_colors = dlg.colors
        else:
            self._viewer.apply_colors(self._current_colors)

    def _on_colors_changed(self, colors: dict[str, str]) -> None:
        self._viewer.apply_colors(colors)
        if self._current_routes:
            self._viewer.show_route(
                self._current_routes,
                show_arrows=self._ribbon.is_show_arrows(),
                show_waypoints=self._ribbon.is_show_waypoints(),
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
            'Mesh Files (*.stl *.obj);;All Files (*)')
        if path:
            self._load(path)

    def dragEnterEvent(self, event: QDragEnterEvent) -> None:
        if event.mimeData().hasUrls():
            if any(u.toLocalFile().lower().endswith(('.stl', '.obj'))
                   for u in event.mimeData().urls()):
                event.acceptProposedAction()

    def dropEvent(self, event: QDropEvent) -> None:
        for url in event.mimeData().urls():
            path = url.toLocalFile()
            if path.lower().endswith(('.stl', '.obj')):
                self._load(path)
                break

    def _load(self, filepath: str) -> None:
        if self._load_worker and self._load_worker.isRunning():
            return
        dlg = _UpAxisDialog(filepath, self)
        dlg.exec()
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
        n_faces = len(model.data.trimesh_mesh.faces)

        self._viewer.load_mesh(model.data)
        self._viewer.enable_bbox_clicking(self._on_bbox_region_clicked)

        self._ribbon.set_model_loaded(True)
        self._ribbon.set_path_exists(False)
        self._ribbon.set_select_mode(False)
        self._viewer.set_select_mode(False)
        self._ribbon.update_mesh_stats(n_faces)
        self._ribbon.clear_stats()

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
        if self._model.data is None:
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

    def _on_generate(self) -> None:
        if not self._model.is_loaded:
            return
        if self._worker and self._worker.isRunning():
            return
        spray_mm    = self._ribbon.get_spray_width_mm()
        path_target = self._ribbon.get_path_target()
        if path_target == 'bbox':
            self._generate_bbox(spray_mm)
        else:
            self._generate_mesh(spray_mm)

    def _generate_bbox(self, spray_mm: float) -> None:
        if not self._selected_regions:
            QMessageBox.warning(self, 'No selection',
                'Click a bounding box face to select it, then generate.')
            return
        bounds    = self._model.data.pyvista_mesh.bounds
        up        = self._model.data.up_axis
        direction = self._ribbon.get_direction()
        v_mm      = self._ribbon.get_v_width_mm() or spray_mm
        offset    = 1 if self._ribbon.is_direction_flipped() else 0
        wpt_mm    = self._ribbon.get_waypoint_spacing_mm()
        routes: list[PaintRoute] = []
        for region in sorted(self._selected_regions):
            try:
                if direction in ('horizontal', 'both'):
                    routes.append(_bbox_generator.generate_bbox_route(
                        region, bounds, spray_mm, up,
                        direction='horizontal', direction_offset=offset,
                        waypoint_spacing_mm=wpt_mm))
                if direction in ('vertical', 'both'):
                    routes.append(_bbox_generator.generate_bbox_route(
                        region, bounds, v_mm, up,
                        direction='vertical', direction_offset=offset,
                        waypoint_spacing_mm=wpt_mm))
            except Exception as exc:
                QMessageBox.critical(self, 'Generation error',
                    f'{type(exc).__name__}: {exc}\n{traceback.format_exc()}')
                return
        self._on_route_ready(routes)

    def _generate_mesh(self, spray_mm: float) -> None:
        pairs = []
        for region_id in sorted(self._selected_regions):
            faces = self._model.get_region_faces(region_id)
            if len(faces) > 0:
                pairs.append((region_id, faces))
        if not pairs:
            QMessageBox.warning(self, 'No selection',
                'Select bounding box regions first.')
            return
        wpt_mm = self._ribbon.get_waypoint_spacing_mm()
        self._ribbon.set_generating(True)
        self.statusBar().showMessage('Generating mesh paths...')
        worker = _PathWorker(self._model.data, pairs, spray_mm,
                             waypoint_spacing_mm=wpt_mm)
        worker.finished.connect(self._on_route_ready)
        worker.error.connect(self._on_route_error)
        self._worker = worker
        worker.start()

    def _on_route_ready(self, routes: list[PaintRoute]) -> None:
        self._ribbon.set_generating(False)
        if self._worker:
            self._worker.deleteLater()
            self._worker = None
        self._current_routes = routes
        self._viewer.show_route(
            routes,
            show_arrows=self._ribbon.is_show_arrows(),
            show_waypoints=self._ribbon.is_show_waypoints(),
        )
        self._ribbon.update_route_stats(routes, self._ribbon.current_unit)
        self._ribbon.set_path_exists(bool(routes))
        total_passes = sum(r.total_passes for r in routes)
        total_conns  = sum(len(r.connections) for r in routes)
        self.statusBar().showMessage(
            f'Path generation complete  —  {total_passes} passes, {total_conns} connections.')

    def _refresh_route_display(self) -> None:
        if self._current_routes:
            self._viewer.show_route(
                self._current_routes,
                show_arrows=self._ribbon.is_show_arrows(),
                show_waypoints=self._ribbon.is_show_waypoints(),
            )

    def _on_route_error(self, message: str) -> None:
        self._ribbon.set_generating(False)
        if self._worker:
            self._worker.deleteLater()
            self._worker = None
        QMessageBox.critical(self, 'Generation error', message)
        self.statusBar().showMessage('Generation failed.')

    def _clear_paths(self) -> None:
        self._viewer.clear_route()
        self._viewer.clear_bbox_grid()
        self._current_routes = []
        self._ribbon.clear_stats()
        self._ribbon.set_path_exists(False)
        self.statusBar().showMessage('Paths cleared.')

    def _update_grid(self) -> None:
        self._viewer.clear_bbox_grid()
        if not self._ribbon.is_show_grid():
            return
        if self._model.data is None:
            return
        bounds = tuple(self._model.data.pyvista_mesh.bounds)
        h_mm = self._ribbon.get_spray_width_mm()
        v_mm = self._ribbon.get_v_width_mm()
        up   = self._model.data.up_axis
        for region in self._selected_regions:
            self._viewer.show_bbox_grid(region, bounds, up, h_mm, v_mm or h_mm)

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------

    def _export_json(self) -> None:
        if not self._current_routes:
            QMessageBox.warning(self, 'Nothing to export', 'Generate a path first.')
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Export JSON', '', 'JSON (*.json)')
        if path:
            self.statusBar().showMessage('Exporting toolpath...')
            export_route_json(self._current_routes, path)
            self.statusBar().showMessage(f'Exported: {path}')

    def _export_csv(self) -> None:
        if not self._current_routes:
            QMessageBox.warning(self, 'Nothing to export', 'Generate a path first.')
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Export CSV', '', 'CSV (*.csv)')
        if path:
            self.statusBar().showMessage('Exporting toolpath...')
            export_route_csv(self._current_routes, path)
            self.statusBar().showMessage(f'Exported: {path}')
