"""Top-level window — owns mesh model, wires all signals."""
from __future__ import annotations
import os
import traceback
from typing import Optional

import numpy as np
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QFileDialog, QMessageBox, QDialog,
    QDialogButtonBox, QRadioButton, QButtonGroup,
    QLabel, QVBoxLayout as QVBox, QScrollArea,
    QProgressBar, QPushButton,
)
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QAction, QDragEnterEvent, QDropEvent

from models.mesh_model import MeshModel
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintRoute
from app.path import generator as _generator
from app.path import bbox_generator as _bbox_generator
from app.ui.viewer import MeshViewer
from app.ui.surface_panel import SurfacePanel
from app.ui.parameter_panel import ParameterPanel
from app.ui.status_panel import StatusPanel
from app.export.json_export import export_route_json
from app.export.csv_export import export_route_csv

_REGIONS = ['TOP', 'BOTTOM', 'FRONT', 'REAR', 'LEFT', 'RIGHT']

_SIDEBAR_STYLE = """
/* whole left panel */
QWidget { background:#f3f2f1; color:#252525; font-size:13px; }

QGroupBox {
    color:#0078D4;
    font-weight:bold;
    font-size:13px;
    border:1px solid #d2d0ce;
    border-radius:5px;
    margin-top:10px;
    padding-top:8px;
    background:#ffffff;
}
QGroupBox::title {
    subcontrol-origin:margin;
    left:10px;
    padding:0 5px;
    color:#0078D4;
    background:#ffffff;
}

QLabel { color:#323130; font-size:12px; background:transparent; }

QPushButton {
    background:#ffffff;
    color:#252525;
    border:1px solid #d2d0ce;
    border-radius:4px;
    padding:6px 4px;
    font-size:12px;
}
QPushButton:hover    { background:#edebe9; border-color:#b0adab; }
QPushButton:checked  { background:#0078D4; color:#ffffff; border-color:#0078D4; }
QPushButton:disabled { background:#f3f2f1; color:#a19f9d; border-color:#e0dfde; }

QCheckBox { color:#323130; font-size:12px; spacing:8px; background:transparent; }
QCheckBox::indicator {
    width:14px; height:14px;
    border:2px solid #8a8886;
    border-radius:3px;
    background:#ffffff;
}
QCheckBox::indicator:checked         { background:#0078D4; border-color:#0078D4; }
QCheckBox::indicator:unchecked:hover { border-color:#0078D4; }

QRadioButton { color:#323130; font-size:12px; spacing:8px; background:transparent; }
QRadioButton::indicator {
    width:14px; height:14px;
    border:2px solid #8a8886;
    border-radius:7px;
    background:#ffffff;
}
QRadioButton::indicator:checked         { background:#0078D4; border-color:#0078D4; }
QRadioButton::indicator:unchecked:hover { border-color:#0078D4; }

QDoubleSpinBox {
    background:#ffffff;
    color:#252525;
    border:1px solid #d2d0ce;
    border-radius:4px;
    padding:4px 6px;
    font-size:12px;
    min-height:26px;
}
QDoubleSpinBox:focus { border-color:#0078D4; }
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background:#f3f2f1; border:none; width:16px;
}

QComboBox {
    background:#ffffff;
    color:#252525;
    border:1px solid #d2d0ce;
    border-radius:4px;
    padding:4px 6px;
    font-size:12px;
    min-height:26px;
}
QComboBox::drop-down { border:none; width:20px; }
QComboBox QAbstractItemView {
    background:#ffffff; color:#252525;
    selection-background-color:#deecf9;
    selection-color:#0078D4;
}

QScrollArea { border:none; background:#f3f2f1; }
QScrollArea > QWidget > QWidget { background:#f3f2f1; }
QScrollBar:vertical {
    background:#f3f2f1; width:8px; border:none; border-radius:4px;
}
QScrollBar::handle:vertical {
    background:#c8c6c4; border-radius:4px; min-height:20px;
}
QScrollBar::handle:vertical:hover { background:#8a8886; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }

QFrame[frameShape="4"] { color:#d2d0ce; }
"""


# ---------------------------------------------------------------------------
# Loading overlay — covers the full central widget while mesh loads
# ---------------------------------------------------------------------------

class _LoadingOverlay(QWidget):
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.setStyleSheet("background:rgba(243,242,241,220);")

        outer = QVBoxLayout(self)
        outer.setAlignment(Qt.AlignmentFlag.AlignCenter)

        panel = QWidget()
        panel.setFixedWidth(360)
        panel.setStyleSheet(
            "QWidget { background:#ffffff; border:1px solid #d2d0ce;"
            " border-radius:8px; padding:24px; }"
        )
        inner = QVBoxLayout(panel)
        inner.setSpacing(14)
        inner.setContentsMargins(24, 24, 24, 24)

        title = QLabel("SmartGrid")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet(
            "font-size:18px; color:#0078D4; font-weight:bold; border:none;"
        )

        self._label = QLabel("Loading...")
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setStyleSheet(
            "font-size:13px; color:#323130; border:none;"
        )

        self._bar = QProgressBar()
        self._bar.setRange(0, 0)
        self._bar.setFixedHeight(4)
        self._bar.setTextVisible(False)
        self._bar.setStyleSheet(
            "QProgressBar { background:#e0dfde; border:none; border-radius:2px; }"
            "QProgressBar::chunk { background:#0078D4; border-radius:2px; }"
        )

        inner.addWidget(title)
        inner.addWidget(self._label)
        inner.addWidget(self._bar)
        outer.addWidget(panel)
        self.hide()

    def show_message(self, msg: str) -> None:
        self._label.setText(msg)

    def showEvent(self, e) -> None:
        super().showEvent(e)
        self.setGeometry(self.parentWidget().rect())

    def resizeEvent(self, e) -> None:
        super().resizeEvent(e)
        if self.isVisible():
            self.setGeometry(self.parentWidget().rect())


# ---------------------------------------------------------------------------
# Background worker — mesh file loading
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
            self.progress.emit(f"Reading {os.path.basename(self._filepath)}...")
            model = _MM()
            model.load(self._filepath, up_axis=self._up_axis)
            self.progress.emit("Finalising...")
            self.finished.emit(model)
        except Exception as exc:
            self.error.emit(f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}")


# ---------------------------------------------------------------------------
# Background worker — mesh-surface path generation
# ---------------------------------------------------------------------------

class _PathWorker(QThread):
    finished = Signal(object)
    error    = Signal(str)

    def __init__(
        self,
        mesh_data: MeshData,
        region_face_pairs: list,
        spray_mm: float,
    ) -> None:
        super().__init__()
        self._mesh_data = mesh_data
        self._pairs     = region_face_pairs
        self._spray_mm  = spray_mm

    def run(self) -> None:
        try:
            routes = []
            for region_id, face_indices in self._pairs:
                route = _generator.generate_route(
                    self._mesh_data,
                    region_id=region_id,
                    region_face_indices=face_indices,
                    spray_width_mm=self._spray_mm,
                )
                routes.append(route)
            self.finished.emit(routes)
        except Exception as exc:
            self.error.emit(f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}")


# ---------------------------------------------------------------------------
# Up-axis dialog
# ---------------------------------------------------------------------------

class _UpAxisDialog(QDialog):
    def __init__(self, filename: str, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Open Mesh")
        layout = QVBox(self)
        layout.setSpacing(10)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.addWidget(QLabel(f"<b>{os.path.basename(filename)}</b>"))
        layout.addWidget(QLabel("Which axis is UP in this file?"))
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
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SmartGrid  —  3D Surface Grid & Pitch Mapper")
        self.resize(1280, 820)
        self.setAcceptDrops(True)

        self._model              = MeshModel()
        self._selected_regions:  set[str]         = set()
        self._current_routes:    list[PaintRoute] = []
        self._worker:      Optional[QThread] = None
        self._load_worker: Optional[QThread] = None

        self._build_ui()
        self._build_menus()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        # Loading overlay — child of central; covers the whole central area
        self._overlay = _LoadingOverlay(central)

        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Sidebar ────────────────────────────────────────────────────
        sidebar_container = QWidget()
        sidebar_container.setFixedWidth(270)
        sidebar_container.setStyleSheet(_SIDEBAR_STYLE)
        sidebar_outer = QVBoxLayout(sidebar_container)
        sidebar_outer.setContentsMargins(8, 8, 8, 8)
        sidebar_outer.setSpacing(8)

        # Import button — always visible at top of sidebar
        self._import_btn = QPushButton("Open STL / OBJ...")
        self._import_btn.setMinimumHeight(36)
        self._import_btn.setStyleSheet(
            "QPushButton { background:#0078D4; color:#ffffff; border:none;"
            " border-radius:4px; font-size:13px; font-weight:bold; }"
            "QPushButton:hover { background:#106ebe; }"
            "QPushButton:pressed { background:#005a9e; }"
        )
        self._import_btn.clicked.connect(self._open_file)
        sidebar_outer.addWidget(self._import_btn)

        # Hint label — shown before any mesh is loaded
        self._hint_label = QLabel("or drag and drop a file onto the viewer")
        self._hint_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._hint_label.setWordWrap(True)
        self._hint_label.setStyleSheet("color:#605e5c; font-size:11px;")
        sidebar_outer.addWidget(self._hint_label)

        # Scrollable panels
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        inner = QWidget()
        left_layout = QVBoxLayout(inner)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)

        self._surface_panel   = SurfacePanel()
        self._parameter_panel = ParameterPanel()
        self._status_panel    = StatusPanel()

        left_layout.addWidget(self._surface_panel)
        left_layout.addWidget(self._parameter_panel)
        left_layout.addWidget(self._status_panel)
        left_layout.addStretch()

        scroll.setWidget(inner)
        sidebar_outer.addWidget(scroll)
        root.addWidget(sidebar_container)

        self._viewer = MeshViewer()
        root.addWidget(self._viewer, stretch=1)

        self.statusBar().showMessage("Open an STL or OBJ file to begin.")

        self._surface_panel.region_shortcut_requested.connect(self._on_region_shortcut)
        self._parameter_panel.generate_requested.connect(self._on_generate)
        self._parameter_panel.clear_requested.connect(self._clear_paths)
        self._parameter_panel.grid_changed.connect(self._update_grid)
        self._parameter_panel.arrows_changed.connect(self._refresh_route_display)
        self._parameter_panel.sweep_changed.connect(self._on_sweep_changed)

    def _build_menus(self) -> None:
        mb = self.menuBar()

        file_menu = mb.addMenu("File")
        open_act = QAction("Open STL / OBJ...", self)
        open_act.setShortcut("Ctrl+O")
        open_act.triggered.connect(self._open_file)
        file_menu.addAction(open_act)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close)

        view_menu = mb.addMenu("View")
        view_menu.addAction("Fit All\tCtrl+Home",  lambda: self._viewer.fit_all())
        view_menu.addAction("Top View",             lambda: self._viewer.set_view('top'))
        view_menu.addAction("Front View",           lambda: self._viewer.set_view('front'))
        view_menu.addAction("Side View",            lambda: self._viewer.set_view('side'))
        view_menu.addSeparator()
        view_menu.addAction("Rotate Left  90",      lambda: self._viewer.roll_view(-90))
        view_menu.addAction("Rotate Right 90",      lambda: self._viewer.roll_view(+90))

        path_menu = mb.addMenu("Path")
        path_menu.addAction("Generate\tCtrl+G",    self._on_generate)
        path_menu.addAction("Flip Direction",       self._flip_direction)
        path_menu.addSeparator()
        path_menu.addAction("Clear Paths",          self._clear_paths)

        export_menu = mb.addMenu("Export")
        export_menu.addAction("Export JSON...", self._export_json)
        export_menu.addAction("Export CSV...",  self._export_csv)

    # ------------------------------------------------------------------
    # Sweep direction (CW / CCW)
    # ------------------------------------------------------------------

    def _on_sweep_changed(self) -> None:
        if self._current_routes:
            self._on_generate()

    def _flip_direction(self) -> None:
        """Path menu shortcut — toggles the CCW radio button."""
        pp = self._parameter_panel
        pp._ccw_radio.setChecked(not pp._ccw_radio.isChecked())

    # ------------------------------------------------------------------
    # File loading
    # ------------------------------------------------------------------

    def _open_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Mesh File", "",
            "Mesh Files (*.stl *.obj);;All Files (*)"
        )
        if path:
            self._load(path)

    def dragEnterEvent(self, event: QDragEnterEvent) -> None:
        if event.mimeData().hasUrls():
            urls = event.mimeData().urls()
            if any(u.toLocalFile().lower().endswith(('.stl', '.obj')) for u in urls):
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
        self._surface_panel.set_enabled(False)
        self._parameter_panel.set_enabled(False)
        self._hint_label.hide()
        self._overlay.show_message(f"Opening {os.path.basename(filepath)}...")
        self._overlay.show()
        self._overlay.raise_()

        worker = _LoadWorker(filepath, up_axis)
        worker.progress.connect(self._overlay.show_message)
        worker.finished.connect(self._on_load_ready)
        worker.error.connect(self._on_load_error)
        self._load_worker = worker
        worker.start()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._overlay.isVisible():
            self._overlay.setGeometry(self.centralWidget().rect())

    def _on_load_ready(self, model) -> None:
        self._overlay.hide()
        self._model = model
        n_faces = len(model.data.trimesh_mesh.faces)

        self._viewer.load_mesh(model.data)
        self._viewer.enable_bbox_clicking(self._on_bbox_region_clicked)

        self._surface_panel.set_enabled(True)
        self._parameter_panel.set_enabled(True)
        self._status_panel.update_mesh_stats(n_faces)
        self._status_panel.clear()

        self.statusBar().showMessage(
            f"Loaded: {os.path.basename(model.data.source_path)}"
            f"  |  {n_faces:,} triangles  —  click a box face to select a region."
        )

    def _on_load_error(self, message: str) -> None:
        self._overlay.hide()
        self._hint_label.show()
        QMessageBox.critical(self, "Load error", message)
        self.statusBar().showMessage("Load failed.")

    # ------------------------------------------------------------------
    # Bbox region selection
    # ------------------------------------------------------------------

    def _on_bbox_region_clicked(self, region: str) -> None:
        if region in self._selected_regions:
            self._selected_regions.discard(region)
            self._viewer.highlight_bbox_region(region, False)
            self._surface_panel.toggle_region_checkbox(region, False)
        else:
            self._selected_regions.add(region)
            self._viewer.highlight_bbox_region(region, True)
            self._surface_panel.toggle_region_checkbox(region, True)

        n = len(self._selected_regions)
        self.statusBar().showMessage(
            f"{n} region(s) selected: {', '.join(sorted(self._selected_regions))}"
            if n else "Click a bounding box face to select it."
        )
        self._update_grid()

    # ------------------------------------------------------------------
    # Region checkboxes (sidebar shortcuts)
    # ------------------------------------------------------------------

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
            f"{n} region(s) selected: {', '.join(sorted(self._selected_regions))}"
            if n else "Click a bounding box face to select it."
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

        spray_mm    = self._parameter_panel.get_spray_width_mm()
        path_target = self._parameter_panel.get_path_target()

        if path_target == 'bbox':
            self._generate_bbox(spray_mm)
        else:
            self._generate_mesh(spray_mm)

    def _generate_bbox(self, spray_mm: float) -> None:
        if not self._selected_regions:
            QMessageBox.warning(self, "No selection",
                "Click a bounding box face to select it, then generate.")
            return

        bounds    = self._model.data.pyvista_mesh.bounds
        up        = self._model.data.up_axis
        direction = self._parameter_panel.get_direction()
        v_mm      = self._parameter_panel.get_v_width_mm() or spray_mm
        routes: list[PaintRoute] = []
        offset = 1 if self._parameter_panel.is_direction_flipped() else 0

        for region in sorted(self._selected_regions):
            try:
                if direction in ('horizontal', 'both'):
                    routes.append(_bbox_generator.generate_bbox_route(
                        region, bounds, spray_mm, up,
                        direction='horizontal', direction_offset=offset))
                if direction in ('vertical', 'both'):
                    routes.append(_bbox_generator.generate_bbox_route(
                        region, bounds, v_mm, up,
                        direction='vertical', direction_offset=offset))
            except Exception as exc:
                QMessageBox.critical(self, "Generation error",
                    f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}")
                return

        self._on_route_ready(routes)

    def _generate_mesh(self, spray_mm: float) -> None:
        pairs: list = []
        for region_id in sorted(self._selected_regions):
            faces = self._model.get_region_faces(region_id)
            if len(faces) > 0:
                pairs.append((region_id, faces))

        if not pairs:
            QMessageBox.warning(self, "No selection",
                "Select bounding box regions first.")
            return

        self._parameter_panel.set_generating(True)
        self.statusBar().showMessage("Generating mesh paths...")

        worker = _PathWorker(self._model.data, pairs, spray_mm)
        worker.finished.connect(self._on_route_ready)
        worker.error.connect(self._on_route_error)
        self._worker = worker
        worker.start()

    def _on_route_ready(self, routes: list[PaintRoute]) -> None:
        self._parameter_panel.set_generating(False)
        self._current_routes = routes
        self._viewer.show_route(routes, show_arrows=self._parameter_panel.is_show_arrows())
        total_passes = sum(r.total_passes for r in routes)
        total_conns  = sum(len(r.connections) for r in routes)
        self._status_panel.update_route_stats(routes, self._parameter_panel.current_unit)
        self.statusBar().showMessage(
            f"Done  —  {total_passes} passes, {total_conns} connections."
        )

    def _refresh_route_display(self) -> None:
        if self._current_routes:
            self._viewer.show_route(
                self._current_routes,
                show_arrows=self._parameter_panel.is_show_arrows(),
            )

    def _on_route_error(self, message: str) -> None:
        self._parameter_panel.set_generating(False)
        QMessageBox.critical(self, "Generation error", message)
        self.statusBar().showMessage("Generation failed.")

    def _clear_paths(self) -> None:
        self._viewer.clear_route()
        self._viewer.clear_bbox_grid()
        self._current_routes = []
        self._status_panel.clear()
        self.statusBar().showMessage("Paths cleared.")

    def _update_grid(self) -> None:
        self._viewer.clear_bbox_grid()
        if not self._parameter_panel.is_show_grid():
            return
        if self._model.data is None:
            return
        bounds = tuple(self._model.data.pyvista_mesh.bounds)
        h_mm = self._parameter_panel.get_spray_width_mm()
        v_mm = self._parameter_panel.get_v_width_mm()
        up   = self._model.data.up_axis
        for region in self._selected_regions:
            self._viewer.show_bbox_grid(region, bounds, up, h_mm, v_mm or h_mm)

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------

    def _export_json(self) -> None:
        if not self._current_routes:
            QMessageBox.warning(self, "Nothing to export", "Generate a path first.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export JSON", "", "JSON (*.json)")
        if path:
            export_route_json(self._current_routes, path)
            self.statusBar().showMessage(f"Exported: {path}")

    def _export_csv(self) -> None:
        if not self._current_routes:
            QMessageBox.warning(self, "Nothing to export", "Generate a path first.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export CSV", "", "CSV (*.csv)")
        if path:
            export_route_csv(self._current_routes, path)
            self.statusBar().showMessage(f"Exported: {path}")
