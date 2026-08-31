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
    QLabel, QVBoxLayout as QVBox,
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
/* ── whole left panel ──────────────────────── */
QWidget { background:#252536; color:#e8e8ff; font-size:13px; }

/* ── group boxes ────────────────────────────── */
QGroupBox {
    color:#b0b8ff;
    font-weight:bold;
    font-size:13px;
    border:1px solid #44446a;
    border-radius:6px;
    margin-top:10px;
    padding-top:8px;
}
QGroupBox::title {
    subcontrol-origin:margin;
    left:10px;
    padding:0 5px;
    color:#9090ff;
}

/* ── labels ─────────────────────────────────── */
QLabel { color:#d0d8ff; font-size:12px; background:transparent; }

/* ── buttons ─────────────────────────────────── */
QPushButton {
    background:#35355a;
    color:#d8d8ff;
    border:1px solid #55558a;
    border-radius:4px;
    padding:6px 4px;
    font-size:12px;
}
QPushButton:hover   { background:#45458a; color:#ffffff; }
QPushButton:checked { background:#1565C0; color:#ffffff; border-color:#42A5F5; }
QPushButton:disabled { background:#2a2a40; color:#555577; }

/* ── checkboxes ───────────────────────────────── */
QCheckBox { color:#d0d8ff; font-size:12px; spacing:8px; background:transparent; }
QCheckBox::indicator {
    width:14px; height:14px;
    border:2px solid #6666aa;
    border-radius:3px;
    background:#2a2a45;
}
QCheckBox::indicator:checked   { background:#1976D2; border-color:#64B5F6; }
QCheckBox::indicator:unchecked:hover { border-color:#9999cc; }

/* ── radio buttons ───────────────────────────── */
QRadioButton { color:#d0d8ff; font-size:12px; spacing:8px; background:transparent; }
QRadioButton::indicator {
    width:14px; height:14px;
    border:2px solid #6666aa;
    border-radius:7px;
    background:#2a2a45;
}
QRadioButton::indicator:checked {
    background:#42A5F5;
    border-color:#90CAF9;
}
QRadioButton::indicator:unchecked:hover { border-color:#9999cc; }

/* ── spin boxes ───────────────────────────────── */
QDoubleSpinBox {
    background:#2e2e50;
    color:#e8e8ff;
    border:1px solid #55558a;
    border-radius:4px;
    padding:4px 6px;
    font-size:12px;
    min-height:26px;
}
QDoubleSpinBox:focus { border-color:#7070cc; }
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background:#35355a; border:none; width:16px;
}

/* ── combo boxes ──────────────────────────────── */
QComboBox {
    background:#2e2e50;
    color:#e8e8ff;
    border:1px solid #55558a;
    border-radius:4px;
    padding:4px 6px;
    font-size:12px;
    min-height:26px;
}
QComboBox::drop-down { border:none; width:20px; }
QComboBox QAbstractItemView {
    background:#2e2e50; color:#e8e8ff;
    selection-background-color:#1565C0;
}

/* ── scroll area ──────────────────────────────── */
QScrollArea { border:none; }

/* ── frame (dividers) ─────────────────────────── */
QFrame[frameShape="4"] { color:#44446a; }
"""


# ---------------------------------------------------------------------------
# Background worker — mesh-surface path generation (may be slow)
# ---------------------------------------------------------------------------

class _PathWorker(QThread):
    finished = Signal(object)   # list[PaintRoute]
    error    = Signal(str)

    def __init__(
        self,
        mesh_data: MeshData,
        region_face_pairs: list,   # list of (region_id, np.ndarray)
    ) -> None:
        super().__init__()
        self._mesh_data = mesh_data
        self._pairs = region_face_pairs

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

    # spray_mm is set after __init__ to match the original call-site pattern
    _spray_mm: float = 50.0


class _FaceWorker(QThread):
    """Background worker for manually picked face sets (region_id='selection')."""
    finished = Signal(object)
    error    = Signal(str)

    def __init__(self, mesh_data: MeshData, face_indices: np.ndarray, spray_mm: float) -> None:
        super().__init__()
        self._mesh_data  = mesh_data
        self._faces      = face_indices
        self._spray_mm   = spray_mm

    def run(self) -> None:
        try:
            route = _generator.generate_route(
                self._mesh_data,
                region_id='selection',
                region_face_indices=self._faces,
                spray_width_mm=self._spray_mm,
            )
            self.finished.emit([route])
        except Exception as exc:
            self.error.emit(f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}")


# ---------------------------------------------------------------------------
# Up-axis dialog
# ---------------------------------------------------------------------------

class _UpAxisDialog(QDialog):
    def __init__(self, filename: str, n_faces: int, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Mesh loaded")
        layout = QVBox(self)
        layout.addWidget(QLabel(f"<b>{os.path.basename(filename)}</b>"))
        layout.addWidget(QLabel(f"{n_faces:,} triangles"))
        layout.addWidget(QLabel("Which axis is UP in this file?"))
        self._group = QButtonGroup(self)
        self._btns: list[QRadioButton] = []
        for i, label in enumerate(['X  (axis 0)', 'Y  (axis 1)', 'Z  (axis 2) — default']):
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
        self.setWindowTitle("SurfaceCoat — Paint Path Generator")
        self.resize(1280, 820)
        self.setAcceptDrops(True)

        self._model               = MeshModel()
        self._selected_regions:   set[str]       = set()   # bbox-face selections
        self._selected_faces:     np.ndarray     = np.array([], dtype=np.int64)  # manual picks
        self._current_routes:     list[PaintRoute] = []
        self._worker:             Optional[QThread] = None
        self._pick_mode:          str             = 'none'

        self._build_ui()
        self._build_menus()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        left_widget = QWidget()
        left_widget.setFixedWidth(270)
        left_widget.setStyleSheet(_SIDEBAR_STYLE)
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_layout.setSpacing(10)

        self._surface_panel   = SurfacePanel()
        self._parameter_panel = ParameterPanel()
        self._status_panel    = StatusPanel()

        left_layout.addWidget(self._surface_panel)
        left_layout.addWidget(self._parameter_panel)
        left_layout.addWidget(self._status_panel)
        left_layout.addStretch()

        root.addWidget(left_widget)

        self._viewer = MeshViewer()
        root.addWidget(self._viewer, stretch=1)

        self.statusBar().showMessage("Open an STL or OBJ file to begin.")

        self._surface_panel.pick_mode_requested.connect(self._on_pick_mode)
        self._surface_panel.region_shortcut_requested.connect(self._on_region_shortcut)
        self._parameter_panel.generate_requested.connect(self._on_generate)
        self._parameter_panel.clear_requested.connect(self._clear_paths)
        self._parameter_panel.grid_changed.connect(self._update_grid)

    def _build_menus(self) -> None:
        mb = self.menuBar()

        file_menu = mb.addMenu("File")
        open_act = QAction("Open STL / OBJ…", self)
        open_act.setShortcut("Ctrl+O")
        open_act.triggered.connect(self._open_file)
        file_menu.addAction(open_act)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close)

        view_menu = mb.addMenu("View")
        view_menu.addAction("Fit All",    lambda: self._viewer.fit_all())
        view_menu.addAction("Top View",   lambda: self._viewer.set_view('top'))
        view_menu.addAction("Front View", lambda: self._viewer.set_view('front'))
        view_menu.addAction("Side View",  lambda: self._viewer.set_view('side'))

        path_menu = mb.addMenu("Path")
        path_menu.addAction("Generate",    self._on_generate)
        path_menu.addAction("Clear Paths", self._clear_paths)

        export_menu = mb.addMenu("Export")
        export_menu.addAction("Export JSON…", self._export_json)
        export_menu.addAction("Export CSV…",  self._export_csv)

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
        self.statusBar().showMessage(f"Loading {os.path.basename(filepath)}…")
        try:
            import trimesh as _tm
            raw = _tm.load(filepath, force='mesh')
            if isinstance(raw, _tm.Scene):
                raw = raw.dump(concatenate=True)
            n_faces = len(raw.faces)
        except Exception as exc:
            QMessageBox.critical(self, "Load error", str(exc))
            return

        dlg = _UpAxisDialog(filepath, n_faces, self)
        dlg.exec()
        up_axis = dlg.up_axis()

        try:
            self._model.load(filepath, up_axis=up_axis)
        except Exception as exc:
            QMessageBox.critical(self, "Load error", str(exc))
            return

        self._selected_regions.clear()
        self._selected_faces  = np.array([], dtype=np.int64)
        self._current_routes  = []
        self._pick_mode       = 'none'

        self._viewer.load_mesh(self._model.data)
        # Default: clicking the bounding box selects regions
        self._viewer.enable_bbox_clicking(self._on_bbox_region_clicked)

        self._surface_panel.set_enabled(True)
        self._surface_panel.reset_pick_buttons()
        self._parameter_panel.set_enabled(True)
        self._status_panel.update_mesh_stats(n_faces)
        self._status_panel.clear()

        self.statusBar().showMessage(
            f"Loaded: {os.path.basename(filepath)}  |  {n_faces:,} triangles  "
            f"— Click a box face to select it."
        )

    # ------------------------------------------------------------------
    # Bbox region selection (default click mode)
    # ------------------------------------------------------------------

    def _on_bbox_region_clicked(self, region: str) -> None:
        """Toggle the clicked bbox face. Only the face color changes — mesh untouched."""
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
    # Region checkboxes (sidebar shortcuts — drive bbox selection)
    # ------------------------------------------------------------------

    def _on_region_shortcut(self, region_id: str, checked: bool) -> None:
        """Sidebar checkbox toggled — update bbox face highlight and region state."""
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
    # Pick mode buttons (Pick Faces / Flood Fill / Clear)
    # ------------------------------------------------------------------

    def _on_pick_mode(self, mode: str) -> None:
        self._pick_mode = mode

        if mode == 'clear':
            self._selected_regions.clear()
            self._selected_faces = np.array([], dtype=np.int64)
            self._viewer.clear_bbox_selection()
            self._viewer.clear_mesh_highlight()
            for r in _REGIONS:
                self._surface_panel.toggle_region_checkbox(r, False)
            self._surface_panel.reset_pick_buttons()
            # Return to bbox clicking
            self._viewer.enable_bbox_clicking(self._on_bbox_region_clicked)
            self._pick_mode = 'none'
            self.statusBar().showMessage("Selection cleared.")

        elif mode == 'single':
            self._viewer.enable_face_picking(self._on_face_picked)
            self.statusBar().showMessage("Click mesh faces to select them.")

        elif mode == 'none':
            # Pick/Flood button un-toggled → return to bbox mode
            self._viewer.enable_bbox_clicking(self._on_bbox_region_clicked)
            self.statusBar().showMessage("Bbox face click mode active.")

    def _on_face_picked(self, face_id: int) -> None:
        if self._model.data is None:
            return
        new_faces = np.array([face_id], dtype=np.int64)

        self._selected_faces = np.unique(
            np.concatenate([self._selected_faces, new_faces])
        ).astype(np.int64)

        self._viewer.highlight_mesh_faces(self._selected_faces)
        self.statusBar().showMessage(f"{len(self._selected_faces):,} mesh faces selected.")

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
        """Flat paths on bbox face planes — synchronous (instant)."""
        if not self._selected_regions:
            QMessageBox.warning(self, "No selection",
                "Click a bounding box face to select it, then generate.")
            return

        bounds = self._model.data.pyvista_mesh.bounds
        routes: list[PaintRoute] = []
        v_mm = self._parameter_panel.get_vertical_width_mm()
        for region in sorted(self._selected_regions):
            try:
                route = _bbox_generator.generate_bbox_route(
                    region, bounds, spray_mm, self._model.data.up_axis,
                    v_width_mm=v_mm,
                )
                routes.append(route)
            except Exception as exc:
                QMessageBox.critical(self, "Generation error",
                    f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}")
                return

        self._on_route_ready(routes)

    def _generate_mesh(self, spray_mm: float) -> None:
        """Mesh-surface paths — uses a background thread."""
        pairs: list = []

        if self._selected_regions:
            for region_id in sorted(self._selected_regions):
                faces = self._model.get_region_faces(region_id)
                if len(faces) > 0:
                    pairs.append((region_id, faces))
            if not pairs:
                QMessageBox.warning(self, "No selection",
                    "No mesh faces found for the selected regions.")
                return
        elif len(self._selected_faces) > 0:
            pairs = [('selection', self._selected_faces.copy())]
        else:
            QMessageBox.warning(self, "No selection",
                "Select bounding box regions or pick mesh faces first.")
            return

        self._parameter_panel.set_generating(True)
        self.statusBar().showMessage("Generating mesh paths…")

        worker = _PathWorker(self._model.data, pairs)
        worker._spray_mm = spray_mm
        worker.finished.connect(self._on_route_ready)
        worker.error.connect(self._on_route_error)
        self._worker = worker
        worker.start()

    def _on_route_ready(self, routes: list[PaintRoute]) -> None:
        self._parameter_panel.set_generating(False)
        self._current_routes = routes
        self._viewer.show_route(routes)
        total_passes = sum(r.total_passes for r in routes)
        total_conns  = sum(len(r.connections) for r in routes)
        self._status_panel.update_route_stats(routes, self._parameter_panel.current_unit)
        self.statusBar().showMessage(
            f"Done — {total_passes} passes, {total_conns} connections."
        )

    def _on_route_error(self, message: str) -> None:
        self._parameter_panel.set_generating(False)
        QMessageBox.critical(self, "Generation error", message)
        self.statusBar().showMessage("Generation failed.")

    def _clear_paths(self) -> None:
        self._viewer.clear_route()
        self._current_routes = []
        self._status_panel.clear()

    def _update_grid(self) -> None:
        """Redraw (or remove) the grey width-grid on all selected bbox faces."""
        self._viewer.clear_bbox_grid()
        if not self._parameter_panel.is_show_grid():
            return
        if self._model.data is None:
            return
        bounds = tuple(self._model.data.pyvista_mesh.bounds)
        h_mm = self._parameter_panel.get_spray_width_mm()
        v_mm = self._parameter_panel.get_vertical_width_mm()
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
