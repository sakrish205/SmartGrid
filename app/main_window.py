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
    QProgressBar, QToolBar,
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
/* ── whole left panel — Office light ─────────── */
QWidget { background:#f3f2f1; color:#252525; font-size:13px; }

/* ── group boxes ─────────────────────────────── */
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

/* ── labels ──────────────────────────────────── */
QLabel { color:#323130; font-size:12px; background:transparent; }

/* ── buttons ─────────────────────────────────── */
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

/* ── checkboxes ──────────────────────────────── */
QCheckBox { color:#323130; font-size:12px; spacing:8px; background:transparent; }
QCheckBox::indicator {
    width:14px; height:14px;
    border:2px solid #8a8886;
    border-radius:3px;
    background:#ffffff;
}
QCheckBox::indicator:checked { background:#0078D4; border-color:#0078D4; }
QCheckBox::indicator:unchecked:hover { border-color:#0078D4; }

/* ── radio buttons ───────────────────────────── */
QRadioButton { color:#323130; font-size:12px; spacing:8px; background:transparent; }
QRadioButton::indicator {
    width:14px; height:14px;
    border:2px solid #8a8886;
    border-radius:7px;
    background:#ffffff;
}
QRadioButton::indicator:checked { background:#0078D4; border-color:#0078D4; }
QRadioButton::indicator:unchecked:hover { border-color:#0078D4; }

/* ── spin boxes ──────────────────────────────── */
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

/* ── combo boxes ─────────────────────────────── */
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

/* ── scroll area ─────────────────────────────── */
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

/* ── frame (dividers) ────────────────────────── */
QFrame[frameShape="4"] { color:#d2d0ce; }
"""

_TOOLBAR_STYLE = """
QToolBar {
    background:#f3f2f1;
    border-bottom:1px solid #d2d0ce;
    spacing:2px;
    padding:2px 6px;
}
QToolButton {
    background:transparent;
    border:1px solid transparent;
    border-radius:3px;
    padding:4px 8px;
    font-size:12px;
    color:#252525;
    min-width:28px;
}
QToolButton:hover   { background:#edebe9; border-color:#d2d0ce; }
QToolButton:checked { background:#dce6f7; border-color:#0078D4; color:#0078D4; font-weight:bold; }
QToolButton:pressed { background:#d0d8ec; }
QToolBar::separator { background:#d2d0ce; width:1px; margin:4px 3px; }
"""


# ---------------------------------------------------------------------------
# Loading overlay — covers viewport while mesh loads
# ---------------------------------------------------------------------------

class _LoadingOverlay(QWidget):
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.setStyleSheet("background:rgba(243,242,241,210);")

        outer = QVBoxLayout(self)
        outer.setAlignment(Qt.AlignmentFlag.AlignCenter)

        panel = QWidget()
        panel.setFixedWidth(320)
        panel.setStyleSheet(
            "QWidget { background:#ffffff; border:1px solid #d2d0ce;"
            " border-radius:8px; padding:20px; }"
        )
        inner = QVBoxLayout(panel)
        inner.setSpacing(12)

        self._label = QLabel("Loading…")
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setStyleSheet(
            "font-size:14px; color:#252525; font-weight:bold; border:none;"
        )

        self._bar = QProgressBar()
        self._bar.setRange(0, 0)
        self._bar.setFixedHeight(6)
        self._bar.setTextVisible(False)
        self._bar.setStyleSheet(
            "QProgressBar { background:#e0dfde; border:none; border-radius:3px; }"
            "QProgressBar::chunk { background:#0078D4; border-radius:3px; }"
        )

        inner.addWidget(self._label)
        inner.addWidget(self._bar)
        outer.addWidget(panel)
        self.hide()

    def show_message(self, msg: str) -> None:
        self._label.setText(msg)

    def showEvent(self, e) -> None:
        super().showEvent(e)
        self.setGeometry(self.parentWidget().rect())


# ---------------------------------------------------------------------------
# Background worker — mesh file loading (blocking I/O + preprocessing)
# ---------------------------------------------------------------------------

class _LoadWorker(QThread):
    finished = Signal(object)   # emits MeshModel (fully loaded)
    progress = Signal(str)
    error    = Signal(str)

    def __init__(self, filepath: str, up_axis: int) -> None:
        super().__init__()
        self._filepath = filepath
        self._up_axis  = up_axis

    def run(self) -> None:
        try:
            from models.mesh_model import MeshModel as _MM
            self.progress.emit(f"Reading {os.path.basename(self._filepath)}…")
            model = _MM()
            model.load(self._filepath, up_axis=self._up_axis)
            self.progress.emit("Finalising…")
            self.finished.emit(model)
        except Exception as exc:
            self.error.emit(f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}")


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
    def __init__(self, filename: str, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Open mesh")
        layout = QVBox(self)
        layout.addWidget(QLabel(f"<b>{os.path.basename(filename)}</b>"))
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
        self.setWindowTitle("SmartGrid — 3D Surface Grid & Pitch Mapper")
        self.resize(1280, 820)
        self.setAcceptDrops(True)

        self._model              = MeshModel()
        self._selected_regions:  set[str]         = set()
        self._current_routes:    list[PaintRoute] = []
        self._worker:            Optional[QThread] = None
        self._load_worker:       Optional[QThread] = None
        self._direction_flipped: bool             = False

        self._build_ui()
        self._build_menus()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        # Loading overlay — child of central; covers the whole viewport while loading
        self._overlay = _LoadingOverlay(central)

        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Sidebar scroll container ────────────────────────────────────
        sidebar_container = QWidget()
        sidebar_container.setFixedWidth(270)
        sidebar_container.setStyleSheet(_SIDEBAR_STYLE)
        sidebar_outer = QVBoxLayout(sidebar_container)
        sidebar_outer.setContentsMargins(0, 0, 0, 0)
        sidebar_outer.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        inner = QWidget()
        left_layout = QVBoxLayout(inner)
        left_layout.setContentsMargins(8, 8, 8, 8)
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

        self._build_toolbar()

    def _build_toolbar(self) -> None:
        tb = QToolBar("View")
        tb.setMovable(False)
        tb.setStyleSheet(_TOOLBAR_STYLE)
        self.addToolBar(tb)

        def act(label: str, tip: str, slot, checkable: bool = False) -> QAction:
            a = QAction(label, self)
            a.setToolTip(tip)
            a.setCheckable(checkable)
            a.triggered.connect(slot)
            tb.addAction(a)
            return a

        act("⌂ Home",  "Fit all",       self._viewer.fit_all)
        act("↑ Top",   "Top view (Z)",  lambda: self._viewer.set_view('top'))
        act("◻ Front", "Front view",    lambda: self._viewer.set_view('front'))
        act("◁ Side",  "Side view",     lambda: self._viewer.set_view('side'))
        tb.addSeparator()
        act("↺ CCW", "Rotate view 90° counter-clockwise", lambda: self._viewer.roll_view(-90))
        act("↻ CW",  "Rotate view 90° clockwise",         lambda: self._viewer.roll_view(+90))
        tb.addSeparator()

        # Grid toggle — synced with sidebar checkbox
        self._tb_grid = act("⊞ Grid", "Show pitch grid on bbox faces",
                            self._on_toolbar_grid, checkable=True)
        self._parameter_panel._show_grid_check.toggled.connect(
            lambda v: self._tb_grid.setChecked(v) if self._tb_grid.isChecked() != v else None)

        # Arrow toggle — synced with sidebar checkbox
        self._tb_arrows = act("→ Arrows", "Show direction arrows",
                              self._on_toolbar_arrows, checkable=True)
        self._parameter_panel._show_arrows_check.toggled.connect(
            lambda v: self._tb_arrows.setChecked(v) if self._tb_arrows.isChecked() != v else None)

        tb.addSeparator()
        act("⇄ Flip Dir", "Flip path sweep direction (CW ↔ CCW)", self._flip_direction)

    def _on_toolbar_grid(self, checked: bool) -> None:
        cb = self._parameter_panel._show_grid_check
        cb.blockSignals(True)
        cb.setChecked(checked)
        cb.blockSignals(False)
        self._update_grid()

    def _on_toolbar_arrows(self, checked: bool) -> None:
        cb = self._parameter_panel._show_arrows_check
        cb.blockSignals(True)
        cb.setChecked(checked)
        cb.blockSignals(False)
        self._refresh_route_display()

    def _flip_direction(self) -> None:
        self._direction_flipped = not self._direction_flipped
        if self._current_routes:
            self._on_generate()  # regenerate with new offset

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
        if self._load_worker and self._load_worker.isRunning():
            return

        dlg = _UpAxisDialog(filepath, self)
        dlg.exec()
        up_axis = dlg.up_axis()

        self._selected_regions.clear()
        self._current_routes = []
        self._surface_panel.set_enabled(False)
        self._parameter_panel.set_enabled(False)
        self._overlay.show_message(f"Opening {os.path.basename(filepath)}…")
        self._overlay.show()
        self._overlay.raise_()

        worker = _LoadWorker(filepath, up_axis)
        worker.progress.connect(self._overlay.show_message)
        worker.finished.connect(self._on_load_ready)
        worker.error.connect(self._on_load_error)
        self._load_worker = worker
        worker.start()

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
            f"  |  {n_faces:,} triangles  — Click a box face to select it."
        )

    def _on_load_error(self, message: str) -> None:
        self._overlay.hide()
        QMessageBox.critical(self, "Load error", message)
        self.statusBar().showMessage("Load failed.")

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

        bounds    = self._model.data.pyvista_mesh.bounds
        up        = self._model.data.up_axis
        direction = self._parameter_panel.get_direction()
        v_mm      = self._parameter_panel.get_v_width_mm() or spray_mm
        routes: list[PaintRoute] = []

        offset = 1 if self._direction_flipped else 0
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
        else:
            QMessageBox.warning(self, "No selection",
                "Select bounding box regions first.")
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
        self._viewer.show_route(routes, show_arrows=self._parameter_panel.is_show_arrows())
        total_passes = sum(r.total_passes for r in routes)
        total_conns  = sum(len(r.connections) for r in routes)
        self._status_panel.update_route_stats(routes, self._parameter_panel.current_unit)
        self.statusBar().showMessage(
            f"Done — {total_passes} passes, {total_conns} connections."
        )

    def _refresh_route_display(self) -> None:
        """Re-render existing routes when arrow visibility is toggled."""
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
        self._viewer.clear_route()       # removes all pass_ and conn_ actors
        self._viewer.clear_bbox_grid()   # removes any grid overlays
        self._current_routes = []
        self._status_panel.clear()
        self.statusBar().showMessage("Paths cleared.")

    def _update_grid(self) -> None:
        """Redraw (or remove) the grey width-grid on all selected bbox faces."""
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
