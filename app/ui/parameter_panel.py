"""Unit dropdown, spray width spinboxes, path-target toggle, Generate button."""
from __future__ import annotations

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QFormLayout, QComboBox,
    QDoubleSpinBox, QRadioButton, QButtonGroup, QPushButton,
    QLabel, QCheckBox, QFrame,
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

from app.path.path_model import UNIT_TO_MM

UNITS = ['mm', 'cm', 'm', 'in', 'ft']


class ParameterPanel(QGroupBox):
    generate_requested = Signal()
    clear_requested    = Signal()
    grid_changed       = Signal()   # emitted when show-grid toggle or widths change

    def __init__(self, parent=None) -> None:
        super().__init__("Path Settings", parent)
        self._current_unit = 'mm'
        self._setup_ui()
        self.set_enabled(False)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(8, 12, 8, 8)

        # ── Unit ────────────────────────────────────────────────────────
        form = QFormLayout()
        form.setSpacing(8)
        form.setLabelAlignment(Qt.AlignRight)

        self._unit_combo = QComboBox()
        self._unit_combo.addItems(UNITS)
        self._unit_combo.setCurrentText('mm')
        self._unit_combo.currentTextChanged.connect(self._on_unit_changed)
        form.addRow(_lbl("Unit:"), self._unit_combo)

        # ── Horizontal spray width (required) ───────────────────────────
        self._h_spin = _spin()
        form.addRow(_lbl("Horiz. width:"), self._h_spin)

        layout.addLayout(form)

        # ── Vertical width (optional) ────────────────────────────────────
        self._v_check = QCheckBox("Vertical width (optional):")
        self._v_check.setChecked(False)
        self._v_check.toggled.connect(lambda on: self._v_spin.setEnabled(on))
        layout.addWidget(self._v_check)

        self._v_spin = _spin()
        self._v_spin.setEnabled(False)
        layout.addWidget(self._v_spin)

        # ── Show Grid checkbox ───────────────────────────────────────────
        self._show_grid_check = QCheckBox("Show grid")
        self._show_grid_check.setChecked(False)
        self._show_grid_check.toggled.connect(lambda _: self.grid_changed.emit())
        layout.addWidget(self._show_grid_check)

        # ── Emit grid_changed when widths change ─────────────────────────
        self._h_spin.valueChanged.connect(lambda _: self.grid_changed.emit())
        self._v_spin.valueChanged.connect(lambda _: self.grid_changed.emit())
        self._v_check.toggled.connect(lambda _: self.grid_changed.emit())

        # ── Divider ──────────────────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color:#555577;")
        layout.addWidget(sep)

        # ── Path landing target ──────────────────────────────────────────
        layout.addWidget(_lbl("Path lands on:", bold=True))

        target_group = QButtonGroup(self)
        self._bbox_target_radio = QRadioButton("Boundary Box")
        self._bbox_target_radio.setChecked(True)
        self._mesh_target_radio = QRadioButton("Mesh Surface")
        target_group.addButton(self._bbox_target_radio, 0)
        target_group.addButton(self._mesh_target_radio, 1)
        layout.addWidget(self._bbox_target_radio)
        layout.addWidget(self._mesh_target_radio)

        layout.addStretch()

        # ── Generate + Clear buttons ─────────────────────────────────────
        self._gen_btn = QPushButton("GENERATE PATH")
        self._gen_btn.setMinimumHeight(42)
        f = QFont(); f.setBold(True); f.setPointSize(10)
        self._gen_btn.setFont(f)
        self._gen_btn.setStyleSheet(
            "QPushButton { background:#1565C0; color:#ffffff; border-radius:5px; }"
            "QPushButton:hover { background:#1976D2; }"
            "QPushButton:disabled { background:#3a3a50; color:#666; }"
        )
        self._gen_btn.clicked.connect(self.generate_requested.emit)
        layout.addWidget(self._gen_btn)

        self._clear_btn = QPushButton("Clear Path")
        self._clear_btn.setMinimumHeight(32)
        self._clear_btn.setStyleSheet(
            "QPushButton { background:#4a2020; color:#ffaaaa; border:1px solid #8b3333;"
            " border-radius:5px; }"
            "QPushButton:hover { background:#6b2a2a; color:#ffffff; }"
            "QPushButton:disabled { background:#2a2a40; color:#555577; }"
        )
        self._clear_btn.clicked.connect(self.clear_requested.emit)
        layout.addWidget(self._clear_btn)

        # suffix update after unit combo initialises
        self._h_spin.setSuffix('  mm')
        self._v_spin.setSuffix('  mm')

    # ------------------------------------------------------------------
    def _on_unit_changed(self, new_unit: str) -> None:
        for spin in (self._h_spin, self._v_spin):
            old_mm = spin.value() * UNIT_TO_MM[self._current_unit]
            spin.blockSignals(True)
            spin.setValue(old_mm / UNIT_TO_MM[new_unit])
            spin.setSuffix(f'  {new_unit}')
            spin.blockSignals(False)
        self._current_unit = new_unit

    # ------------------------------------------------------------------
    def get_spray_width_mm(self) -> float:
        return self._h_spin.value() * UNIT_TO_MM[self._current_unit]

    def get_vertical_width_mm(self) -> float | None:
        if self._v_check.isChecked():
            return self._v_spin.value() * UNIT_TO_MM[self._current_unit]
        return None

    def get_path_target(self) -> str:
        return 'bbox' if self._bbox_target_radio.isChecked() else 'mesh'

    def get_direction(self) -> str:
        return 'horizontal'

    @property
    def current_unit(self) -> str:
        return self._current_unit

    def is_show_grid(self) -> bool:
        return self._show_grid_check.isChecked()

    def set_enabled(self, enabled: bool) -> None:
        self._unit_combo.setEnabled(enabled)
        self._h_spin.setEnabled(enabled)
        self._v_check.setEnabled(enabled)
        self._show_grid_check.setEnabled(enabled)
        self._v_spin.setEnabled(enabled and self._v_check.isChecked())
        self._bbox_target_radio.setEnabled(enabled)
        self._mesh_target_radio.setEnabled(enabled)
        self._gen_btn.setEnabled(enabled)
        self._clear_btn.setEnabled(enabled)

    def set_generating(self, generating: bool) -> None:
        self._gen_btn.setEnabled(not generating)
        self._clear_btn.setEnabled(not generating)
        self._gen_btn.setText("Generating…" if generating else "GENERATE PATH")


# ------------------------------------------------------------------
def _spin() -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(0.1, 100_000.0)
    s.setDecimals(1)
    s.setValue(50.0)
    return s


def _lbl(text: str, bold: bool = False) -> QLabel:
    lbl = QLabel(text)
    f = lbl.font()
    f.setBold(bold)
    lbl.setFont(f)
    return lbl
