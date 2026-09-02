"""Unit dropdown, spray width, direction selector, path-target toggle, Generate/Clear buttons."""
from __future__ import annotations

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QFormLayout, QComboBox,
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
    grid_changed       = Signal()

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
        form.setSpacing(6)
        form.setLabelAlignment(Qt.AlignRight)

        self._unit_combo = QComboBox()
        self._unit_combo.addItems(UNITS)
        self._unit_combo.setCurrentText('mm')
        self._unit_combo.currentTextChanged.connect(self._on_unit_changed)
        form.addRow(_lbl("Unit:"), self._unit_combo)

        # ── Spray width ──────────────────────────────────────────────────
        self._h_spin = _spin()
        form.addRow(_lbl("Pitch (mm):"), self._h_spin)
        layout.addLayout(form)

        # ── Direction ────────────────────────────────────────────────────
        layout.addWidget(_lbl("Direction:", bold=True))

        dir_group = QButtonGroup(self)
        self._h_radio  = QRadioButton("Horizontal")
        self._v_radio  = QRadioButton("Vertical")
        self._hv_radio = QRadioButton("Both (crosshatch)")
        self._h_radio.setChecked(True)
        dir_group.addButton(self._h_radio,  0)
        dir_group.addButton(self._v_radio,  1)
        dir_group.addButton(self._hv_radio, 2)
        layout.addWidget(self._h_radio)
        layout.addWidget(self._v_radio)
        layout.addWidget(self._hv_radio)

        # V width spinbox — only visible when "Both" is selected
        self._v_label = _lbl("V width:")
        self._v_spin  = _spin()
        layout.addWidget(self._v_label)
        layout.addWidget(self._v_spin)
        self._v_label.hide()
        self._v_spin.hide()

        dir_group.idClicked.connect(self._on_direction_changed)

        # ── Show Grid ────────────────────────────────────────────────────
        self._show_grid_check = QCheckBox("Show grid")
        self._show_grid_check.toggled.connect(lambda _: self.grid_changed.emit())
        layout.addWidget(self._show_grid_check)

        # Emit grid_changed when widths change
        self._h_spin.valueChanged.connect(lambda _: self.grid_changed.emit())
        self._v_spin.valueChanged.connect(lambda _: self.grid_changed.emit())

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

        # ── Generate button ──────────────────────────────────────────────
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

        # ── Clear Path button ────────────────────────────────────────────
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

        self._h_spin.setSuffix('  mm')
        self._v_spin.setSuffix('  mm')

    # ------------------------------------------------------------------
    def _on_direction_changed(self, btn_id: int) -> None:
        both = (btn_id == 2)
        self._v_label.setVisible(both)
        self._v_spin.setVisible(both)
        self.grid_changed.emit()

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

    def get_v_width_mm(self) -> float | None:
        """Return vertical width only when Both is selected."""
        if self._hv_radio.isChecked():
            return self._v_spin.value() * UNIT_TO_MM[self._current_unit]
        return None

    def get_direction(self) -> str:
        if self._v_radio.isChecked():
            return 'vertical'
        if self._hv_radio.isChecked():
            return 'both'
        return 'horizontal'

    def get_path_target(self) -> str:
        return 'bbox' if self._bbox_target_radio.isChecked() else 'mesh'

    def is_show_grid(self) -> bool:
        return self._show_grid_check.isChecked()

    @property
    def current_unit(self) -> str:
        return self._current_unit

    def set_enabled(self, enabled: bool) -> None:
        for w in (self._unit_combo, self._h_spin, self._v_spin,
                  self._h_radio, self._v_radio, self._hv_radio,
                  self._show_grid_check,
                  self._bbox_target_radio, self._mesh_target_radio,
                  self._gen_btn, self._clear_btn):
            w.setEnabled(enabled)

    def set_generating(self, generating: bool) -> None:
        self._gen_btn.setEnabled(not generating)
        self._clear_btn.setEnabled(not generating)
        self._gen_btn.setText("Generating…" if generating else "GENERATE PATH")


# ------------------------------------------------------------------
def _spin() -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(0.1, 100_000.0)
    s.setDecimals(1)
    s.setValue(100.0)
    return s


def _lbl(text: str, bold: bool = False) -> QLabel:
    lbl = QLabel(text)
    if bold:
        lbl.setStyleSheet("font-weight: bold;")
    return lbl
