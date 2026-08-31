"""Selection controls: face-pick buttons + optional region shortcuts."""
from __future__ import annotations
from typing import Callable, Optional

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton,
    QCheckBox, QLabel, QFrame,
)
from PySide6.QtCore import Signal

REGIONS = ['TOP', 'BOTTOM', 'FRONT', 'REAR', 'LEFT', 'RIGHT']


class SurfacePanel(QGroupBox):
    """Left-panel section for selecting mesh faces."""

    # Emitted when the user wants to activate a picking mode
    pick_mode_requested = Signal(str)   # 'single' | 'flood' | 'clear'
    # Emitted when a region shortcut checkbox changes
    region_shortcut_requested = Signal(str, bool)  # region_id, checked

    def __init__(self, parent=None) -> None:
        super().__init__("Surface Selection", parent)
        self._setup_ui()
        self.set_enabled(False)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        # ── Primary picking buttons ──────────────────────────────────
        self._pick_btn = QPushButton("Pick Faces")
        self._pick_btn.setToolTip("Click individual triangles to select them")
        self._pick_btn.setCheckable(True)

        self._clear_btn = QPushButton("Clear Selection")

        layout.addWidget(self._pick_btn)
        layout.addWidget(self._clear_btn)

        # Divider
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        layout.addWidget(line)

        # ── Region shortcut checkboxes ───────────────────────────────
        layout.addWidget(QLabel("Quick select:"))
        self._checkboxes: dict[str, QCheckBox] = {}
        grid_layout = QVBoxLayout()
        grid_layout.setSpacing(2)
        for name in REGIONS:
            cb = QCheckBox(name)
            self._checkboxes[name] = cb
            cb.stateChanged.connect(
                lambda state, n=name: self.region_shortcut_requested.emit(n, bool(state))
            )
            grid_layout.addWidget(cb)
        layout.addLayout(grid_layout)

        # Select all / clear shortcuts row
        row = QHBoxLayout()
        all_btn = QPushButton("All")
        all_btn.clicked.connect(self._select_all_regions)
        none_btn = QPushButton("None")
        none_btn.clicked.connect(self._clear_all_regions)
        row.addWidget(all_btn)
        row.addWidget(none_btn)
        layout.addLayout(row)

        layout.addStretch()

        # Wire pick-mode buttons
        self._pick_btn.clicked.connect(lambda checked: self._on_pick_mode('single', checked))
        self._clear_btn.clicked.connect(lambda: self.pick_mode_requested.emit('clear'))

    def _on_pick_mode(self, mode: str, checked: bool) -> None:
        if checked:
            self.pick_mode_requested.emit(mode)
        else:
            self.pick_mode_requested.emit('none')

    def _select_all_regions(self) -> None:
        for cb in self._checkboxes.values():
            cb.blockSignals(True)
            cb.setChecked(True)
            cb.blockSignals(False)
        # Emit once for all
        for name in REGIONS:
            self.region_shortcut_requested.emit(name, True)

    def _clear_all_regions(self) -> None:
        for cb in self._checkboxes.values():
            cb.blockSignals(True)
            cb.setChecked(False)
            cb.blockSignals(False)
        for name in REGIONS:
            self.region_shortcut_requested.emit(name, False)

    def toggle_region_checkbox(self, region: str, checked: bool) -> None:
        """Sync checkbox state from an external event without emitting a signal."""
        cb = self._checkboxes.get(region)
        if cb is None:
            return
        cb.blockSignals(True)
        cb.setChecked(checked)
        cb.blockSignals(False)

    def reset_pick_buttons(self) -> None:
        """Deselect Pick Faces button without emitting signals."""
        self._pick_btn.blockSignals(True)
        self._pick_btn.setChecked(False)
        self._pick_btn.blockSignals(False)

    def set_enabled(self, enabled: bool) -> None:
        self._pick_btn.setEnabled(enabled)
        self._clear_btn.setEnabled(enabled)
        for cb in self._checkboxes.values():
            cb.setEnabled(enabled)
