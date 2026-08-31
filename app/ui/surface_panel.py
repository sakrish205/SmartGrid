"""Region shortcut checkboxes for bbox face selection."""
from __future__ import annotations

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton,
    QCheckBox, QLabel,
)
from PySide6.QtCore import Signal

REGIONS = ['TOP', 'BOTTOM', 'FRONT', 'REAR', 'LEFT', 'RIGHT']


class SurfacePanel(QGroupBox):
    region_shortcut_requested = Signal(str, bool)  # region_id, checked

    def __init__(self, parent=None) -> None:
        super().__init__("Surface Selection", parent)
        self._setup_ui()
        self.set_enabled(False)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        layout.addWidget(QLabel("Quick select:"))
        self._checkboxes: dict[str, QCheckBox] = {}
        for name in REGIONS:
            cb = QCheckBox(name)
            self._checkboxes[name] = cb
            cb.stateChanged.connect(
                lambda state, n=name: self.region_shortcut_requested.emit(n, bool(state))
            )
            layout.addWidget(cb)

        row = QHBoxLayout()
        all_btn  = QPushButton("All")
        none_btn = QPushButton("None")
        all_btn.clicked.connect(self._select_all)
        none_btn.clicked.connect(self._clear_all)
        row.addWidget(all_btn)
        row.addWidget(none_btn)
        layout.addLayout(row)

        layout.addStretch()

    def _select_all(self) -> None:
        for cb in self._checkboxes.values():
            cb.blockSignals(True)
            cb.setChecked(True)
            cb.blockSignals(False)
        for name in REGIONS:
            self.region_shortcut_requested.emit(name, True)

    def _clear_all(self) -> None:
        for cb in self._checkboxes.values():
            cb.blockSignals(True)
            cb.setChecked(False)
            cb.blockSignals(False)
        for name in REGIONS:
            self.region_shortcut_requested.emit(name, False)

    def toggle_region_checkbox(self, region: str, checked: bool) -> None:
        cb = self._checkboxes.get(region)
        if cb is None:
            return
        cb.blockSignals(True)
        cb.setChecked(checked)
        cb.blockSignals(False)

    def set_enabled(self, enabled: bool) -> None:
        for cb in self._checkboxes.values():
            cb.setEnabled(enabled)
