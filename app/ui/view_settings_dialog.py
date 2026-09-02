"""View Settings dialog — colour pickers for all viewer elements."""
from __future__ import annotations

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QDialogButtonBox, QGroupBox, QGridLayout, QFrame,
)
from PySide6.QtGui import QColor
from PySide6.QtCore import Signal, Qt


# Default colours — all as hex strings
DEFAULTS: dict[str, str] = {
    'background':     '#e8e8e8',
    'mesh':           '#78909c',
    'bbox_wire':      '#0078D4',
    'pass_forward':   '#2196F3',
    'pass_reverse':   '#FF5722',
    'connector':      '#FF69B4',
    'grid':           '#546e7a',
    'face_highlight': '#FFD700',
}

_LABELS: dict[str, str] = {
    'background':     'Background',
    'mesh':           'Mesh surface',
    'bbox_wire':      'Bounding box',
    'pass_forward':   'Pass — forward',
    'pass_reverse':   'Pass — reverse',
    'connector':      'Connector',
    'grid':           'Grid lines',
    'face_highlight': 'Face highlight',
}


class ViewSettingsDialog(QDialog):
    """Modal dialog; call exec() and read .colors on accept."""

    colors_changed = Signal(dict)   # emitted live as user picks colours

    def __init__(self, current: dict[str, str], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("View Settings")
        self.setMinimumWidth(340)
        self._colors: dict[str, str] = dict(current)
        self._btns:   dict[str, QPushButton] = {}
        self._build_ui()

    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(12)
        outer.setContentsMargins(16, 16, 16, 16)

        group = QGroupBox("Element colours")
        grid = QGridLayout(group)
        grid.setSpacing(8)
        grid.setColumnStretch(0, 1)

        for row, (key, label) in enumerate(_LABELS.items()):
            lbl = QLabel(label)
            btn = self._make_swatch(key)
            self._btns[key] = btn
            grid.addWidget(lbl, row, 0)
            grid.addWidget(btn, row, 1)

        outer.addWidget(group)

        # Reset to defaults
        reset_btn = QPushButton("Reset to defaults")
        reset_btn.clicked.connect(self._reset_defaults)
        outer.addWidget(reset_btn)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        outer.addWidget(sep)

        bb = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel
        )
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self._on_cancel)
        outer.addWidget(bb)

        self._original = dict(self._colors)

    # ------------------------------------------------------------------

    def _make_swatch(self, key: str) -> QPushButton:
        btn = QPushButton()
        btn.setFixedSize(64, 24)
        btn.setFlat(False)
        self._apply_swatch_style(btn, self._colors[key])
        btn.clicked.connect(lambda _, k=key: self._pick(k))
        return btn

    def _apply_swatch_style(self, btn: QPushButton, hex_color: str) -> None:
        qc  = QColor(hex_color)
        lum = 0.299 * qc.red() + 0.587 * qc.green() + 0.114 * qc.blue()
        text = '#000000' if lum > 140 else '#ffffff'
        btn.setStyleSheet(
            f"QPushButton {{ background:{hex_color}; color:{text};"
            f" border:1px solid #a0a0a0; border-radius:3px; font-size:11px; }}"
            f"QPushButton:hover {{ border:2px solid #0078D4; }}"
        )
        btn.setText(hex_color.upper())

    def _pick(self, key: str) -> None:
        from PySide6.QtWidgets import QColorDialog
        dlg = QColorDialog(QColor(self._colors[key]), self)
        dlg.setOption(QColorDialog.ColorDialogOption.ShowAlphaChannel, False)
        if dlg.exec() == QColorDialog.DialogCode.Accepted:
            hex_color = dlg.currentColor().name()
            self._colors[key] = hex_color
            self._apply_swatch_style(self._btns[key], hex_color)
            self.colors_changed.emit(dict(self._colors))

    def _reset_defaults(self) -> None:
        self._colors = dict(DEFAULTS)
        for key, btn in self._btns.items():
            self._apply_swatch_style(btn, self._colors[key])
        self.colors_changed.emit(dict(self._colors))

    def _on_cancel(self) -> None:
        # Restore original colours live
        self._colors = dict(self._original)
        self.colors_changed.emit(dict(self._colors))
        self.reject()

    # ------------------------------------------------------------------

    @property
    def colors(self) -> dict[str, str]:
        return dict(self._colors)
