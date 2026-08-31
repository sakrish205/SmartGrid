"""Displays pass count, total length, and mesh stats after generation."""
from __future__ import annotations
from typing import Optional

from PySide6.QtWidgets import QGroupBox, QFormLayout, QLabel

from app.path.path_model import PaintRoute, UNIT_TO_MM


class StatusPanel(QGroupBox):
    def __init__(self, parent=None) -> None:
        super().__init__("Statistics", parent)
        self._setup_ui()

    def _setup_ui(self) -> None:
        form = QFormLayout(self)
        form.setSpacing(6)

        self._faces_lbl  = QLabel("—")
        self._passes_lbl = QLabel("—")
        self._conns_lbl  = QLabel("—")
        self._length_lbl = QLabel("—")
        self._spacing_lbl = QLabel("—")

        form.addRow("Triangles:",    self._faces_lbl)
        form.addRow("Paint passes:", self._passes_lbl)
        form.addRow("Connections:",  self._conns_lbl)
        form.addRow("Total length:", self._length_lbl)
        form.addRow("Spacing:",      self._spacing_lbl)

    def update_mesh_stats(self, n_faces: int) -> None:
        self._faces_lbl.setText(f"{n_faces:,}")

    def update_route_stats(self, routes: list[PaintRoute], display_unit: str) -> None:
        total_passes = sum(r.total_passes for r in routes)
        total_conns  = sum(len(r.connections) for r in routes)
        total_mm     = sum(r.total_length_mm for r in routes)

        # Convert length to display unit
        factor = UNIT_TO_MM.get(display_unit, 1.0)
        length_display = total_mm / factor

        unit_label = {'mm': 'mm', 'cm': 'cm', 'm': 'm', 'in': 'in', 'ft': 'ft'}.get(display_unit, display_unit)
        spacing_mm = routes[0].spacing_mm if routes else 0.0
        spacing_display = spacing_mm / factor

        self._passes_lbl.setText(str(total_passes))
        self._conns_lbl.setText(str(total_conns))
        self._length_lbl.setText(f"{length_display:.2f} {unit_label}")
        self._spacing_lbl.setText(f"{spacing_display:.2f} {unit_label}")

    def clear(self) -> None:
        for lbl in (self._passes_lbl, self._conns_lbl, self._length_lbl, self._spacing_lbl):
            lbl.setText("—")
