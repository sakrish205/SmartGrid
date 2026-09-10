"""ViewCube — clickable isometric cube widget that drives viewer camera."""
from __future__ import annotations

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPolygon, QColor, QFont, QPen
from PySide6.QtCore import Qt, QPoint, Signal


_FACES = {
    'top':   {'label': 'TOP',   'color': QColor(210, 210, 210), 'hover': QColor(240, 240, 255)},
    'front': {'label': 'FRONT', 'color': QColor(160, 160, 160), 'hover': QColor(180, 180, 220)},
    'right': {'label': 'RIGHT', 'color': QColor(110, 110, 110), 'hover': QColor(130, 130, 190)},
}


def _poly(pts: list[tuple[int, int]]) -> QPolygon:
    return QPolygon([QPoint(x, y) for x, y in pts])


def _centroid(pts: list[tuple[int, int]]) -> tuple[float, float]:
    return sum(x for x, _ in pts) / len(pts), sum(y for _, y in pts) / len(pts)


class ViewCube(QWidget):
    """Isometric cube: click TOP / FRONT / RIGHT to snap the 3-D camera."""

    view_changed = Signal(str)   # emits 'top' | 'front' | 'right'

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedSize(120, 120)
        self.setMouseTracking(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self._hovered: str | None = None
        self._polys:  dict[str, QPolygon] = {}

    def _build_polys(self) -> dict[str, QPolygon]:
        cx, cy, s, d = 60, 62, 28, 16   # centre, half-side, isometric depth
        return {
            'top':   _poly([(cx - s, cy - s), (cx, cy - s - d),
                             (cx + s, cy - s), (cx, cy - s + d)]),
            'front': _poly([(cx - s, cy - s), (cx, cy - s + d),
                             (cx, cy + s + d), (cx - s, cy + s)]),
            'right': _poly([(cx, cy - s + d), (cx + s, cy - s),
                             (cx + s, cy + s), (cx, cy + s + d)]),
        }

    def _face_at(self, pos: QPoint) -> str | None:
        for name, poly in self._polys.items():
            if poly.containsPoint(pos, Qt.FillRule.OddEvenFill):
                return name
        return None

    def mouseMoveEvent(self, event) -> None:
        hit = self._face_at(event.pos())
        if hit != self._hovered:
            self._hovered = hit
            self.update()

    def leaveEvent(self, event) -> None:
        self._hovered = None
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            hit = self._face_at(event.pos())
            if hit:
                self.view_changed.emit(hit)

    def paintEvent(self, event) -> None:
        self._polys = self._build_polys()
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(QPen(QColor(60, 60, 60), 1))

        for name, poly in self._polys.items():
            cfg = _FACES[name]
            color = cfg['hover'] if self._hovered == name else cfg['color']
            p.setBrush(color)
            p.drawPolygon(poly)

        p.setFont(QFont('Segoe UI', 7, QFont.Weight.Bold))
        cx, cy, s, d = 60, 62, 28, 16

        labels = {
            'top':   (_centroid([(cx-s, cy-s), (cx, cy-s-d), (cx+s, cy-s), (cx, cy-s+d)]), 'TOP'),
            'front': (_centroid([(cx-s, cy-s), (cx, cy-s+d), (cx, cy+s+d), (cx-s, cy+s)]), 'FRONT'),
            'right': (_centroid([(cx, cy-s+d), (cx+s, cy-s), (cx+s, cy+s), (cx, cy+s+d)]), 'RIGHT'),
        }
        for name, ((lx, ly), text) in labels.items():
            color = QColor(255, 255, 255) if self._hovered == name else QColor(30, 30, 30)
            p.setPen(color)
            fm = p.fontMetrics()
            p.drawText(int(lx - fm.horizontalAdvance(text) / 2), int(ly + fm.ascent() / 2 - 1), text)

        p.end()
