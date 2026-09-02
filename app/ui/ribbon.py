"""SmartGrid — single-strip professional Ribbon bar."""
from __future__ import annotations
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QLabel, QPushButton, QToolButton, QComboBox, QDoubleSpinBox,
    QCheckBox, QRadioButton, QButtonGroup, QSizePolicy, QFrame,
)
from PySide6.QtCore import Signal, Qt, QSize, QByteArray
from PySide6.QtGui import QIcon, QPixmap, QPainter, QFont
from PySide6.QtSvg import QSvgRenderer

from app.path.path_model import UNIT_TO_MM, PaintRoute

_UNITS   = ['mm', 'cm', 'm', 'in', 'ft']
_REGIONS = ['TOP', 'BOTTOM', 'FRONT', 'REAR', 'LEFT', 'RIGHT']

RIBBON_H = 88   # total ribbon height in pixels

# ---------------------------------------------------------------------------
# Minimal monochrome SVG icons  (20 × 20 viewBox)
# ---------------------------------------------------------------------------
_SVG: dict[str, bytes] = {
    'open': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M2 6.5h5.8L9.3 8H18v9H2V6.5zm1 1.5v7h14V9.5H8.7L7.2 8H3z"'
        b' fill="#3c3c3c"/>'
        b'<path d="M3 4h5v1.5H3V4z" fill="#3c3c3c"/>'
        b'</svg>'
    ),
    'fit': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M2 2h5v2H4v3H2V2zm11 0h5v5h-2V4h-3V2z'
        b'M2 13h2v3h3v2H2v-5zm13 3h-3v2h5v-5h-2v3z" fill="#3c3c3c"/>'
        b'</svg>'
    ),
    'generate': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M4 2h12v16H4V2zm1 1.5v13h10v-13H5z" fill="#3c3c3c"/>'
        b'<path d="M7 6.5h6v1H7v-1zm0 3h6v1H7v-1zm0 3h4v1H7v-1z" fill="#3c3c3c"/>'
        b'</svg>'
    ),
    'clear': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M7 1.5h6v2h5v1.5H2V3.5h5V1.5zm-2 5h10l-1 12H6L5 6.5z'
        b'M8.5 8.5v7h1v-7h-1zm3 0v7h1v-7h-1z" fill="#3c3c3c"/>'
        b'</svg>'
    ),
    'export': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M10 2v11l-3-3-1 1 4 4 4-4-1-1-3 3V2h-2z" fill="#3c3c3c"/>'
        b'<path d="M3 16h14v1.5H3V16z" fill="#3c3c3c"/>'
        b'</svg>'
    ),
    'top': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M10 4L3 8l7 3.5L17 8 10 4zm0 1.8L14.2 8 10 10.1 5.8 8 10 5.8z"'
        b' fill="#3c3c3c"/>'
        b'<path d="M4.5 11.5l-1.5 1L10 16l7-3.5-1.5-1L10 14.5 4.5 11.5z"'
        b' fill="#aaa"/>'
        b'</svg>'
    ),
    'bottom': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M10 16L3 12l7-3.5 7 3.5L10 16zm0-1.8L5.8 12 10 9.9l4.2 2.1L10 14.2z"'
        b' fill="#3c3c3c"/>'
        b'<path d="M4.5 8.5l-1.5 1L10 13l7-3.5-1.5-1L10 11.5 4.5 8.5z"'
        b' fill="#aaa"/>'
        b'</svg>'
    ),
    'front': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<rect x="3" y="3" width="14" height="14" fill="none"'
        b' stroke="#3c3c3c" stroke-width="1.5"/>'
        b'<line x1="10" y1="3" x2="10" y2="17" stroke="#aaa" stroke-width="1"/>'
        b'<line x1="3" y1="10" x2="17" y2="10" stroke="#aaa" stroke-width="1"/>'
        b'</svg>'
    ),
    'rear': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<rect x="3" y="3" width="14" height="14" fill="none"'
        b' stroke="#3c3c3c" stroke-width="1.5" stroke-dasharray="3 2"/>'
        b'<line x1="10" y1="3" x2="10" y2="17" stroke="#aaa" stroke-width="1"/>'
        b'<line x1="3" y1="10" x2="17" y2="10" stroke="#aaa" stroke-width="1"/>'
        b'</svg>'
    ),
    'left': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M13 3H7v14h6V3zm-5 1h4v12H8V4z" fill="#3c3c3c"/>'
        b'<path d="M7 3L4 5v10l3 2V3z" fill="#3c3c3c" opacity="0.5"/>'
        b'</svg>'
    ),
    'right': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M7 3h6v14H7V3zm1 1v12h4V4H8z" fill="#3c3c3c"/>'
        b'<path d="M13 3l3 2v10l-3 2V3z" fill="#3c3c3c" opacity="0.5"/>'
        b'</svg>'
    ),
    'settings': (
        b'<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">'
        b'<path d="M8.1 1.5H12l.6 2.3c.5.2 1 .5 1.4.9l2.3-.8 1.9 3.3-1.8 1.5'
        b'c.1.3.1.6.1.8 0 .3 0 .6-.1.9l1.8 1.5-1.9 3.3-2.3-.8c-.4.4-.9.7-1.4.9'
        b'L12 17.5H8.1l-.6-2.3c-.5-.2-1-.5-1.4-.9l-2.3.8-1.9-3.3L3.7 10.3'
        b'C3.6 10 3.5 9.7 3.5 9.4c0-.3 0-.6.1-.9L1.8 7l1.9-3.3 2.3.8'
        b'c.4-.4.9-.7 1.4-.9L8 1.5zm1.9 5.5a2.5 2.5 0 100 5 2.5 2.5 0 000-5z"'
        b' fill="#3c3c3c"/>'
        b'</svg>'
    ),
}


def _make_icon(name: str, size: int = 18) -> QIcon:
    data = _SVG.get(name, _SVG['settings'])
    renderer = QSvgRenderer(QByteArray(data))
    pix = QPixmap(size, size)
    pix.fill(Qt.GlobalColor.transparent)
    p = QPainter(pix)
    renderer.render(p)
    p.end()
    return QIcon(pix)


# ---------------------------------------------------------------------------
# Style constants
# ---------------------------------------------------------------------------
_RIBBON_BG = '#f0f0f0'
_SEP_COLOR  = '#c8c8c8'
_GRP_LABEL_CSS = 'color:#666;font-size:10px;font-family:"Segoe UI",Arial;'

_BTN_CSS = (
    'QToolButton{'
    '  background:transparent;border:1px solid transparent;border-radius:2px;'
    '  padding:2px 4px;font-size:11px;font-family:"Segoe UI",Arial;color:#1f1f1f;'
    '}'
    'QToolButton:hover{background:#e0e0e0;border-color:#c0c0c0;}'
    'QToolButton:checked{background:#cde8ff;border-color:#0078d4;}'
    'QToolButton:pressed{background:#b8d8f8;border-color:#0078d4;}'
    'QToolButton:disabled{color:#a0a0a0;}'
)

_LARGE_BTN_CSS = (
    'QToolButton{'
    '  background:transparent;border:1px solid transparent;border-radius:2px;'
    '  padding:3px;font-size:11px;font-family:"Segoe UI",Arial;color:#1f1f1f;'
    '  min-width:46px;'
    '}'
    'QToolButton:hover{background:#e0e0e0;border-color:#c0c0c0;}'
    'QToolButton:pressed{background:#b8d8f8;border-color:#0078d4;}'
    'QToolButton:disabled{color:#a0a0a0;}'
)

_TOGGLE_CSS = (
    'QPushButton{'
    '  background:#fff;border:1px solid #ccc;border-radius:2px;'
    '  padding:2px 5px;font-size:11px;font-family:"Segoe UI",Arial;color:#1f1f1f;'
    '  min-width:32px;min-height:20px;'
    '}'
    'QPushButton:hover{background:#e8e8e8;border-color:#999;}'
    'QPushButton:checked{background:#cde8ff;border-color:#0078d4;color:#003e7e;}'
    'QPushButton:disabled{background:#f4f4f4;color:#aaa;}'
)

_PRIMARY_BTN_CSS = (
    'QToolButton{'
    '  background:#0078d4;border:1px solid #005a9e;border-radius:2px;'
    '  padding:3px;font-size:11px;font-family:"Segoe UI",Arial;color:#fff;'
    '  font-weight:600;min-width:78px;'
    '}'
    'QToolButton:hover{background:#106ebe;}'
    'QToolButton:pressed{background:#005a9e;}'
    'QToolButton:disabled{background:#e4e4e4;border-color:#ccc;color:#aaa;}'
)

_DANGER_BTN_CSS = (
    'QToolButton{'
    '  background:#fff;border:1px solid #d9534f;border-radius:2px;'
    '  padding:3px;font-size:11px;font-family:"Segoe UI",Arial;color:#c9302c;'
    '  min-width:78px;'
    '}'
    'QToolButton:hover{background:#fdf0f0;}'
    'QToolButton:pressed{background:#f5c6c6;}'
    'QToolButton:disabled{background:#f4f4f4;border-color:#ccc;color:#aaa;}'
)

_COMBO_CSS = (
    'QComboBox{background:#fff;border:1px solid #ccc;border-radius:2px;'
    '  padding:2px 4px;font-size:11px;font-family:"Segoe UI",Arial;'
    '  color:#1f1f1f;min-height:22px;}'
    'QComboBox:focus{border-color:#0078d4;}'
    'QComboBox::drop-down{border:none;width:16px;}'
    'QComboBox QAbstractItemView{background:#fff;color:#1f1f1f;'
    '  selection-background-color:#cde8ff;selection-color:#003e7e;}'
)

_SPIN_CSS = (
    'QDoubleSpinBox{background:#fff;border:1px solid #ccc;border-radius:2px;'
    '  padding:2px 4px;font-size:11px;font-family:"Segoe UI",Arial;'
    '  color:#1f1f1f;min-height:22px;}'
    'QDoubleSpinBox:focus{border-color:#0078d4;}'
    'QDoubleSpinBox::up-button,QDoubleSpinBox::down-button'
    '{background:#f0f0f0;border:none;width:14px;}'
)

_CHK_CSS = (
    'QCheckBox{font-size:11px;font-family:"Segoe UI",Arial;color:#1f1f1f;spacing:4px;}'
    'QCheckBox::indicator{width:13px;height:13px;border:1px solid #888;'
    '  border-radius:2px;background:#fff;}'
    'QCheckBox::indicator:checked{background:#0078d4;border-color:#0078d4;}'
)

_RADIO_CSS = (
    'QRadioButton{font-size:11px;font-family:"Segoe UI",Arial;color:#1f1f1f;spacing:4px;}'
    'QRadioButton::indicator{width:13px;height:13px;border:1px solid #888;'
    '  border-radius:7px;background:#fff;}'
    'QRadioButton::indicator:checked{background:#0078d4;border-color:#0078d4;}'
)

_STAT_LABEL_CSS = 'font-size:11px;font-family:"Segoe UI",Arial;color:#333;'
_STAT_VALUE_CSS = 'font-size:11px;font-family:"Segoe UI",Arial;color:#1f1f1f;font-weight:600;'


# ---------------------------------------------------------------------------
# Ribbon group widget
# ---------------------------------------------------------------------------

class _Group(QWidget):
    def __init__(self, title: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        vl = QVBoxLayout(self)
        vl.setContentsMargins(4, 2, 4, 0)
        vl.setSpacing(0)

        self._content = QWidget()
        self._content.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        self._hl = QHBoxLayout(self._content)
        self._hl.setContentsMargins(0, 0, 0, 0)
        self._hl.setSpacing(3)
        self._hl.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)

        sep_line = QFrame()
        sep_line.setFrameShape(QFrame.Shape.HLine)
        sep_line.setFixedHeight(1)
        sep_line.setStyleSheet('color:#d0d0d0;')

        lbl = QLabel(title)
        lbl.setFixedHeight(16)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(_GRP_LABEL_CSS)

        vl.addWidget(self._content, 1)
        vl.addWidget(sep_line)
        vl.addWidget(lbl)

    def add(self, w: QWidget) -> None:
        self._hl.addWidget(w)

    def add_layout(self, layout) -> None:
        self._hl.addLayout(layout)

    def add_spacing(self, px: int) -> None:
        self._hl.addSpacing(px)


class _VSep(QFrame):
    """Vertical separator between groups."""
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.VLine)
        self.setFixedWidth(1)
        self.setStyleSheet(f'color:{_SEP_COLOR};')


# ---------------------------------------------------------------------------
# Button / control factories
# ---------------------------------------------------------------------------

def _large_btn(label: str, icon: QIcon) -> QToolButton:
    """Icon above text, full ribbon height, primary style."""
    b = QToolButton()
    b.setIcon(icon)
    b.setIconSize(QSize(20, 20))
    b.setText(label)
    b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
    b.setStyleSheet(_LARGE_BTN_CSS)
    b.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
    return b


def _primary_btn(label: str, icon: QIcon) -> QToolButton:
    b = QToolButton()
    b.setIcon(icon)
    b.setIconSize(QSize(20, 20))
    b.setText(label)
    b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
    b.setStyleSheet(_PRIMARY_BTN_CSS)
    b.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
    return b


def _danger_btn(label: str, icon: QIcon) -> QToolButton:
    b = QToolButton()
    b.setIcon(icon)
    b.setIconSize(QSize(20, 20))
    b.setText(label)
    b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
    b.setStyleSheet(_DANGER_BTN_CSS)
    b.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
    return b


def _small_btn(label: str, icon: Optional[QIcon] = None) -> QToolButton:
    b = QToolButton()
    if icon:
        b.setIcon(icon)
        b.setIconSize(QSize(16, 16))
        b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
    else:
        b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
    b.setText(label)
    b.setStyleSheet(_BTN_CSS)
    return b


def _toggle_btn(label: str) -> QPushButton:
    b = QPushButton(label)
    b.setCheckable(True)
    b.setStyleSheet(_TOGGLE_CSS)
    return b


def _row_label(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet('font-size:10px;font-family:"Segoe UI",Arial;color:#555;')
    return lbl


# ---------------------------------------------------------------------------
# Main Ribbon widget
# ---------------------------------------------------------------------------

class SmartRibbon(QWidget):
    # ── signals ──────────────────────────────────────────────────────────
    open_requested      = Signal()
    view_fit            = Signal()
    view_set            = Signal(str)   # direction string
    grid_changed        = Signal()
    arrows_changed      = Signal()
    region_toggled      = Signal(str, bool)
    select_mode_changed = Signal(bool)
    generate_requested  = Signal()
    clear_requested     = Signal()
    sweep_changed       = Signal()
    export_json         = Signal()
    export_csv          = Signal()
    view_settings_req   = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFixedHeight(RIBBON_H)
        self.setStyleSheet(
            f'SmartRibbon{{background:{_RIBBON_BG};'
            f'border-bottom:1px solid {_SEP_COLOR};}}'
        )
        self._current_unit = 'mm'
        self._region_btns: dict[str, QPushButton] = {}
        self._build()
        self._connect_internal()

    # ------------------------------------------------------------------
    def _build(self) -> None:
        hl = QHBoxLayout(self)
        hl.setContentsMargins(0, 0, 0, 0)
        hl.setSpacing(0)

        groups = [
            self._build_file(),
            self._build_view(),
            self._build_surface(),
            self._build_path_settings(),
            self._build_path(),
            self._build_stats(),
            self._build_export(),
        ]

        for g in groups:
            hl.addWidget(_VSep())
            hl.addWidget(g)

        hl.addWidget(_VSep())
        hl.addStretch(1)

    # ── File ─────────────────────────────────────────────────────────────
    def _build_file(self) -> _Group:
        g = _Group('File')
        self._open_btn = _large_btn('Open', _make_icon('open', 20))
        g.add(self._open_btn)
        return g

    # ── View ─────────────────────────────────────────────────────────────
    def _build_view(self) -> _Group:
        g = _Group('View')

        vl = QVBoxLayout()
        vl.setContentsMargins(0, 0, 0, 0)
        vl.setSpacing(2)

        # Row 1 — camera presets
        row1 = QHBoxLayout()
        row1.setSpacing(2)
        row1.setContentsMargins(0, 0, 0, 0)

        self._fit_btn    = _small_btn('Fit All', _make_icon('fit', 16))
        self._top_btn    = _small_btn('Top',     _make_icon('top', 16))
        self._bot_btn    = _small_btn('Bottom',  _make_icon('bottom', 16))
        self._front_btn  = _small_btn('Front',   _make_icon('front', 16))
        self._rear_btn   = _small_btn('Rear',    _make_icon('rear', 16))
        self._left_btn   = _small_btn('Left',    _make_icon('left', 16))
        self._right_btn  = _small_btn('Right',   _make_icon('right', 16))

        for b in (self._fit_btn, self._top_btn, self._bot_btn,
                  self._front_btn, self._rear_btn, self._left_btn, self._right_btn):
            b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
            row1.addWidget(b)

        # Row 2 — overlays + settings
        row2 = QHBoxLayout()
        row2.setSpacing(6)
        row2.setContentsMargins(0, 0, 0, 0)

        self._grid_check   = QCheckBox('Grid')
        self._arrows_check = QCheckBox('Arrows')
        self._grid_check.setStyleSheet(_CHK_CSS)
        self._arrows_check.setStyleSheet(_CHK_CSS)

        vs_btn = _small_btn('Settings', _make_icon('settings', 16))
        vs_btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        vs_btn.clicked.connect(self.view_settings_req)

        row2.addWidget(self._grid_check)
        row2.addWidget(self._arrows_check)
        row2.addSpacing(4)
        row2.addWidget(vs_btn)

        vl.addLayout(row1)
        vl.addLayout(row2)
        g.add_layout(vl)
        return g

    # ── Surface Selection ─────────────────────────────────────────────────
    def _build_surface(self) -> _Group:
        g = _Group('Surface Selection')

        vl = QVBoxLayout()
        vl.setContentsMargins(0, 0, 0, 0)
        vl.setSpacing(2)

        # Row 1 — region toggle buttons
        row1 = QHBoxLayout()
        row1.setSpacing(2)
        row1.setContentsMargins(0, 0, 0, 0)
        for region in _REGIONS:
            short = {'BOTTOM': 'BOT', 'FRONT': 'FRT', 'REAR': 'REAR',
                     'LEFT': 'LEFT', 'RIGHT': 'RGT'}.get(region, region)
            btn = _toggle_btn(short)
            btn.setToolTip(region)
            self._region_btns[region] = btn
            row1.addWidget(btn)

        # Row 2 — All / None / Select Faces
        row2 = QHBoxLayout()
        row2.setSpacing(4)
        row2.setContentsMargins(0, 0, 0, 0)

        self._all_btn  = _small_btn('All')
        self._none_btn = _small_btn('None')
        self._all_btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
        self._none_btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)

        self._select_btn = QPushButton('Select Faces')
        self._select_btn.setCheckable(True)
        self._select_btn.setStyleSheet(
            'QPushButton{background:#fff;border:1px solid #ccc;border-radius:2px;'
            '  padding:2px 6px;font-size:11px;font-family:"Segoe UI",Arial;color:#1f1f1f;'
            '  min-height:20px;}'
            'QPushButton:hover{background:#e8e8e8;}'
            'QPushButton:checked{background:#cde8ff;border-color:#0078d4;color:#003e7e;}'
        )

        row2.addWidget(self._all_btn)
        row2.addWidget(self._none_btn)
        row2.addSpacing(4)
        row2.addWidget(self._select_btn)

        vl.addLayout(row1)
        vl.addLayout(row2)
        g.add_layout(vl)
        return g

    # ── Path Settings ─────────────────────────────────────────────────────
    def _build_path_settings(self) -> _Group:
        g = _Group('Path Settings')

        hl = QHBoxLayout()
        hl.setSpacing(8)
        hl.setContentsMargins(0, 0, 0, 0)

        # Unit + Pitch
        form_vl = QVBoxLayout()
        form_vl.setSpacing(3)
        form_vl.setContentsMargins(0, 0, 0, 0)

        unit_hl = QHBoxLayout()
        unit_hl.setSpacing(3)
        unit_hl.addWidget(_row_label('Unit'))
        self._unit_combo = QComboBox()
        self._unit_combo.addItems(_UNITS)
        self._unit_combo.setCurrentText('mm')
        self._unit_combo.setFixedWidth(58)
        self._unit_combo.setStyleSheet(_COMBO_CSS)
        unit_hl.addWidget(self._unit_combo)

        pitch_hl = QHBoxLayout()
        pitch_hl.setSpacing(3)
        pitch_hl.addWidget(_row_label('Pitch'))
        self._pitch_spin = QDoubleSpinBox()
        self._pitch_spin.setRange(0.1, 100_000.0)
        self._pitch_spin.setDecimals(1)
        self._pitch_spin.setValue(100.0)
        self._pitch_spin.setSuffix('  mm')
        self._pitch_spin.setFixedWidth(90)
        self._pitch_spin.setStyleSheet(_SPIN_CSS)
        pitch_hl.addWidget(self._pitch_spin)

        form_vl.addLayout(unit_hl)
        form_vl.addLayout(pitch_hl)
        hl.addLayout(form_vl)

        hl.addWidget(_mk_vsep())

        # Sweep direction
        sweep_vl = QVBoxLayout()
        sweep_vl.setSpacing(2)
        sweep_vl.setContentsMargins(0, 0, 0, 0)
        sweep_vl.addWidget(_row_label('Sweep'))
        self._cw_radio  = QRadioButton('CW')
        self._ccw_radio = QRadioButton('CCW')
        self._cw_radio.setChecked(True)
        self._cw_radio.setStyleSheet(_RADIO_CSS)
        self._ccw_radio.setStyleSheet(_RADIO_CSS)
        sweep_grp = QButtonGroup(self)
        sweep_grp.addButton(self._cw_radio,  0)
        sweep_grp.addButton(self._ccw_radio, 1)
        sweep_vl.addWidget(self._cw_radio)
        sweep_vl.addWidget(self._ccw_radio)
        hl.addLayout(sweep_vl)

        hl.addWidget(_mk_vsep())

        # Path target
        target_vl = QVBoxLayout()
        target_vl.setSpacing(2)
        target_vl.setContentsMargins(0, 0, 0, 0)
        target_vl.addWidget(_row_label('Path on'))
        self._bbox_radio = QRadioButton('Boundary Box')
        self._mesh_radio = QRadioButton('Mesh Surface')
        self._bbox_radio.setChecked(True)
        self._bbox_radio.setStyleSheet(_RADIO_CSS)
        self._mesh_radio.setStyleSheet(_RADIO_CSS)
        target_grp = QButtonGroup(self)
        target_grp.addButton(self._bbox_radio, 0)
        target_grp.addButton(self._mesh_radio, 1)
        target_vl.addWidget(self._bbox_radio)
        target_vl.addWidget(self._mesh_radio)
        hl.addLayout(target_vl)

        g.add_layout(hl)
        return g

    # ── Path ─────────────────────────────────────────────────────────────
    def _build_path(self) -> _Group:
        g = _Group('Path')
        self._gen_btn   = _primary_btn('Generate\nPath', _make_icon('generate', 20))
        self._clear_btn = _danger_btn('Clear\nPath',  _make_icon('clear', 20))
        g.add(self._gen_btn)
        g.add_spacing(2)
        g.add(self._clear_btn)
        return g

    # ── Statistics ────────────────────────────────────────────────────────
    def _build_stats(self) -> _Group:
        g = _Group('Statistics')

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(2)
        grid.setContentsMargins(0, 0, 0, 0)

        def _add_stat(row, col_lbl, col_val, label):
            lbl = QLabel(label)
            lbl.setStyleSheet(_STAT_LABEL_CSS)
            val = QLabel('—')
            val.setStyleSheet(_STAT_VALUE_CSS)
            grid.addWidget(lbl, row, col_lbl)
            grid.addWidget(val, row, col_val)
            return val

        self._tri_val    = _add_stat(0, 0, 1, 'Triangles')
        self._passes_val = _add_stat(1, 0, 1, 'Passes')
        self._conn_val   = _add_stat(0, 2, 3, 'Connections')
        self._len_val    = _add_stat(1, 2, 3, 'Length')
        self._spacing_val = _add_stat(0, 4, 5, 'Spacing')

        g.add_layout(grid)
        return g

    # ── Export ────────────────────────────────────────────────────────────
    def _build_export(self) -> _Group:
        g = _Group('Export')
        self._exp_json_btn = _large_btn('Export\nJSON', _make_icon('export', 20))
        self._exp_csv_btn  = _large_btn('Export\nCSV',  _make_icon('export', 20))
        g.add(self._exp_json_btn)
        g.add(self._exp_csv_btn)
        return g

    # ------------------------------------------------------------------
    # Internal signal connections
    # ------------------------------------------------------------------

    def _connect_internal(self) -> None:
        self._open_btn.clicked.connect(self.open_requested)
        self._fit_btn.clicked.connect(self.view_fit)

        for name, btn in [
            ('top', self._top_btn), ('bottom', self._bot_btn),
            ('front', self._front_btn), ('rear', self._rear_btn),
            ('left', self._left_btn), ('right', self._right_btn),
        ]:
            btn.clicked.connect(lambda _=False, n=name: self.view_set.emit(n))

        self._grid_check.toggled.connect(lambda _: self.grid_changed.emit())
        self._arrows_check.toggled.connect(lambda _: self.arrows_changed.emit())

        for region, btn in self._region_btns.items():
            btn.toggled.connect(
                lambda checked, r=region: self.region_toggled.emit(r, checked))

        self._all_btn.clicked.connect(self._select_all_regions)
        self._none_btn.clicked.connect(self._clear_all_regions)
        self._select_btn.toggled.connect(self.select_mode_changed)

        self._unit_combo.currentTextChanged.connect(self._on_unit_changed)
        self._pitch_spin.valueChanged.connect(lambda _: self.grid_changed.emit())

        self._cw_radio.toggled.connect(lambda _: self.sweep_changed.emit())
        self._ccw_radio.toggled.connect(lambda _: self.sweep_changed.emit())

        self._gen_btn.clicked.connect(self.generate_requested)
        self._clear_btn.clicked.connect(self.clear_requested)

        self._exp_json_btn.clicked.connect(self.export_json)
        self._exp_csv_btn.clicked.connect(self.export_csv)

    def _select_all_regions(self) -> None:
        for region, btn in self._region_btns.items():
            btn.blockSignals(True)
            btn.setChecked(True)
            btn.blockSignals(False)
        for region in _REGIONS:
            self.region_toggled.emit(region, True)

    def _clear_all_regions(self) -> None:
        for region, btn in self._region_btns.items():
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)
        for region in _REGIONS:
            self.region_toggled.emit(region, False)

    def _on_unit_changed(self, new_unit: str) -> None:
        old_mm = self._pitch_spin.value() * UNIT_TO_MM.get(self._current_unit, 1.0)
        self._pitch_spin.blockSignals(True)
        self._pitch_spin.setValue(old_mm / UNIT_TO_MM.get(new_unit, 1.0))
        self._pitch_spin.setSuffix(f'  {new_unit}')
        self._pitch_spin.blockSignals(False)
        self._current_unit = new_unit
        self.grid_changed.emit()

    # ------------------------------------------------------------------
    # Public accessors (mirror ParameterPanel API)
    # ------------------------------------------------------------------

    @property
    def current_unit(self) -> str:
        return self._current_unit

    def get_spray_width_mm(self) -> float:
        return self._pitch_spin.value() * UNIT_TO_MM.get(self._current_unit, 1.0)

    def get_v_width_mm(self) -> Optional[float]:
        return None   # vertical direction not exposed

    def get_direction(self) -> str:
        return 'horizontal'

    def get_path_target(self) -> str:
        return 'bbox' if self._bbox_radio.isChecked() else 'mesh'

    def is_direction_flipped(self) -> bool:
        return self._ccw_radio.isChecked()

    def is_show_grid(self) -> bool:
        return self._grid_check.isChecked()

    def is_show_arrows(self) -> bool:
        return self._arrows_check.isChecked()

    # ------------------------------------------------------------------
    # Public state setters (called by MainWindow)
    # ------------------------------------------------------------------

    def set_model_loaded(self, loaded: bool) -> None:
        for region, btn in self._region_btns.items():
            btn.setEnabled(loaded)
        for w in (self._all_btn, self._none_btn, self._select_btn,
                  self._unit_combo, self._pitch_spin,
                  self._cw_radio, self._ccw_radio,
                  self._bbox_radio, self._mesh_radio,
                  self._gen_btn, self._grid_check, self._arrows_check,
                  self._top_btn, self._bot_btn, self._front_btn,
                  self._rear_btn, self._left_btn, self._right_btn,
                  self._fit_btn):
            w.setEnabled(loaded)
        if not loaded:
            self._clear_btn.setEnabled(False)
            self._exp_json_btn.setEnabled(False)
            self._exp_csv_btn.setEnabled(False)

    def set_generating(self, generating: bool) -> None:
        self._gen_btn.setEnabled(not generating)
        self._clear_btn.setEnabled(not generating)
        self._gen_btn.setText('Generating…\n' if generating else 'Generate\nPath')

    def set_path_exists(self, exists: bool) -> None:
        self._clear_btn.setEnabled(exists)
        self._exp_json_btn.setEnabled(exists)
        self._exp_csv_btn.setEnabled(exists)

    def set_unit(self, unit: str) -> None:
        if unit in _UNITS and unit != self._current_unit:
            self._unit_combo.setCurrentText(unit)  # triggers _on_unit_changed

    def set_region_checked(self, region: str, checked: bool) -> None:
        btn = self._region_btns.get(region)
        if btn is None:
            return
        btn.blockSignals(True)
        btn.setChecked(checked)
        btn.blockSignals(False)

    def set_select_mode(self, active: bool) -> None:
        self._select_btn.blockSignals(True)
        self._select_btn.setChecked(active)
        self._select_btn.blockSignals(False)

    def update_mesh_stats(self, n_faces: int) -> None:
        self._tri_val.setText(f'{n_faces:,}')

    def update_route_stats(self, routes: list, display_unit: str) -> None:
        from app.path.path_model import UNIT_TO_MM
        total_passes = sum(r.total_passes for r in routes)
        total_conns  = sum(len(r.connections) for r in routes)
        total_mm     = sum(r.total_length_mm for r in routes)
        factor       = UNIT_TO_MM.get(display_unit, 1.0)
        length_disp  = total_mm / factor
        spacing_mm   = routes[0].spacing_mm if routes else 0.0
        spacing_disp = spacing_mm / factor
        self._passes_val.setText(str(total_passes))
        self._conn_val.setText(str(total_conns))
        self._len_val.setText(f'{length_disp:.2f} {display_unit}')
        self._spacing_val.setText(f'{spacing_disp:.2f} {display_unit}')

    def clear_stats(self) -> None:
        for v in (self._passes_val, self._conn_val, self._len_val, self._spacing_val):
            v.setText('—')


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _mk_vsep() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.VLine)
    f.setFixedWidth(1)
    f.setStyleSheet(f'color:{_SEP_COLOR};')
    return f
