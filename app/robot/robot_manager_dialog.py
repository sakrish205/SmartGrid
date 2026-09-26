"""Robot profile manager dialog."""
from __future__ import annotations
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QListWidget, QPushButton,
    QDialogButtonBox, QFormLayout, QLineEdit, QDoubleSpinBox,
    QLabel, QMessageBox,
)
from PySide6.QtCore import Qt

from app.robot.robot_profile import RobotProfile, load_profiles, save_profiles

_DIALOG_CSS = (
    'QDialog{background:#ffffff;}'
    'QLabel{color:#252525;background:transparent;}'
    'QListWidget{background:#ffffff;color:#252525;border:1px solid #d2d0ce;'
    '  border-radius:2px;outline:none;}'
    'QListWidget::item{padding:4px 8px;color:#252525;}'
    'QListWidget::item:selected{background:#0078d4;color:#ffffff;}'
    'QListWidget::item:hover{background:#edebe9;}'
    'QPushButton{background:#ffffff;color:#252525;border:1px solid #d2d0ce;'
    '  border-radius:2px;padding:5px 14px;min-width:80px;}'
    'QPushButton:hover{background:#edebe9;border-color:#8a8886;}'
    'QPushButton:pressed{background:#d2d0ce;}'
    'QPushButton:disabled{color:#a19f9d;border-color:#e1dfdd;}'
    'QLineEdit{background:#ffffff;color:#252525;border:1px solid #d2d0ce;'
    '  border-radius:2px;padding:3px 6px;}'
    'QLineEdit:focus{border-color:#0078d4;}'
    'QDoubleSpinBox{background:#ffffff;color:#252525;border:1px solid #d2d0ce;'
    '  border-radius:2px;padding:2px 4px;}'
    'QFormLayout QLabel{color:#252525;}'
)


class RobotProfileDialog(QDialog):
    """Add / edit a single robot profile."""

    def __init__(self, profile: RobotProfile | None = None, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle('Robot Profile' if profile else 'New Robot Profile')
        self.setMinimumWidth(400)
        self.setStyleSheet(_DIALOG_CSS)

        p = profile or RobotProfile(name='New Robot')
        self._fields: dict[str, QDoubleSpinBox | QLineEdit] = {}

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)

        def _spin(lo, hi, val, dec=1, suffix=''):
            w = QDoubleSpinBox()
            w.setRange(lo, hi)
            w.setDecimals(dec)
            w.setValue(val)
            if suffix:
                w.setSuffix(f'  {suffix}')
            return w

        name_edit = QLineEdit(p.name)
        form.addRow('Name', name_edit)
        self._fields['name'] = name_edit

        specs = [
            # (field_name, label, min, max, value, decimals, suffix)
            ('tcp_spray_speed_mmps',      'Spray speed',           1, 5000,  p.tcp_spray_speed_mmps,      1, 'mm/s'),
            ('approach_speed_mmps',       'Approach speed',        1, 5000,  p.approach_speed_mmps,       1, 'mm/s'),
            ('max_joint_speed_degps',     'Max joint speed',       1,  360,  p.max_joint_speed_degps,     1, '°/s'),
            ('max_orientation_change_deg','Max orientation Δ',     1,   90,  p.max_orientation_change_deg,1, '°'),
            ('min_approach_angle_deg',    'Min approach angle',  -90,   90,  p.min_approach_angle_deg,    1, '°'),
            ('max_approach_angle_deg',    'Max approach angle',  -90,   90,  p.max_approach_angle_deg,    1, '°'),
            ('max_wrist_tilt_deg',        'Max wrist tilt',        0,  180,  p.max_wrist_tilt_deg,        1, '°'),
            ('standoff_optimal_mm',       'Standoff optimal',      1, 2000,  p.standoff_optimal_mm,       1, 'mm'),
            ('standoff_min_mm',           'Standoff min',          1, 2000,  p.standoff_min_mm,           1, 'mm'),
            ('standoff_max_mm',           'Standoff max',          1, 2000,  p.standoff_max_mm,           1, 'mm'),
            ('nozzle_radius_mm',          'Nozzle radius',         0,  500,  p.nozzle_radius_mm,          1, 'mm'),
            ('gun_body_length_mm',        'Gun body length',       1, 2000,  p.gun_body_length_mm,        1, 'mm'),
            ('gun_body_diameter_mm',      'Gun body diameter',     1,  500,  p.gun_body_diameter_mm,      1, 'mm'),
            ('spray_cone_deg',            'Spray cone',            1,  180,  p.spray_cone_deg,            1, '°'),
            ('overlap_factor_pct',        'Overlap',               0,   80,  p.overlap_factor_pct,        1, '%'),
            ('default_speed_mmpm',        'Default speed',         1, 1e6,   p.default_speed_mmpm,        0, 'mm/min'),
        ]
        for fname, label, lo, hi, val, dec, sfx in specs:
            w = _spin(lo, hi, val, dec, sfx)
            form.addRow(label, w)
            self._fields[fname] = w

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self._accept)
        btns.rejected.connect(self.reject)

        vl = QVBoxLayout(self)
        vl.addLayout(form)
        vl.addWidget(btns)

        self._result: RobotProfile | None = None

    def _accept(self) -> None:
        name = self._fields['name'].text().strip()
        if not name:
            QMessageBox.warning(self, 'Validation', 'Name cannot be empty.')
            return
        kwargs = {'name': name}
        for fname, w in self._fields.items():
            if fname == 'name':
                continue
            kwargs[fname] = w.value()
        kwargs['speed_units'] = 'mm/min'
        self._result = RobotProfile(**kwargs)
        self.accept()

    @property
    def profile(self) -> RobotProfile | None:
        return self._result


class RobotManagerDialog(QDialog):
    """List of saved robot profiles with Add / Edit / Delete / Set Active."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle('Robot Profiles')
        self.setMinimumSize(420, 340)
        self.setStyleSheet(_DIALOG_CSS)

        self._profiles, self._active = load_profiles()

        self._list = QListWidget()
        self._list.currentRowChanged.connect(self._on_sel)
        self._refresh_list()

        self._btn_add    = QPushButton('Add…')
        self._btn_edit   = QPushButton('Edit…')
        self._btn_delete = QPushButton('Delete')
        self._btn_active = QPushButton('Set Active')
        self._active_lbl = QLabel()
        self._update_active_label()

        for btn, slot in [
            (self._btn_add,    self._add),
            (self._btn_edit,   self._edit),
            (self._btn_delete, self._delete),
            (self._btn_active, self._set_active),
        ]:
            btn.clicked.connect(slot)

        btn_col = QVBoxLayout()
        for b in (self._btn_add, self._btn_edit, self._btn_delete, self._btn_active):
            btn_col.addWidget(b)
        btn_col.addStretch()

        row = QHBoxLayout()
        row.addWidget(self._list, stretch=1)
        row.addLayout(btn_col)

        close_btn = QDialogButtonBox(QDialogButtonBox.Close)
        close_btn.rejected.connect(self.accept)

        vl = QVBoxLayout(self)
        vl.addWidget(QLabel('Active:'))
        vl.addWidget(self._active_lbl)
        vl.addLayout(row)
        vl.addWidget(close_btn)

        self._on_sel(-1)

    # ------------------------------------------------------------------

    def _refresh_list(self) -> None:
        self._list.clear()
        for p in self._profiles:
            marker = ' ✓' if p.name == self._active else ''
            self._list.addItem(f'{p.name}{marker}')

    def _update_active_label(self) -> None:
        self._active_lbl.setText(self._active or '(none)')

    def _on_sel(self, row: int) -> None:
        has = row >= 0
        self._btn_edit.setEnabled(has)
        self._btn_delete.setEnabled(has)
        self._btn_active.setEnabled(has)

    def _save(self) -> None:
        save_profiles(self._profiles, self._active)
        self._refresh_list()
        self._update_active_label()

    def _add(self) -> None:
        dlg = RobotProfileDialog(parent=self)
        if dlg.exec() == QDialog.Accepted and dlg.profile:
            if any(p.name == dlg.profile.name for p in self._profiles):
                QMessageBox.warning(self, 'Duplicate', f'A profile named "{dlg.profile.name}" already exists.')
                return
            self._profiles.append(dlg.profile)
            self._save()

    def _edit(self) -> None:
        row = self._list.currentRow()
        if row < 0:
            return
        dlg = RobotProfileDialog(self._profiles[row], parent=self)
        if dlg.exec() == QDialog.Accepted and dlg.profile:
            old_name = self._profiles[row].name
            self._profiles[row] = dlg.profile
            if self._active == old_name:
                self._active = dlg.profile.name
            self._save()

    def _delete(self) -> None:
        row = self._list.currentRow()
        if row < 0:
            return
        name = self._profiles[row].name
        if QMessageBox.question(self, 'Delete', f'Delete "{name}"?') != QMessageBox.Yes:
            return
        self._profiles.pop(row)
        if self._active == name:
            self._active = None
        self._save()

    def _set_active(self) -> None:
        row = self._list.currentRow()
        if row < 0:
            return
        self._active = self._profiles[row].name
        self._save()

    @property
    def active_name(self) -> str | None:
        return self._active
