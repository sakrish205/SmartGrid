"""PyVista QtInteractor wrapper. Owns ALL VTK actors — never let them go out of scope."""
from __future__ import annotations
from typing import Callable, Optional

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtWidgets import QWidget, QVBoxLayout, QMenu
from PySide6.QtGui import QAction
from PySide6.QtCore import Qt

try:
    from vtkmodules.vtkRenderingCore import vtkCellPicker
except ImportError:
    from vtk import vtkCellPicker   # older installs

from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintRoute
from app.ui.view_settings_dialog import DEFAULTS as _COLOR_DEFAULTS


def _hex_to_rgb(hex_color: str) -> tuple:
    c = hex_color.lstrip('#')
    return tuple(int(c[i:i+2], 16) / 255.0 for i in (0, 2, 4))


class MeshViewer(QWidget):
    """3D viewer: bbox clicking, manual face picking, route display."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._colors: dict[str, str] = dict(_COLOR_DEFAULTS)

        self.plotter = QtInteractor(self)
        self.plotter.set_background(self._colors['background'])
        layout.addWidget(self.plotter)

        self._actors: dict[str, object] = {}
        self._mesh_data: Optional[MeshData] = None
        self._bbox_solid_mesh: Optional[pv.PolyData] = None
        self._picking_mode: str = 'none'
        self._select_mode: bool = False
        self._saved_interactor_style = None

        # VTK observer handles (two per mode: press + release)
        self._obs_press:   Optional[int] = None
        self._obs_release: Optional[int] = None
        self._press_pos:   Optional[tuple] = None

        self.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.customContextMenuRequested.connect(self._show_view_menu)

    # ------------------------------------------------------------------
    # View context menu
    # ------------------------------------------------------------------

    def _show_view_menu(self, pos) -> None:
        menu = QMenu(self)
        menu.setStyleSheet(
            'QMenu{background:#ffffff;border:1px solid #c0c0c0;font-size:11px;'
            '  font-family:"Segoe UI",Arial;color:#1f1f1f;}'
            'QMenu::item{padding:4px 20px 4px 10px;}'
            'QMenu::item:selected{background:#0078d4;color:#fff;}'
            'QMenu::separator{height:1px;background:#d0d0d0;margin:2px 0;}'
        )
        title = menu.addAction('— View —')
        title.setEnabled(False)
        menu.addSeparator()
        for label, direction in [
            ('Fit All',      None),
            ('Top',         'top'),
            ('Bottom',      'bottom'),
            ('Front',       'front'),
            ('Rear',        'rear'),
            ('Left',        'left'),
            ('Right',       'right'),
        ]:
            act = menu.addAction(label)
            if direction is None:
                act.triggered.connect(self.fit_all)
            else:
                act.triggered.connect(lambda _=False, d=direction: self.set_view(d))
        menu.addSeparator()
        menu.addAction('Rotate Left  90°').triggered.connect(lambda: self.roll_view(-90))
        menu.addAction('Rotate Right 90°').triggered.connect(lambda: self.roll_view(+90))
        menu.exec(self.mapToGlobal(pos))

    # ------------------------------------------------------------------
    # Mesh loading
    # ------------------------------------------------------------------

    def load_mesh(self, mesh_data: MeshData) -> None:
        """Display mesh + tight green wireframe box + invisible picking box + axes."""
        self._remove_vtk_observers()
        self.plotter.clear()
        self._actors.clear()
        self._mesh_data = mesh_data
        self._bbox_solid_mesh = None
        self._picking_mode = 'none'

        # Solid mesh — fully opaque so hardware depth-buffer picks work
        actor_mesh = self.plotter.add_mesh(
            mesh_data.pyvista_mesh,
            color=self._colors['mesh'],
            opacity=1.0,
            show_edges=False,
            lighting=True,
            reset_camera=True,
        )
        self._actors['base_mesh'] = actor_mesh
        actor_mesh.SetPickable(False)

        # Wireframe bbox cage
        actor_wire = self.plotter.add_mesh(
            mesh_data.pyvista_mesh.outline(),
            color=self._colors['bbox_wire'],
            line_width=2.5,
            reset_camera=False,
        )
        self._actors['bbox_wire'] = actor_wire
        actor_wire.SetPickable(False)

        # Near-invisible solid box — click target for bbox mode
        bounds = mesh_data.pyvista_mesh.bounds
        pad = max(bounds[1]-bounds[0], bounds[3]-bounds[2], bounds[5]-bounds[4]) * 0.001
        padded = (
            bounds[0]-pad, bounds[1]+pad,
            bounds[2]-pad, bounds[3]+pad,
            bounds[4]-pad, bounds[5]+pad,
        )
        self._bbox_solid_mesh = pv.Box(bounds=padded)
        actor_box = self.plotter.add_mesh(
            self._bbox_solid_mesh,
            opacity=0.05,          # tiny but in depth buffer — needed for picking
            show_edges=False,
            reset_camera=False,
        )
        self._actors['bbox_solid'] = actor_box
        actor_box.SetPickable(True)

        # 3D orientation axes — top-right corner
        try:
            self.plotter.add_axes(
                line_width=4,
                xlabel='X', ylabel='Y', zlabel='Z',
                viewport=(0.75, 0.75, 1.0, 1.0),
            )
        except Exception:
            try:
                self.plotter.add_axes(line_width=4, xlabel='X', ylabel='Y', zlabel='Z')
            except Exception:
                pass

        self.plotter.render()

    # ------------------------------------------------------------------
    # VTK observer helpers (reliable single-click with drag detection)
    # ------------------------------------------------------------------

    def _get_vtk_iren(self):
        """Return the raw VTK interactor regardless of pyvistaqt version."""
        iren = getattr(self.plotter, 'iren', None)
        if iren is None:
            return None
        # pyvistaqt >= 0.11 wraps VTK interactor; the raw one is at .interactor
        raw = getattr(iren, 'interactor', None)
        if raw is not None and hasattr(raw, 'AddObserver'):
            return raw
        # Fallback: iren itself might already be the raw VTK interactor
        if hasattr(iren, 'AddObserver'):
            return iren
        return None

    def _remove_vtk_observers(self) -> None:
        raw = self._get_vtk_iren()
        if raw is None:
            return
        for oid in (self._obs_press, self._obs_release):
            if oid is not None:
                try:
                    raw.RemoveObserver(oid)
                except Exception:
                    pass
        self._obs_press = None
        self._obs_release = None

    def _install_vtk_observers(
        self,
        on_click: Callable[[tuple], None],
    ) -> None:
        """Install press+release observers for face picking."""
        self._remove_vtk_observers()
        raw = self._get_vtk_iren()
        if raw is None:
            return

        def _press(obj, event):
            self._press_pos = raw.GetEventPosition()

        def _release(obj, event):
            if self._press_pos is None:
                return
            pos = raw.GetEventPosition()
            press = self._press_pos
            self._press_pos = None
            if self._select_mode:
                # Treat as click only if mouse moved less than 5 px (not a drag)
                if max(abs(pos[0] - press[0]), abs(pos[1] - press[1])) < 5:
                    on_click(pos)

        self._obs_press   = raw.AddObserver('LeftButtonPressEvent',   _press,   1.0)
        self._obs_release = raw.AddObserver('LeftButtonReleaseEvent', _release, 1.0)

    def set_select_mode(self, active: bool) -> None:
        """Toggle select mode. Swaps the interactor style so camera does not
        rotate while Select Faces is active."""
        self._select_mode = active
        raw = self._get_vtk_iren()
        if raw is None:
            self.plotter.render()
            return
        if active:
            # Save current style and replace with a passive one (no camera moves)
            self._saved_interactor_style = raw.GetInteractorStyle()
            from vtkmodules.vtkInteractionStyle import vtkInteractorStyleUser
            raw.SetInteractorStyle(vtkInteractorStyleUser())
        else:
            if self._saved_interactor_style is not None:
                raw.SetInteractorStyle(self._saved_interactor_style)
                self._saved_interactor_style = None
        self.plotter.render()

    # ------------------------------------------------------------------
    # Bbox clicking (default mode)
    # ------------------------------------------------------------------

    def enable_bbox_clicking(self, callback: Callable[[str], None]) -> None:
        """Click a bbox face → callback(region_name).  Mesh is not pickable."""
        if self._mesh_data is None:
            return

        if 'base_mesh' in self._actors:
            self._actors['base_mesh'].SetPickable(False)
        if 'bbox_solid' in self._actors:
            self._actors['bbox_solid'].SetPickable(True)

        bbox_bounds = self._mesh_data.pyvista_mesh.bounds
        up_axis     = self._mesh_data.up_axis
        box_actor   = self._actors.get('bbox_solid')

        picker = vtkCellPicker()
        picker.SetTolerance(0.005)
        if box_actor is not None:
            picker.AddPickList(box_actor)
            picker.PickFromListOn()
        self._cell_picker = picker

        def on_click(pos):
            picker.Pick(pos[0], pos[1], 0, self.plotter.renderer)
            if picker.GetCellId() >= 0:
                pt = picker.GetPickPosition()
                region = _centroid_to_region(pt, bbox_bounds, up_axis)
                if region:
                    callback(region)

        self._install_vtk_observers(on_click)
        self._picking_mode = 'bbox'
        self.plotter.render()

    # ------------------------------------------------------------------
    # Face picking (Pick Faces button)
    # ------------------------------------------------------------------

    def enable_face_picking(self, callback: Callable[[int], None]) -> None:
        """Click a mesh triangle → callback(face_id).  Bbox is not pickable."""
        if 'base_mesh' in self._actors:
            self._actors['base_mesh'].SetPickable(True)
        if 'bbox_solid' in self._actors:
            self._actors['bbox_solid'].SetPickable(False)

        picker = vtkCellPicker()
        picker.SetTolerance(0.005)
        self._cell_picker = picker

        def on_click(pos):
            picker.Pick(pos[0], pos[1], 0, self.plotter.renderer)
            cid = picker.GetCellId()
            if cid >= 0:
                # Only accept picks that land on the base mesh (not grids, etc.)
                if picker.GetActor() is self._actors.get('base_mesh'):
                    callback(int(cid))

        self._install_vtk_observers(on_click)
        self._picking_mode = 'face'
        self.plotter.render()

    def disable_picking(self) -> None:
        self._remove_vtk_observers()
        self._picking_mode = 'none'

    # ------------------------------------------------------------------
    # Bbox face highlights (yellow) — mesh is never touched
    # ------------------------------------------------------------------

    def highlight_bbox_region(self, region: str, selected: bool) -> None:
        key = f'bbox_face_{region}'
        if key in self._actors:
            self.plotter.remove_actor(self._actors.pop(key))

        if selected and self._mesh_data is not None:
            face_mesh = _make_bbox_face(
                region,
                self._mesh_data.pyvista_mesh.bounds,
                self._mesh_data.up_axis,
            )
            if face_mesh is not None:
                actor = self.plotter.add_mesh(
                    face_mesh,
                    color=self._colors['face_highlight'],
                    opacity=0.45,
                    show_edges=True,
                    edge_color='#FFC107',
                    line_width=2.0,
                    lighting=False,
                    backface_culling=False,
                    reset_camera=False,
                )
                # Polygon offset: render the quad in front of coincident mesh
                # triangles without physically moving it (no visible gap).
                m = actor.GetMapper()
                m.SetResolveCoincidentTopologyToPolygonOffset()
                m.SetRelativeCoincidentTopologyPolygonOffsetParameters(-2.0, -2.0)
                self._actors[key] = actor

        self.plotter.render()

    def clear_bbox_selection(self) -> None:
        keys = [k for k in list(self._actors.keys()) if k.startswith('bbox_face_')]
        for k in keys:
            self.plotter.remove_actor(self._actors.pop(k))
        self.plotter.render()

    # ------------------------------------------------------------------
    # Grid / net overlay on bbox faces
    # ------------------------------------------------------------------

    def show_bbox_grid(
        self,
        region: str,
        bounds,
        up_axis: int,
        h_mm: float,
        v_mm: float,
    ) -> None:
        """Draw a white grid net on the bbox face at h_mm × v_mm cell size."""
        key = f'bbox_grid_{region}'
        if key in self._actors:
            self.plotter.remove_actor(self._actors.pop(key))

        lines = _make_grid_lines(region, bounds, up_axis, h_mm, v_mm)
        if lines is None or lines.n_points == 0:
            return

        actor = self.plotter.add_mesh(
            lines,
            color=self._colors['grid'],
            opacity=0.75,
            line_width=1.2,
            reset_camera=False,
        )
        self._actors[key] = actor
        self.plotter.render()

    def clear_bbox_grid(self, region: Optional[str] = None) -> None:
        """Remove grid for one region (or all if region is None)."""
        if region is not None:
            key = f'bbox_grid_{region}'
            if key in self._actors:
                self.plotter.remove_actor(self._actors.pop(key))
        else:
            keys = [k for k in list(self._actors.keys()) if k.startswith('bbox_grid_')]
            for k in keys:
                self.plotter.remove_actor(self._actors.pop(k))
        self.plotter.render()

    # ------------------------------------------------------------------
    # Mesh face highlight (blue — only for manual Pick Faces mode)
    # ------------------------------------------------------------------

    def highlight_mesh_faces(self, selected_face_ids: np.ndarray) -> None:
        """Overlay selected faces in blue; base mesh actor is NEVER replaced (picker stays valid)."""
        if self._mesh_data is None:
            return

        # Remove only the selection overlay — never remove base_mesh
        if 'sel_mesh' in self._actors:
            self.plotter.remove_actor(self._actors.pop('sel_mesh'))

        base_actor = self._actors.get('base_mesh')
        if base_actor is None:
            self.plotter.render()
            return

        if len(selected_face_ids) == 0:
            # Restore full opacity and user-configured mesh color
            base_actor.GetProperty().SetOpacity(1.0)
            base_actor.GetProperty().SetColor(*_hex_to_rgb(self._colors.get('mesh', '#78909c')))
        else:
            # Dim the base so selected faces stand out
            base_actor.GetProperty().SetOpacity(0.35)

            sel = self._mesh_data.pyvista_mesh.extract_cells(selected_face_ids)
            actor_sel = self.plotter.add_mesh(
                sel, color='#42A5F5', opacity=1.0,
                show_edges=False, lighting=True,
                backface_culling=False, reset_camera=False,
            )
            self._actors['sel_mesh'] = actor_sel
            actor_sel.SetPickable(False)

        self.plotter.render()

    def clear_mesh_highlight(self) -> None:
        self.highlight_mesh_faces(np.array([], dtype=np.int64))

    def highlight_selection(self, ids: np.ndarray) -> None:
        self.highlight_mesh_faces(ids)

    def clear_highlight(self) -> None:
        self.clear_mesh_highlight()

    # ------------------------------------------------------------------
    # Route visualisation
    # ------------------------------------------------------------------

    def show_route(
        self,
        routes: list[PaintRoute],
        show_arrows: bool = False,
        show_waypoints: bool = False,
    ) -> None:
        self.clear_route()

        # Arrow size — 2% of the longest bbox extent, clamped to a sensible range
        if self._mesh_data is not None:
            b = self._mesh_data.pyvista_mesh.bounds
            extents = [b[1]-b[0], b[3]-b[2], b[5]-b[4]]
            arrow_len = max(extents) * 0.025
            arrow_len = max(10.0, min(arrow_len, 300.0))
        else:
            arrow_len = 50.0

        for ri, route in enumerate(routes):
            for paint_pass in route.passes:
                if len(paint_pass.points) < 2:
                    continue
                line = pv.lines_from_points(paint_pass.points)
                color = (self._colors['pass_forward'] if paint_pass.is_forward
                         else self._colors['pass_reverse'])
                actor = self.plotter.add_mesh(
                    line, color=color,
                    line_width=float(self._colors.get('pass_line_width', '5.0')),
                    render_lines_as_tubes=True, reset_camera=False,
                )
                key = f'pass_{ri}_{paint_pass.id}_{paint_pass.sub_index}'
                self._actors[key] = actor

                if show_arrows:
                    _add_pass_chevrons(
                        self.plotter, self._actors,
                        paint_pass.points, color, arrow_len,
                        f'arr_{ri}_{paint_pass.id}_{paint_pass.sub_index}',
                        line_width=float(self._colors.get('arrow_line_width', '4.0')),
                    )

                if show_waypoints and len(paint_pass.points) >= 1:
                    wpt_color  = self._colors.get('waypoint', '#FFD700')
                    wpt_size   = float(self._colors.get('waypoint_size', '8.0'))
                    wpt_cloud  = pv.PolyData(paint_pass.points)
                    wpt_actor  = self.plotter.add_mesh(
                        wpt_cloud,
                        color=wpt_color,
                        point_size=wpt_size,
                        render_points_as_spheres=True,
                        reset_camera=False,
                    )
                    self._actors[f'wpt_{ri}_{paint_pass.id}_{paint_pass.sub_index}'] = wpt_actor

            for conn in route.connections:
                if len(conn.points) < 2:
                    continue
                line = pv.lines_from_points(conn.points)
                actor = self.plotter.add_mesh(
                    line,
                    color=self._colors['connector'],
                    line_width=3,
                    render_lines_as_tubes=True,
                    reset_camera=False,
                )
                self._actors[f'conn_{ri}_{conn.id}'] = actor

        self.plotter.render()

    def apply_colors(self, colors: dict[str, str]) -> None:
        """Apply a colour dict live — updates all existing actors immediately."""
        self._colors = dict(colors)

        self.plotter.set_background(colors['background'])

        if 'base_mesh' in self._actors:
            self._actors['base_mesh'].GetProperty().SetColor(
                *_hex_to_rgb(colors['mesh']))

        if 'bbox_wire' in self._actors:
            self._actors['bbox_wire'].GetProperty().SetColor(
                *_hex_to_rgb(colors['bbox_wire']))

        fwd_rgb = _hex_to_rgb(colors['pass_forward'])
        rev_rgb = _hex_to_rgb(colors['pass_reverse'])
        con_rgb = _hex_to_rgb(colors['connector'])
        wpt_rgb = _hex_to_rgb(colors.get('waypoint', '#FFD700'))
        wpt_size = float(colors.get('waypoint_size', '8.0'))
        for key, actor in self._actors.items():
            if key.startswith('pass_'):
                pass  # rerun show_route to change pass colors
            elif key.startswith('conn_'):
                actor.GetProperty().SetColor(*con_rgb)
            elif key.startswith('wpt_'):
                actor.GetProperty().SetColor(*wpt_rgb)
                actor.GetProperty().SetPointSize(wpt_size)
            elif key.startswith('bbox_grid_'):
                actor.GetProperty().SetColor(*_hex_to_rgb(colors['grid']))
            elif key.startswith('bbox_face_'):
                actor.GetProperty().SetColor(*_hex_to_rgb(colors['face_highlight']))

        self.plotter.render()

    def roll_view(self, degrees: float) -> None:
        self.plotter.camera.roll += degrees
        self.plotter.render()

    def clear_route(self) -> None:
        keys = [k for k in self._actors if k.startswith(('pass_', 'conn_', 'arr_', 'wpt_'))]
        for k in keys:
            self.plotter.remove_actor(self._actors.pop(k))
        self.plotter.render()

    # ------------------------------------------------------------------
    # Camera helpers
    # ------------------------------------------------------------------

    def fit_all(self) -> None:
        self.plotter.reset_camera()
        self.plotter.render()

    def set_view(self, direction: str) -> None:
        views = {
            'top':    ((0, 0,  1), (0, 1, 0)),
            'bottom': ((0, 0, -1), (0, 1, 0)),
            'front':  ((0, -1, 0), (0, 0, 1)),
            'rear':   ((0,  1, 0), (0, 0, 1)),
            'left':   ((-1, 0, 0), (0, 0, 1)),
            'right':  (( 1, 0, 0), (0, 0, 1)),
            'side':   (( 1, 0, 0), (0, 0, 1)),
        }
        if direction not in views:
            return
        pos_dir, view_up = views[direction]
        self.plotter.view_vector(pos_dir, viewup=view_up)
        self.plotter.render()


# ------------------------------------------------------------------
# Module-level geometry helpers
# ------------------------------------------------------------------

def _centroid_to_region(pt, bounds, up_axis: int) -> Optional[str]:
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    mins = [xmin, ymin, zmin]
    maxs = [xmax, ymax, zmax]
    fwd_axis   = (up_axis + 1) % 3
    right_axis = (up_axis + 2) % 3
    scores = {
        'TOP':    abs(pt[up_axis]    - maxs[up_axis]),
        'BOTTOM': abs(pt[up_axis]    - mins[up_axis]),
        'FRONT':  abs(pt[fwd_axis]   - maxs[fwd_axis]),
        'REAR':   abs(pt[fwd_axis]   - mins[fwd_axis]),
        'RIGHT':  abs(pt[right_axis] - maxs[right_axis]),
        'LEFT':   abs(pt[right_axis] - mins[right_axis]),
    }
    return min(scores, key=scores.get)


def _make_grid_lines(
    region: str, bounds, up_axis: int, h_mm: float, v_mm: float
) -> Optional[pv.PolyData]:
    """Return a PolyData of grid lines on the bbox face at h_mm × v_mm spacing."""
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    mins = [xmin, ymin, zmin]
    maxs = [xmax, ymax, zmax]
    fwd_axis   = (up_axis + 1) % 3
    right_axis = (up_axis + 2) % 3

    face_map = {
        'TOP':    (up_axis,    +1), 'BOTTOM': (up_axis,    -1),
        'FRONT':  (fwd_axis,   +1), 'REAR':   (fwd_axis,   -1),
        'RIGHT':  (right_axis, +1), 'LEFT':   (right_axis, -1),
    }
    if region not in face_map:
        return None

    face_axis, face_sign = face_map[region]
    face_pos = maxs[face_axis] if face_sign > 0 else mins[face_axis]

    if region in ('TOP', 'BOTTOM'):
        step_axis, pass_axis = fwd_axis, right_axis
        step_spacing, pass_spacing = h_mm, v_mm
    elif region in ('FRONT', 'REAR'):
        step_axis, pass_axis = up_axis, right_axis
        step_spacing, pass_spacing = v_mm, h_mm
    else:  # LEFT / RIGHT
        step_axis, pass_axis = up_axis, fwd_axis
        step_spacing, pass_spacing = v_mm, h_mm

    s_min, s_max = mins[step_axis], maxs[step_axis]
    p_min, p_max = mins[pass_axis], maxs[pass_axis]

    # Tiny offset so grid sits just above the face (avoids z-fighting)
    offset = (maxs[face_axis] - mins[face_axis]) * 0.002
    fp = face_pos + (offset if face_sign > 0 else -offset)

    all_pts: list[np.ndarray] = []
    cells: list[int] = []
    idx = 0

    def _add_line(a_val, b_val, fixed_axis, fixed_val, other_axis):
        nonlocal idx
        pa = np.zeros(3); pb = np.zeros(3)
        pa[face_axis] = pb[face_axis] = fp
        pa[fixed_axis] = pb[fixed_axis] = fixed_val
        pa[other_axis] = a_val; pb[other_axis] = b_val
        all_pts.extend([pa, pb])
        cells.extend([2, idx, idx + 1])
        idx += 2

    # Lines parallel to pass_axis — one every step_spacing
    for s in np.arange(s_min, s_max + step_spacing * 0.01, step_spacing):
        _add_line(p_min, p_max, step_axis, float(s), pass_axis)

    # Lines parallel to step_axis — one every pass_spacing
    for p in np.arange(p_min, p_max + pass_spacing * 0.01, pass_spacing):
        _add_line(s_min, s_max, pass_axis, float(p), step_axis)

    if not all_pts:
        return None

    mesh = pv.PolyData()
    mesh.points = np.array(all_pts, dtype=float)
    mesh.lines  = np.array(cells, dtype=np.int_)
    return mesh


def _add_pass_chevrons(
    plotter,
    actors: dict,
    points: np.ndarray,
    color: str,
    arrow_len: float,
    key_prefix: str,
    line_width: float = 4.0,
) -> None:
    """Draw surveying-style chevron tick marks (><) along a pass line."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return

    tick_len = arrow_len * 0.4

    # Walk the polyline and collect (center, seg_dir) at ~150 mm intervals
    seg_lens = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    total_len = seg_lens.sum()
    if total_len < 1e-9:
        return

    interval = max(150.0, total_len / max(1, int(total_len / 150)))

    positions: list[tuple] = []
    cum = 0.0
    next_pos = interval / 2.0  # first chevron at half-interval from start
    for i in range(len(pts) - 1):
        seg_d = pts[i + 1] - pts[i]
        sl = seg_lens[i]
        if sl < 1e-9:
            continue
        seg_dir = seg_d / sl
        while cum + sl >= next_pos:
            t = next_pos - cum
            center = pts[i] + seg_dir * t
            positions.append((center, seg_dir))
            next_pos += interval
        cum += sl

    if not positions:
        positions = [(pts[len(pts) // 2], (pts[-1] - pts[0]) / max(np.linalg.norm(pts[-1] - pts[0]), 1e-9))]

    # Build all chevron line pairs into one PolyData (fast — single actor)
    all_pts: list[np.ndarray] = []
    cells: list[int] = []
    idx = 0
    _z = np.array([0., 0., 1.])
    _y = np.array([0., 1., 0.])

    for center, seg_dir in positions:
        perp = np.cross(seg_dir, _z)
        pn = np.linalg.norm(perp)
        if pn < 0.15:
            perp = np.cross(seg_dir, _y)
            pn = np.linalg.norm(perp)
        perp = perp / pn if pn > 1e-9 else _y

        # Back-left and back-right arms (chevron points forward like ">")
        p1 = center - tick_len * (seg_dir + perp)
        p2 = center - tick_len * (seg_dir - perp)
        all_pts += [center.copy(), p1, center.copy(), p2]
        cells += [2, idx, idx + 1, 2, idx + 2, idx + 3]
        idx += 4

    mesh = pv.PolyData()
    mesh.points = np.array(all_pts, dtype=float)
    mesh.lines  = np.array(cells, dtype=np.int_)

    actor = plotter.add_mesh(
        mesh, color=color, line_width=line_width,
        render_lines_as_tubes=False, reset_camera=False,
    )
    actors[f'{key_prefix}_chev'] = actor


def _make_bbox_face(region: str, bounds, up_axis: int) -> Optional[pv.PolyData]:
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    mins = [xmin, ymin, zmin]
    maxs = [xmax, ymax, zmax]
    fwd_axis   = (up_axis + 1) % 3
    right_axis = (up_axis + 2) % 3

    face_map = {
        'TOP':    (up_axis,    +1),
        'BOTTOM': (up_axis,    -1),
        'FRONT':  (fwd_axis,   +1),
        'REAR':   (fwd_axis,   -1),
        'RIGHT':  (right_axis, +1),
        'LEFT':   (right_axis, -1),
    }
    if region not in face_map:
        return None

    face_axis, face_sign = face_map[region]
    face_pos = maxs[face_axis] if face_sign > 0 else mins[face_axis]
    a, b = [i for i in range(3) if i != face_axis]

    pts = np.zeros((4, 3), dtype=float)
    pts[:, face_axis] = face_pos
    pts[0, a] = mins[a]; pts[0, b] = mins[b]
    pts[1, a] = maxs[a]; pts[1, b] = mins[b]
    pts[2, a] = maxs[a]; pts[2, b] = maxs[b]
    pts[3, a] = mins[a]; pts[3, b] = maxs[b]

    return pv.PolyData(pts, np.array([4, 0, 1, 2, 3]))
