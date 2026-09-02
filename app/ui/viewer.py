"""PyVista QtInteractor wrapper. Owns ALL VTK actors — never let them go out of scope."""
from __future__ import annotations
from typing import Callable, Optional

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtWidgets import QWidget, QVBoxLayout

try:
    from vtkmodules.vtkRenderingCore import vtkCellPicker
except ImportError:
    from vtk import vtkCellPicker   # older installs

from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintRoute


class MeshViewer(QWidget):
    """3D viewer: bbox clicking, manual face picking, route display."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.plotter = QtInteractor(self)
        self.plotter.set_background('#1e1e2e')
        layout.addWidget(self.plotter)

        self._actors: dict[str, object] = {}
        self._mesh_data: Optional[MeshData] = None
        self._bbox_solid_mesh: Optional[pv.PolyData] = None
        self._picking_mode: str = 'none'

        # VTK observer handles (two per mode: press + release)
        self._obs_press:   Optional[int] = None
        self._obs_release: Optional[int] = None
        self._press_pos:   Optional[tuple] = None   # for drag-vs-click detection

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
            color='#b0bec5',
            opacity=1.0,
            show_edges=True,
            edge_color='#607d8b',
            line_width=0.5,
            lighting=True,
            reset_camera=True,
        )
        self._actors['base_mesh'] = actor_mesh
        actor_mesh.SetPickable(False)

        # Green wireframe cage (line cells — visual only)
        actor_wire = self.plotter.add_mesh(
            mesh_data.pyvista_mesh.outline(),
            color='#00FF41',
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
        drag_threshold: int = 6,
    ) -> None:
        """Install press+release observers that fire only when no drag occurred."""
        self._remove_vtk_observers()
        raw = self._get_vtk_iren()
        if raw is None:
            return

        def _press(obj, event):
            self._press_pos = raw.GetEventPosition()

        def _release(obj, event):
            if self._press_pos is None:
                return
            x, y = raw.GetEventPosition()
            px, py = self._press_pos
            self._press_pos = None
            if abs(x - px) > drag_threshold or abs(y - py) > drag_threshold:
                return   # drag (camera rotate) — ignore
            on_click((x, y))

        self._obs_press   = raw.AddObserver('LeftButtonPressEvent',   _press,   1.0)
        self._obs_release = raw.AddObserver('LeftButtonReleaseEvent', _release, 1.0)

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
                    color='#FFD700',
                    opacity=0.30,           # translucent enough to see paths through
                    show_edges=True,
                    edge_color='#FFC107',
                    line_width=2.0,
                    lighting=False,
                    backface_culling=False,
                    reset_camera=False,
                )
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
            color='#9E9E9E',
            opacity=0.70,
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
            # Restore full opacity
            base_actor.GetProperty().SetOpacity(1.0)
            base_actor.GetProperty().SetColor(0.69, 0.745, 0.769)   # #b0bec5
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

    def show_route(self, routes: list[PaintRoute], show_arrows: bool = False) -> None:
        self.clear_route()

        # Arrow size — 2% of the longest bbox extent, clamped to a sensible range
        if self._mesh_data is not None:
            b = self._mesh_data.pyvista_mesh.bounds
            extents = [b[1]-b[0], b[3]-b[2], b[5]-b[4]]
            arrow_len = max(extents) * 0.025
            arrow_len = max(10.0, min(arrow_len, 300.0))
        else:
            arrow_len = 50.0

        for route in routes:
            for paint_pass in route.passes:
                if len(paint_pass.points) < 2:
                    continue
                line = pv.lines_from_points(paint_pass.points)
                if paint_pass.direction == 'vertical':
                    color = '#4CAF50' if paint_pass.is_forward else '#FF9800'
                else:
                    color = '#2196F3' if paint_pass.is_forward else '#FF5722'
                actor = self.plotter.add_mesh(
                    line, color=color, line_width=5,
                    render_lines_as_tubes=True, reset_camera=False,
                )
                self._actors[f'pass_{paint_pass.id}_{paint_pass.sub_index}'] = actor

                if show_arrows:
                    _add_pass_arrows(
                        self.plotter, self._actors,
                        paint_pass.points, color, arrow_len,
                        f'arr_{paint_pass.id}_{paint_pass.sub_index}',
                    )

            for conn in route.connections:
                if len(conn.points) < 2:
                    continue
                line = pv.lines_from_points(conn.points)
                actor = self.plotter.add_mesh(
                    line,
                    color='#FF69B4',
                    line_width=3,
                    render_lines_as_tubes=True,
                    reset_camera=False,
                )
                self._actors[f'conn_{conn.id}'] = actor

        self.plotter.render()

    def _arrow_keys(self) -> list[str]:
        return [k for k in self._actors if k.startswith('arr_')]

    def clear_route(self) -> None:
        keys = [k for k in self._actors if k.startswith(('pass_', 'conn_', 'arr_'))]
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
            'top':   ((0, 0, 1), (0, 1, 0)),
            'front': ((0, -1, 0), (0, 0, 1)),
            'side':  ((1, 0, 0), (0, 0, 1)),
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


def _add_pass_arrows(
    plotter,
    actors: dict,
    points: np.ndarray,
    color: str,
    arrow_len: float,
    key_prefix: str,
) -> None:
    """Place one cone arrow per segment of a pass, pointing in travel direction."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return

    # Segment midpoints and direction vectors
    seg_mids  = (pts[:-1] + pts[1:]) * 0.5
    seg_vecs  = pts[1:] - pts[:-1]
    norms     = np.linalg.norm(seg_vecs, axis=1, keepdims=True)
    mask      = norms[:, 0] > 1e-9
    if not np.any(mask):
        return
    seg_mids  = seg_mids[mask]
    seg_vecs  = seg_vecs[mask]
    norms     = norms[mask]
    seg_dirs  = seg_vecs / norms

    # Down-sample: one arrow per ~200 mm of path length, min 1
    cum_len = np.cumsum(norms[:, 0])
    total   = cum_len[-1]
    interval = max(total / max(1, int(total / 200)), 1.0)
    chosen   = [0]
    last     = cum_len[0]
    for i in range(1, len(cum_len)):
        if cum_len[i] - last >= interval:
            chosen.append(i)
            last = cum_len[i]

    for i, idx in enumerate(chosen):
        center = seg_mids[idx]
        direction = seg_dirs[idx]
        # pv.Arrow: tip at +x by default; scale to arrow_len
        arrow = pv.Arrow(
            start=center - direction * arrow_len * 0.5,
            direction=direction,
            tip_length=0.35,
            tip_radius=0.12,
            shaft_radius=0.04,
            scale=arrow_len,
        )
        actor = plotter.add_mesh(
            arrow, color=color, opacity=0.92,
            lighting=True, reset_camera=False,
        )
        actors[f'{key_prefix}_{i}'] = actor


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
