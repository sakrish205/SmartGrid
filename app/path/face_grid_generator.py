"""Generate spray-paint passes on a named mesh region face using shadow projection.

Public API
----------
generate_face_grid_route(...)
    Shadow-projected passes: each row's depth is the outermost vertex in that
    row's band, so paths always sit on (or above, with standoff) the actual
    mesh surface rather than a global flat bbox plane.

get_face_grid_plane_corners(...)
    Returns the 4 corners of the spray plane rectangle for 3-D visualisation.
    Always uses region vertex tight bounds for face depth (matches paths).

compute_mesh_shaped_passes(...)
    Low-level: given geometry parameters and the region vertex cloud, returns
    list[PaintPass] with per-row width AND depth clipped to the mesh silhouette.
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.path_model import PaintPass, Connection, PaintRoute
from app.path.resampler import resample_arc

FACE_PLANE_NAME  = 'Face Plane'
SPRAY_PLANE_NAME = 'Spray Plane'


def _axes(up_axis: int) -> tuple[int, int, int]:
    fwd   = (up_axis + 1) % 3
    right = (up_axis + 2) % 3
    return up_axis, fwd, right


def _resolve_face_map(up_axis: int) -> dict[str, tuple[int, int]]:
    up, fwd, right = _axes(up_axis)
    return {
        'TOP':    (up,    +1),
        'BOTTOM': (up,    -1),
        'FRONT':  (fwd,  +1),
        'REAR':   (fwd,  -1),
        'RIGHT':  (right, +1),
        'LEFT':   (right, -1),
    }


def _plane_axes(region: str, up_axis: int) -> tuple[int, int, int]:
    """Return (face_axis, pass_axis, step_axis) for the named region."""
    up, fwd, right = _axes(up_axis)
    face_axis = _resolve_face_map(up_axis)[region][0]

    if region in ('TOP', 'BOTTOM'):
        pass_axis, step_axis = right, fwd
    elif region in ('FRONT', 'REAR'):
        pass_axis, step_axis = right, up
    else:  # LEFT / RIGHT
        pass_axis, step_axis = fwd, up

    return face_axis, pass_axis, step_axis


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def generate_face_grid_route(
    region: str,
    face_indices: np.ndarray,
    mesh: trimesh.Trimesh,
    up_axis: int,
    spray_width_mm: float,
    direction_offset: int = 0,
    waypoint_spacing_mm: float = 0.0,
    standoff_mm: float = 0.0,
) -> PaintRoute:
    """Return a PaintRoute of parallel passes using shadow projection.

    Each row's depth is the outermost vertex within that row's band along the
    face axis.  Standoff is added outward from that surface, so paths never
    land inside recessed geometry.
    """
    if region not in _resolve_face_map(up_axis):
        raise ValueError(f'Unknown region: {region!r}')

    face_map = _resolve_face_map(up_axis)
    face_axis, face_sign = face_map[region]
    _, pass_axis, step_axis = _plane_axes(region, up_axis)

    region_verts = mesh.vertices[mesh.faces[face_indices].ravel()]

    all_passes = compute_mesh_shaped_passes(
        face_axis=face_axis,
        step_axis=step_axis,
        pass_axis=pass_axis,
        step_spacing=spray_width_mm,
        region_verts=region_verts,
        start_id=0,
        region=region,
        direction_offset=direction_offset,
        waypoint_spacing_mm=waypoint_spacing_mm,
        face_sign=face_sign,
        standoff_mm=standoff_mm,
    )

    connections: list[Connection] = []
    for i in range(len(all_passes) - 1):
        connections.append(Connection(
            id=i,
            from_pass_id=all_passes[i].id,
            to_pass_id=all_passes[i + 1].id,
            points=np.array([
                all_passes[i].points[-1].copy(),
                all_passes[i + 1].points[0].copy(),
            ], dtype=float),
            is_air_move=False,
        ))

    total_length = sum(
        float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1)))
        for p in all_passes if len(p.points) >= 2
    )

    return PaintRoute(
        region_id=region,
        passes=all_passes,
        connections=connections,
        unit='mm',
        spacing_mm=spray_width_mm,
        total_passes=len(all_passes),
        total_length_mm=total_length,
    )


def get_face_grid_plane_corners(
    region: str,
    face_indices: np.ndarray,
    mesh: trimesh.Trimesh,
    up_axis: int,
    standoff_mm: float = 0.0,
    mesh_bounds: tuple | None = None,
) -> np.ndarray:
    """Return (4, 3) corners of the spray plane rectangle for visualization.

    Face depth always comes from region vertex tight bounds (matching path
    shadow projection).  Width/height come from mesh_bounds when provided so
    the visual plane spans the full bbox face, giving a clear reference frame.
    """
    face_map = _resolve_face_map(up_axis)
    face_axis, face_sign = face_map[region]
    _, pass_axis, step_axis = _plane_axes(region, up_axis)

    region_verts = mesh.vertices[mesh.faces[face_indices].ravel()]
    rmins = region_verts.min(axis=0)
    rmaxs = region_verts.max(axis=0)

    # Width/height: full bbox when available, else region extent
    if mesh_bounds is not None:
        bx0, bx1, by0, by1, bz0, bz1 = mesh_bounds
        b0 = np.array([bx0, by0, bz0], dtype=float)
        b1 = np.array([bx1, by1, bz1], dtype=float)
        pass_min, pass_max = float(b0[pass_axis]), float(b1[pass_axis])
        step_min, step_max = float(b0[step_axis]), float(b1[step_axis])
    else:
        pass_min, pass_max = float(rmins[pass_axis]), float(rmaxs[pass_axis])
        step_min, step_max = float(rmins[step_axis]), float(rmaxs[step_axis])

    # Depth: always region outermost surface + standoff
    face_pos = float(rmaxs[face_axis] if face_sign > 0 else rmins[face_axis])
    face_pos += face_sign * standoff_mm

    corners = np.zeros((4, 3), dtype=float)
    for ci, (pa, sa) in enumerate([
        (pass_min, step_min),
        (pass_max, step_min),
        (pass_max, step_max),
        (pass_min, step_max),
    ]):
        corners[ci][face_axis] = face_pos
        corners[ci][pass_axis] = pa
        corners[ci][step_axis] = sa

    return corners


# ---------------------------------------------------------------------------
# Low-level helper
# ---------------------------------------------------------------------------

def compute_mesh_shaped_passes(
    face_axis: int,
    step_axis: int,
    pass_axis: int,
    step_spacing: float,
    region_verts: np.ndarray,
    start_id: int,
    region: str,
    direction_offset: int = 0,
    waypoint_spacing_mm: float = 0.0,
    face_sign: int = 0,
    standoff_mm: float = 0.0,
    face_pos: float = 0.0,   # used only when face_sign == 0
) -> list[PaintPass]:
    """Return parallel passes whose width AND depth follow the mesh silhouette.

    When face_sign is non-zero each row's depth is the outermost vertex in
    that row's band along face_axis (shadow projection), and standoff_mm is
    added outward from there.  When face_sign == 0 the fixed face_pos is used.
    """
    step_min = float(region_verts[:, step_axis].min())
    step_max = float(region_verts[:, step_axis].max())
    global_pass_min = float(region_verts[:, pass_axis].min())
    global_pass_max = float(region_verts[:, pass_axis].max())

    # Global fallback depth (outermost vertex of whole region)
    if face_sign > 0:
        global_surface = float(region_verts[:, face_axis].max())
    elif face_sign < 0:
        global_surface = float(region_verts[:, face_axis].min())
    else:
        global_surface = face_pos

    span = step_max - step_min
    if span <= step_spacing:
        step_positions = [(step_min + step_max) / 2.0]
    else:
        first = step_min + step_spacing / 2.0
        step_positions = list(np.arange(first, step_max, step_spacing))

    band_half = step_spacing * 0.65   # wide enough to find vertices at each row

    passes: list[PaintPass] = []
    for local_idx, step_pos in enumerate(step_positions):
        pass_id   = start_id + local_idx
        is_forward = ((pass_id + direction_offset) % 2 == 0)

        in_band    = np.abs(region_verts[:, step_axis] - step_pos) <= band_half
        band_verts = region_verts[in_band]

        if len(band_verts) == 0:
            pass_min   = global_pass_min
            pass_max   = global_pass_max
            row_surface = global_surface
        else:
            pass_min    = float(band_verts[:, pass_axis].min())
            pass_max    = float(band_verts[:, pass_axis].max())
            if face_sign > 0:
                row_surface = float(band_verts[:, face_axis].max())
            elif face_sign < 0:
                row_surface = float(band_verts[:, face_axis].min())
            else:
                row_surface = face_pos

        row_face_pos = (row_surface + face_sign * standoff_mm
                        if face_sign != 0 else face_pos)

        pt_a = np.zeros(3, dtype=float)
        pt_b = np.zeros(3, dtype=float)
        pt_a[face_axis] = pt_b[face_axis] = row_face_pos
        pt_a[step_axis] = pt_b[step_axis] = step_pos
        pt_a[pass_axis] = pass_min
        pt_b[pass_axis] = pass_max

        pts = np.array([pt_a, pt_b], dtype=float)
        if not is_forward:
            pts = pts[::-1].copy()

        if waypoint_spacing_mm > 0:
            pts = resample_arc(pts, waypoint_spacing_mm)

        passes.append(PaintPass(
            id=pass_id,
            region_id=region,
            direction='horizontal',
            points=pts,
            is_forward=is_forward,
            sub_index=0,
            slice_position=float(step_pos),
        ))

    return passes
