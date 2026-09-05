"""Generate spray-paint passes on a named mesh region face using shadow projection.

Public API
----------
generate_face_grid_route(...)
    Surface-tilted passes: computes mean normal of selected faces, builds an
    orthonormal spray-plane basis (mean_normal, pass_vec, step_vec), then
    shadow-projects each row's depth from the outermost vertex in that band.
    The spray plane automatically tilts to match the actual surface orientation.

get_face_grid_plane_corners(...)
    Returns 4 corners of the tilted spray plane for 3-D visualisation.
    Plane normal and extent are derived from the actual face normals.

compute_mesh_shaped_passes(...)
    Low-level axis-aligned helper (used by bbox path mode).
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
# Surface-tilt basis
# ---------------------------------------------------------------------------

def _compute_surface_basis(
    face_indices: np.ndarray,
    mesh: trimesh.Trimesh,
    up_axis: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (mean_normal, pass_vec, step_vec) orthonormal basis.

    mean_normal — outward unit normal of the surface (plane tilts to match it)
    pass_vec    — left-right direction across the spray plane (along passes)
    step_vec    — step direction (advances between rows)
    """
    normals = mesh.face_normals[face_indices]
    mean_n = normals.mean(axis=0)
    n_len = np.linalg.norm(mean_n)
    if n_len < 1e-9:
        mean_n = np.zeros(3, dtype=float)
        mean_n[up_axis] = 1.0
    else:
        mean_n = mean_n / n_len

    # pass_vec: horizontal direction in the spray plane, derived from global up
    up_vec = np.zeros(3, dtype=float)
    up_vec[up_axis] = 1.0
    pass_vec = np.cross(mean_n, up_vec)
    pv_len = np.linalg.norm(pass_vec)
    if pv_len < 1e-9:
        # Normal is nearly parallel to up — use forward axis instead
        fwd_vec = np.zeros(3, dtype=float)
        fwd_vec[(up_axis + 1) % 3] = 1.0
        pass_vec = np.cross(mean_n, fwd_vec)
        pv_len = np.linalg.norm(pass_vec)
    pass_vec = pass_vec / pv_len

    # step_vec: perpendicular to both normal and pass_vec (the step direction)
    step_vec = np.cross(pass_vec, mean_n)
    step_vec = step_vec / np.linalg.norm(step_vec)

    return mean_n, pass_vec, step_vec


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
    """Return a PaintRoute of surface-tilted parallel passes.

    Computes the mean face normal of face_indices, builds an orthonormal
    spray-plane basis (mean_normal, pass_vec, step_vec), then for each row
    shadow-projects the outermost vertex depth along mean_normal so paths sit
    on the actual tilted surface.  Standoff lifts paths outward from there.
    """
    if region not in _resolve_face_map(up_axis):
        raise ValueError(f'Unknown region: {region!r}')

    face_axis, face_sign = _resolve_face_map(up_axis)[region]
    # Basis from forward-facing faces only — closed-mesh all-face normals cancel to zero
    basis_faces = np.where(mesh.face_normals[:, face_axis] * face_sign > 0.0)[0].astype(np.int64)
    if len(basis_faces) == 0:
        basis_faces = face_indices
    mean_n, pass_vec, step_vec = _compute_surface_basis(basis_faces, mesh, up_axis)

    verts = mesh.vertices[mesh.faces[face_indices].ravel()]
    pass_proj = verts @ pass_vec   # 1-D coords along left-right axis
    step_proj = verts @ step_vec   # 1-D coords along step axis
    depth_proj = verts @ mean_n    # 1-D coords along spray-normal

    step_min = float(step_proj.min())
    step_max = float(step_proj.max())
    global_pass_min = float(pass_proj.min())
    global_pass_max = float(pass_proj.max())
    global_depth = float(depth_proj.max())   # outermost surface

    span = step_max - step_min
    if span <= spray_width_mm:
        step_positions = [(step_min + step_max) / 2.0]
    else:
        first = step_min + spray_width_mm / 2.0
        step_positions = list(np.arange(first, step_max, spray_width_mm))

    band_half = spray_width_mm * 0.65

    all_passes: list[PaintPass] = []
    for local_idx, step_pos in enumerate(step_positions):
        pass_id    = local_idx
        is_forward = ((pass_id + direction_offset) % 2 == 0)

        in_band    = np.abs(step_proj - step_pos) <= band_half
        band_verts = verts[in_band]

        if len(band_verts) == 0:
            p_min      = global_pass_min
            p_max      = global_pass_max
            row_depth  = global_depth
        else:
            p_min     = float((band_verts @ pass_vec).min())
            p_max     = float((band_verts @ pass_vec).max())
            row_depth = float((band_verts @ mean_n).max())

        row_face_pos = row_depth + standoff_mm

        # Build world-space endpoints in the tilted plane
        pt_a = row_face_pos * mean_n + p_min * pass_vec + step_pos * step_vec
        pt_b = row_face_pos * mean_n + p_max * pass_vec + step_pos * step_vec

        pts = np.array([pt_a, pt_b], dtype=float)
        if not is_forward:
            pts = pts[::-1].copy()

        if waypoint_spacing_mm > 0:
            pts = resample_arc(pts, waypoint_spacing_mm)

        all_passes.append(PaintPass(
            id=pass_id,
            region_id=region,
            direction='horizontal',
            points=pts,
            is_forward=is_forward,
            sub_index=0,
            slice_position=float(step_pos),
        ))

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
        spray_normal=mean_n.copy(),
    )


def get_face_grid_plane_corners(
    region: str,
    face_indices: np.ndarray,
    mesh: trimesh.Trimesh,
    up_axis: int,
    standoff_mm: float = 0.0,
    mesh_bounds: tuple | None = None,
) -> np.ndarray:
    """Return (4, 3) corners of the tilted spray plane for visualization.

    The plane normal is derived from the mean face normal of face_indices so
    the visualised plane automatically tilts to match the actual surface.
    Depth is the outermost vertex projected along mean_normal, then lifted by
    standoff_mm.  The mesh_bounds parameter is accepted for API compatibility
    but ignored — extent comes from face_indices vertices.
    """
    face_axis, face_sign = _resolve_face_map(up_axis)[region]
    # Basis from forward-facing faces — same rule as generate_face_grid_route
    basis_faces = np.where(mesh.face_normals[:, face_axis] * face_sign > 0.0)[0].astype(np.int64)
    if len(basis_faces) == 0:
        basis_faces = face_indices
    mean_n, pass_vec, step_vec = _compute_surface_basis(basis_faces, mesh, up_axis)

    # Extent from basis_faces (forward-facing surface), not face_indices (may be full mesh)
    # This keeps the reference plane tight around the actual visible surface area.
    verts = mesh.vertices[mesh.faces[basis_faces].ravel()]
    pass_proj  = verts @ pass_vec
    step_proj  = verts @ step_vec
    depth_proj = verts @ mean_n

    pass_min, pass_max = float(pass_proj.min()), float(pass_proj.max())
    step_min, step_max = float(step_proj.min()), float(step_proj.max())
    face_depth = float(depth_proj.max()) + standoff_mm

    # Centre of the plane in world space
    pc = (pass_min + pass_max) / 2.0
    sc = (step_min + step_max) / 2.0
    centre = face_depth * mean_n + pc * pass_vec + sc * step_vec

    dp = (pass_max - pass_min) / 2.0
    ds = (step_max - step_min) / 2.0

    return np.array([
        centre - dp * pass_vec - ds * step_vec,   # BL
        centre + dp * pass_vec - ds * step_vec,   # BR
        centre + dp * pass_vec + ds * step_vec,   # TR
        centre - dp * pass_vec + ds * step_vec,   # TL
    ], dtype=float)


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
