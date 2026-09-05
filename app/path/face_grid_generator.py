"""Generate flat spray-paint passes on the face plane of a named mesh region.

Public API
----------
generate_face_grid_route(...)
    Mesh-shaped passes: each pass spans only as wide as the mesh is at that
    height, derived from region vertex bands.  Face plane position anchors to
    the full mesh bbox face (blue reference line); standoff is measured from
    there outward (red robot path plane).

get_face_grid_plane_corners(...)
    Returns the 4 corners of either the blue (standoff=0) or red (standoff>0)
    reference rectangle for 3-D visualisation.

compute_mesh_shaped_passes(...)
    Low-level: given explicit geometry parameters and the region vertex cloud,
    returns a list[PaintPass] with per-row width clipped to the mesh silhouette.
    Call this directly if you need the pass list without the full route wrapper.
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.path_model import PaintPass, Connection, PaintRoute
from app.path.resampler import resample_arc

# Public plane names — shown in viewer legend and used as actor keys
FACE_PLANE_NAME  = 'Face Plane'   # blue: bbox face at zero standoff
SPRAY_PLANE_NAME = 'Spray Plane'  # red:  robot path at standoff distance


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
    face_map = _resolve_face_map(up_axis)
    face_axis = face_map[region][0]

    if region in ('TOP', 'BOTTOM'):
        pass_axis = right
        step_axis = fwd
    elif region in ('FRONT', 'REAR'):
        pass_axis = right
        step_axis = up
    else:  # LEFT / RIGHT
        pass_axis = fwd
        step_axis = up

    return face_axis, pass_axis, step_axis


def generate_face_grid_route(
    region: str,
    face_indices: np.ndarray,
    mesh: trimesh.Trimesh,
    up_axis: int,
    spray_width_mm: float,
    direction_offset: int = 0,
    waypoint_spacing_mm: float = 0.0,
    standoff_mm: float = 0.0,
    mesh_bounds: tuple | None = None,
) -> PaintRoute:
    """Return a PaintRoute of parallel passes projected onto the outermost mesh surface.

    Each pass row sits at the actual maximum (or minimum) vertex depth within that
    row's band — "shadow projection" — so paths never land inside recessed geometry.
    Standoff is then added outward from that surface depth.
    """
    if region not in _resolve_face_map(up_axis):
        raise ValueError(f'Unknown region: {region!r}')

    face_map = _resolve_face_map(up_axis)
    face_axis, face_sign = face_map[region]
    _, pass_axis, step_axis = _plane_axes(region, up_axis)

    # Region vertices — mesh-shaped pass width AND per-row surface depth
    region_verts = mesh.vertices[mesh.faces[face_indices].ravel()]

    all_passes = compute_mesh_shaped_passes(
        face_axis=face_axis,
        face_pos=0.0,          # unused when face_sign is provided
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
        for p in all_passes
        if len(p.points) >= 2
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
    region_face_pos: bool = False,
) -> np.ndarray:
    """Return (4, 3) corners of the face grid plane rectangle for visualization.

    mesh_bounds  — when given, plane extents (width/height) use the full mesh bbox.
    region_face_pos — when True, face position is taken from region vertex tight
                      bounds rather than the mesh bbox, while extents still follow
                      mesh_bounds.  Use this for the blue Face Plane so it sits
                      exactly on the actual mesh face surface.
    """
    face_map = _resolve_face_map(up_axis)
    face_axis, face_sign = face_map[region]
    _, pass_axis, step_axis = _plane_axes(region, up_axis)

    region_verts = mesh.vertices[mesh.faces[face_indices].ravel()]
    rmins = region_verts.min(axis=0)
    rmaxs = region_verts.max(axis=0)

    if mesh_bounds is not None:
        bxmin, bxmax, bymin, bymax, bzmin, bzmax = mesh_bounds
        bmins = np.array([bxmin, bymin, bzmin], dtype=float)
        bmaxs = np.array([bxmax, bymax, bzmax], dtype=float)
        pass_min = float(bmins[pass_axis])
        pass_max = float(bmaxs[pass_axis])
        step_min = float(bmins[step_axis])
        step_max = float(bmaxs[step_axis])
    else:
        pass_min = float(rmins[pass_axis])
        pass_max = float(rmaxs[pass_axis])
        step_min = float(rmins[step_axis])
        step_max = float(rmaxs[step_axis])

    # Face position: always from region tight bounds (actual mesh surface), not global bbox.
    # This matches the shadow-projection path generation so the visual plane lines up.
    if region_face_pos or True:   # always use region surface depth for accuracy
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


# ── Public low-level helper ──────────────────────────────────────────────────

def compute_mesh_shaped_passes(
    face_axis: int,
    face_pos: float,
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
) -> list[PaintPass]:
    """Return parallel passes whose width AND depth follow the mesh silhouette.

    Parameters
    ----------
    face_axis   : world axis perpendicular to the face plane (0=X,1=Y,2=Z)
    face_pos    : fallback face-plane coordinate (used only when face_sign=0)
    step_axis   : axis along which passes are stacked (vertical spacing)
    pass_axis   : axis along which each pass sweeps (horizontal)
    step_spacing: distance between consecutive passes (spray pitch, mm)
    region_verts: (N,3) float array — ALL vertices of the selected region
    start_id    : first pass id (for multi-region indexing)
    region      : region name string (stored on each PaintPass)
    direction_offset: 0=CW, 1=CCW first pass
    waypoint_spacing_mm: >0 resamples each pass at this interval
    face_sign   : +1 or -1 — outward normal sign along face_axis.
                  When non-zero, each row's depth is the outermost vertex in
                  that band (shadow projection) + standoff, so paths never
                  land inside recessed geometry.
    standoff_mm : outward offset from the projected surface (added along
                  face_sign direction when face_sign != 0).

    At each step position the function samples only the region vertices within
    ±65 % of step_spacing around that row.  The min/max of those vertices along
    pass_axis become the pass endpoints, and — when face_sign is set — the
    outermost vertex along face_axis becomes the pass depth (shadow projection).
    """
    step_min = float(region_verts[:, step_axis].min())
    step_max = float(region_verts[:, step_axis].max())
    global_pass_min = float(region_verts[:, pass_axis].min())
    global_pass_max = float(region_verts[:, pass_axis].max())
    global_face_pos = (
        float(region_verts[:, face_axis].max()) if face_sign > 0
        else float(region_verts[:, face_axis].min())
    ) if face_sign != 0 else face_pos

    span = step_max - step_min
    if span <= step_spacing:
        step_positions = [(step_min + step_max) / 2.0]
    else:
        first = step_min + step_spacing / 2.0
        step_positions = list(np.arange(first, step_max, step_spacing))

    # Search band: wide enough to always find some vertices near each step line
    band_half = step_spacing * 0.65

    passes: list[PaintPass] = []
    for local_idx, step_pos in enumerate(step_positions):
        pass_id = start_id + local_idx
        is_forward = ((pass_id + direction_offset) % 2 == 0)

        # Vertices within this step band → determines pass width AND surface depth
        in_band = np.abs(region_verts[:, step_axis] - step_pos) <= band_half
        band_verts = region_verts[in_band]
        if len(band_verts) == 0:
            pass_min = global_pass_min
            pass_max = global_pass_max
            row_face_pos = global_face_pos
        else:
            pass_min = float(band_verts[:, pass_axis].min())
            pass_max = float(band_verts[:, pass_axis].max())
            # Shadow projection: outermost surface vertex in this row's band
            if face_sign > 0:
                row_face_pos = float(band_verts[:, face_axis].max())
            elif face_sign < 0:
                row_face_pos = float(band_verts[:, face_axis].min())
            else:
                row_face_pos = face_pos

        # Apply standoff outward from the actual surface
        if face_sign != 0:
            row_face_pos += face_sign * standoff_mm
        else:
            row_face_pos = face_pos

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
