"""Generate spray-paint passes on a named mesh region face.

Public API
----------
generate_face_grid_route(...)   Adaptive shadow-projection
generate_conform_route(...)     Conform: tilted-basis planes + trimesh intersection
get_face_grid_plane_corners(...)  4 corners of the tilted spray plane for visualisation
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.path_model import PaintPass, Connection, PaintRoute
from app.path.resampler import resample_arc, rdp_simplify
from app.path.stitcher import stitch_segments as _stitch_segs


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
    # Restrict basis_faces to the selected region's forward-facing faces.
    _fwd_mask = mesh.face_normals[face_indices, face_axis] * face_sign > 0.0
    basis_faces = face_indices[_fwd_mask]
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
        step_positions = list(np.arange(first, step_max + spray_width_mm * 0.5, spray_width_mm))

    band_half = spray_width_mm * 1.0

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
        conn_pts = np.array([
            all_passes[i].points[-1].copy(),
            all_passes[i + 1].points[0].copy(),
        ], dtype=float)
        if waypoint_spacing_mm > 0:
            conn_pts = resample_arc(conn_pts, waypoint_spacing_mm)
        connections.append(Connection(
            id=i,
            from_pass_id=all_passes[i].id,
            to_pass_id=all_passes[i + 1].id,
            points=conn_pts,
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


# ---------------------------------------------------------------------------
# Conform route — tilted-basis planes + trimesh intersection + uniform standoff
# ---------------------------------------------------------------------------

_RDP_EPS        = 0.3    # mm — same as generator.py
_MIN_PASS_FRAC  = 0.10
_MIN_PASS_ABS   = 5.0    # mm
_MAX_ANGLE_DEV  = 65.0   # degrees
_MAX_SUB_LEVEL  = 6




def _filter_chains(chains: list[np.ndarray], spray_width_mm: float) -> list[np.ndarray]:
    if not chains:
        return chains
    lengths = [float(np.sum(np.linalg.norm(np.diff(c, axis=0), axis=1))) for c in chains]
    order = sorted(range(len(chains)), key=lambda i: lengths[i], reverse=True)
    chains = [chains[i] for i in order]
    lengths = [lengths[i] for i in order]
    min_len = max(spray_width_mm * _MIN_PASS_FRAC, _MIN_PASS_ABS)
    cos_lim = np.cos(np.radians(_MAX_ANGLE_DEV))
    pdir = chains[0][-1] - chains[0][0]
    pn = np.linalg.norm(pdir)
    primary_dir = pdir / pn if pn > 1e-9 else np.array([1., 0., 0.])
    kept = [chains[0]]
    for c, arc in zip(chains[1:], lengths[1:]):
        if arc < min_len:
            continue
        d = c[-1] - c[0]
        dn = np.linalg.norm(d)
        if dn > 1e-9 and abs(np.dot(d / dn, primary_dir)) < cos_lim:
            continue
        kept.append(c)
    return kept[:_MAX_SUB_LEVEL]


def generate_conform_route(
    region: str,
    face_indices: np.ndarray,
    mesh: trimesh.Trimesh,
    up_axis: int,
    spray_width_mm: float,
    direction_offset: int = 0,
    waypoint_spacing_mm: float = 0.0,
    standoff_mm: float = 0.0,
) -> PaintRoute:
    """Conform toolpath: tilted-basis cutting planes + trimesh intersection.

    Uses Adaptive's mean-normal basis so planes are perpendicular to the
    surface step direction (correct arc-length spacing on tilted surfaces).
    Intersects the actual mesh (like Mesh Surface) for exact path geometry.
    Standoff applied uniformly via mean_n — no per-waypoint nearest-face snap.
    Face set: classifier-assigned faces only (no full-mesh forward-face leakage).
    """
    if len(face_indices) == 0:
        raise ValueError(f"Conform: region '{region}' has no faces.")

    face_axis, face_sign = _resolve_face_map(up_axis)[region]
    # Restrict basis_faces to the selected region's forward-facing faces only.
    # Using the whole mesh contaminates mean_n on complex meshes (other regions'
    # upward faces dilute the surface normal of the selected region).
    _fwd_mask = mesh.face_normals[face_indices, face_axis] * face_sign > 0.0
    basis_faces = face_indices[_fwd_mask]
    if len(basis_faces) == 0:
        basis_faces = face_indices
    mean_n, pass_vec, step_vec = _compute_surface_basis(basis_faces, mesh, up_axis)

    # Step extent along step_vec from the selected region vertices
    verts = mesh.vertices[mesh.faces[face_indices].ravel()]
    step_proj = verts @ step_vec
    step_min = float(step_proj.min()) - 0.001
    step_max = float(step_proj.max()) + 0.001

    span = step_max - step_min
    if span <= spray_width_mm:
        step_positions = [(step_min + step_max) / 2.0]
    else:
        first = step_min + spray_width_mm / 2.0
        step_positions = list(np.arange(first, step_max, spray_width_mm))

    all_passes: list[PaintPass] = []
    pass_id = 0

    for plane_index, step_pos in enumerate(step_positions):
        # Cutting plane: normal = step_vec, origin = step_pos along step_vec
        plane_origin = step_pos * step_vec
        result = trimesh.intersections.mesh_plane(
            mesh,
            plane_normal=step_vec,
            plane_origin=plane_origin,
            return_faces=True,
        )
        if result is None:
            continue
        segments, seg_face_ids = result
        if segments is None or len(segments) == 0:
            continue

        # Filter to classifier-assigned faces only (np.isin — avoids Python loop)
        mask = np.isin(seg_face_ids, face_indices)
        segments = segments[mask]
        if len(segments) == 0:
            continue

        # Remove degenerate segments
        lens = np.linalg.norm(segments[:, 1, :] - segments[:, 0, :], axis=1)
        segments = segments[lens > 1e-12]
        if len(segments) == 0:
            continue

        chains = _stitch_segs(segments)
        chains = _filter_chains(chains, spray_width_mm)

        is_forward = (plane_index % 2 == 0) if direction_offset == 0 else (plane_index % 2 == 1)

        for sub_idx, chain in enumerate(chains):
            pts = chain if is_forward else chain[::-1].copy()
            pts = rdp_simplify(pts, _RDP_EPS)
            if len(pts) < 2:
                continue
            # Uniform standoff along mean surface normal — no per-point snap
            if standoff_mm > 0.0:
                pts = pts + standoff_mm * mean_n
            if waypoint_spacing_mm > 0 and len(pts) >= 2:
                pts = resample_arc(pts, waypoint_spacing_mm)
            all_passes.append(PaintPass(
                id=pass_id,
                region_id=region,
                direction='horizontal',
                points=pts,
                is_forward=is_forward,
                sub_index=sub_idx,
                slice_position=float(step_pos),
            ))
            pass_id += 1

    connections: list[Connection] = []
    for i in range(len(all_passes) - 1):
        conn_pts = np.array([
            all_passes[i].points[-1].copy(),
            all_passes[i + 1].points[0].copy(),
        ], dtype=float)
        if waypoint_spacing_mm > 0:
            conn_pts = resample_arc(conn_pts, waypoint_spacing_mm)
        connections.append(Connection(
            id=i,
            from_pass_id=all_passes[i].id,
            to_pass_id=all_passes[i + 1].id,
            points=conn_pts,
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
) -> np.ndarray:
    """Return (4, 3) corners of the tilted spray plane for visualization.

    The plane normal is derived from the mean face normal of face_indices so
    the visualised plane automatically tilts to match the actual surface.
    Depth is the outermost vertex projected along mean_normal, then lifted by
    standoff_mm.  Extent comes from face_indices vertices.
    """
    face_axis, face_sign = _resolve_face_map(up_axis)[region]
    _fwd_mask = mesh.face_normals[face_indices, face_axis] * face_sign > 0.0
    basis_faces = face_indices[_fwd_mask]
    if len(basis_faces) == 0:
        basis_faces = face_indices
    mean_n, pass_vec, step_vec = _compute_surface_basis(basis_faces, mesh, up_axis)

    verts = mesh.vertices[mesh.faces[face_indices].ravel()]
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

