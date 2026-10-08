"""Generate spray-paint passes on a named mesh region face.

Public API
----------
generate_adaptive_grid_route(...)  Adaptive H×V intersection-grid route
generate_face_grid_route(...)      Adaptive shadow-projection (single direction)
generate_conform_route(...)        Conform: tilted-basis planes + trimesh intersection
get_face_grid_plane_corners(...)   4 corners of the tilted spray plane for visualisation
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.path_model import PaintPass, Connection, PaintRoute
from app.path.resampler import resample_arc, rdp_simplify, prune_collinear
from app.path.stitcher import stitch_segments as _stitch_segs
from app.path.local_normals import interpolate_normals, build_tcp_frames
from app.path import connector as _connector


def _batch_normals_tangents(passes: list, mesh, mean_n: np.ndarray) -> None:
    """One BVH call for all waypoints; assign normals + tangent back per pass."""
    if not passes:
        return
    counts  = [len(p.points) for p in passes]
    all_pts = np.vstack([p.points for p in passes])
    all_n   = interpolate_normals(all_pts, mesh)
    flip    = all_n @ mean_n < 0
    all_n[flip] = -all_n[flip]
    idx = 0
    for p, n_pts in zip(passes, counts):
        local_n   = all_n[idx:idx + n_pts]
        p.normals = local_n
        p.tangent = build_tcp_frames(p.points, local_n)
        idx += n_pts


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


def _get_basis_faces(region: str, face_indices: np.ndarray,
                     mesh, up_axis: int) -> np.ndarray:
    """Return forward-facing subset of face_indices for basis computation.

    Falls back to hemisphere filter for combined region names (e.g. 'FRONT+LEFT').
    """
    face_map = _resolve_face_map(up_axis)
    if region in face_map:
        face_axis, face_sign = face_map[region]
        mask = mesh.face_normals[face_indices, face_axis] * face_sign > 0.0
        result = face_indices[mask]
        return result if len(result) > 0 else face_indices
    # Combined region — hemisphere filter from mean normal
    mean_n = mesh.face_normals[face_indices].mean(axis=0)
    n = np.linalg.norm(mean_n)
    if n > 1e-9:
        mask = mesh.face_normals[face_indices] @ (mean_n / n) >= 0.0
        result = face_indices[mask]
        return result if mask.any() else face_indices
    return face_indices



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
        # Normal is nearly parallel to up — try forward axis
        fwd_vec = np.zeros(3, dtype=float)
        fwd_vec[(up_axis + 1) % 3] = 1.0
        pass_vec = np.cross(mean_n, fwd_vec)
        pv_len = np.linalg.norm(pass_vec)
        if pv_len < 1e-9:
            # Completely degenerate — use lateral axis as final fallback
            pass_vec = np.zeros(3, dtype=float)
            pass_vec[(up_axis + 2) % 3] = 1.0
            pv_len = 1.0
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
    direction: str = 'horizontal',
) -> PaintRoute:
    """Return a PaintRoute of surface-tilted parallel passes.

    Computes the mean face normal of face_indices, builds an orthonormal
    spray-plane basis (mean_normal, pass_vec, step_vec), then for each row
    shadow-projects the outermost vertex depth along mean_normal so paths sit
    on the actual tilted surface.  Standoff lifts paths outward from there.
    """
    basis_faces = _get_basis_faces(region, face_indices, mesh, up_axis)
    mean_n, pass_vec, step_vec = _compute_surface_basis(basis_faces, mesh, up_axis)
    if direction == 'vertical':
        pass_vec, step_vec = step_vec, pass_vec

    verts      = mesh.vertices[mesh.faces[basis_faces].ravel()]
    all_verts  = mesh.vertices[mesh.faces[face_indices].ravel()]
    # Extent from all assigned faces; depth sampling from forward-facing basis_faces only.
    step_min        = float((all_verts @ step_vec).min())
    step_max        = float((all_verts @ step_vec).max())
    global_pass_min = float((all_verts @ pass_vec).min())
    global_pass_max = float((all_verts @ pass_vec).max())
    global_depth    = float((verts @ mean_n).max())

    _step     = spray_width_mm * 0.85
    band_half = spray_width_mm * 2.0
    step_proj = verts @ step_vec   # basis_faces only — for depth band sampling

    span = step_max - step_min
    if span <= _step:
        step_positions = [(step_min + step_max) / 2.0]
    else:
        first = step_min + _step / 2.0
        step_positions = list(np.arange(first, step_max + _step * 0.5, _step))

    all_passes: list[PaintPass] = []
    for local_idx, step_pos in enumerate(step_positions):
        pass_id    = local_idx
        is_forward = ((pass_id + direction_offset) % 2 == 0)

        in_band    = np.abs(step_proj - step_pos) <= band_half
        band_verts = verts[in_band]
        row_depth  = float((band_verts @ mean_n).max()) if len(band_verts) > 0 else global_depth

        row_face_pos = row_depth + standoff_mm
        pt_a = row_face_pos * mean_n + global_pass_min * pass_vec + step_pos * step_vec
        pt_b = row_face_pos * mean_n + global_pass_max * pass_vec + step_pos * step_vec

        pts = np.array([pt_a, pt_b], dtype=float)
        if not is_forward:
            pts = pts[::-1].copy()

        if waypoint_spacing_mm > 0:
            pts = resample_arc(pts, waypoint_spacing_mm)

        all_passes.append(PaintPass(
            id=pass_id,
            region_id=region,
            direction=direction,
            points=pts,
            is_forward=is_forward,
            sub_index=0,
            slice_position=float(step_pos),
        ))

    _batch_normals_tangents(all_passes, mesh, mean_n)

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


def _keep_outermost_segs(segments: np.ndarray, mean_n: np.ndarray, gap_mm: float) -> np.ndarray:
    """Keep only segments on the outermost surface along mean_n.

    Projects each segment midpoint onto mean_n. When a gap >= gap_mm exists
    between depth-sorted clusters (outer shell vs inner panel), everything
    below the largest gap is dropped.
    ponytail: single largest-gap split — if a part has 3+ shells at very
    different depths this keeps only the outermost; add iterative splitting
    if that ever matters.
    """
    if len(segments) <= 1:
        return segments
    midpts = segments.mean(axis=1)           # (N, 3)
    depths = midpts @ mean_n                 # scalar depth per segment
    order  = np.argsort(depths)
    gaps   = np.diff(depths[order])
    if gaps.size == 0 or gaps.max() < gap_mm:
        return segments                      # no inner panel gap — keep all
    split = int(np.argmax(gaps)) + 1        # first index in outermost cluster
    keep  = np.zeros(len(segments), dtype=bool)
    keep[order[split:]] = True
    return segments[keep]




def generate_conform_route(
    region: str,
    face_indices: np.ndarray,
    mesh: trimesh.Trimesh,
    up_axis: int,
    spray_width_mm: float,
    direction_offset: int = 0,
    waypoint_spacing_mm: float = 0.0,
    standoff_mm: float = 0.0,
    direction: str = 'horizontal',
) -> PaintRoute:
    """Conform: Mesh Surface slicing logic + per-waypoint standoff + TCP frames.

    Reuses the same trimesh.intersections.mesh_plane loop as generate_route
    (Mesh Surface), then lifts each waypoint along its local surface normal
    by standoff_mm and stores per-waypoint normals + TCP tangent frames.
    """
    if len(face_indices) == 0:
        raise ValueError(f"Conform: region '{region}' has no faces.")

    basis_faces = _get_basis_faces(region, face_indices, mesh, up_axis)
    mean_n, pass_vec, step_vec = _compute_surface_basis(basis_faces, mesh, up_axis)
    if direction == 'vertical':
        pass_vec, step_vec = step_vec, pass_vec

    all_verts = mesh.vertices[mesh.faces[face_indices].ravel()]
    step_min = float((all_verts @ step_vec).min()) - 0.001
    step_max = float((all_verts @ step_vec).max()) + 0.001
    _step = spray_width_mm * 0.85
    span = step_max - step_min
    if span <= _step:
        step_positions = [(step_min + step_max) / 2.0]
    else:
        first = step_min + _step / 2.0
        step_positions = list(np.arange(first, step_max, _step))

    all_passes: list[PaintPass] = []
    pass_id = 0

    for plane_index, step_pos in enumerate(step_positions):
        result = trimesh.intersections.mesh_plane(
            mesh,
            plane_normal=step_vec,
            plane_origin=step_pos * step_vec,
            return_faces=True,
        )
        if result is None:
            continue
        raw_segs, seg_face_ids = result
        if raw_segs is None or len(raw_segs) == 0:
            continue

        face_mask = mesh.face_normals[seg_face_ids] @ mean_n >= 0.0
        segments  = raw_segs[face_mask]
        if len(segments) == 0:
            continue

        # Drop inner-panel segments: keep only the outermost depth cluster.
        # Gap threshold: larger of 8 mm or 12% of spray width — catches typical
        # bumper/panel shell separation (10-50 mm) without splitting surface ripple.
        _gap_mm  = max(spray_width_mm * 0.12, 8.0)
        segments = _keep_outermost_segs(segments, mean_n, _gap_mm)
        if len(segments) == 0:
            continue

        seg_lens = np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1)
        segments  = segments[seg_lens > 1e-12]
        if len(segments) == 0:
            continue

        polylines = _stitch_segs(segments)
        if not polylines:
            continue

        from app.path.generator import _filter_polylines
        polylines = _filter_polylines(polylines, spray_width_mm)[:_MAX_SUB_LEVEL]

        is_forward = (plane_index + direction_offset) % 2 == 0

        for sub_idx, poly in enumerate(polylines):
            if len(poly) < 2:
                continue
            pts = poly if is_forward else poly[::-1].copy()
            pts = rdp_simplify(pts, _RDP_EPS)
            _sp = waypoint_spacing_mm if waypoint_spacing_mm > 0 else spray_width_mm * 0.4
            if len(pts) >= 2:
                pts = resample_arc(pts, _sp)
            pts = prune_collinear(pts)
            if len(pts) < 2:
                continue

            all_passes.append(PaintPass(
                id=pass_id, region_id=region, direction=direction,
                points=pts, is_forward=is_forward, sub_index=sub_idx,
                slice_position=float(step_pos),
            ))
            pass_id += 1

    # Same TSP-lite ordering as generate_route
    from collections import defaultdict
    _lmap: dict[float, list] = defaultdict(list)
    for _p in all_passes:
        _lmap[round(_p.slice_position, 4)].append(_p)
    _sorted: list[PaintPass] = []
    for _pos in sorted(_lmap.keys()):
        _grp = _lmap[_pos]
        if len(_grp) > 1:
            _cur = _sorted[-1].points[-1] if _sorted else _grp[0].points[0]
            _rem = list(_grp)
            while _rem:
                _i = min(range(len(_rem)), key=lambda i: min(
                    np.linalg.norm(_rem[i].points[0] - _cur),
                    np.linalg.norm(_rem[i].points[-1] - _cur),
                ))
                _p = _rem.pop(_i)
                if np.linalg.norm(_p.points[-1] - _cur) < np.linalg.norm(_p.points[0] - _cur):
                    _p = PaintPass(id=_p.id, region_id=_p.region_id, direction=_p.direction,
                                   points=_p.points[::-1].copy(), is_forward=not _p.is_forward,
                                   sub_index=_p.sub_index, slice_position=_p.slice_position)
                _sorted.append(_p)
                _cur = _p.points[-1]
        else:
            _sorted.extend(_grp)
    all_passes = _sorted

    # Batch normals after TSP ordering — one BVH call covers all waypoints,
    # and points are already in final order so no reversal needed.
    _batch_normals_tangents(all_passes, mesh, mean_n)

    # Apply standoff along mean_n (same as BBox/Adaptive) — per-waypoint normals are
    # for TCP orientation only; using them for offset on curved surfaces scatters
    # waypoints up to 90° off when face normals diverge from the spray direction.
    if standoff_mm > 0.0:
        for _p in all_passes:
            _p.points = _p.points + standoff_mm * mean_n

    connections = _connector.connect_passes(
        all_passes, spray_width_mm=spray_width_mm, waypoint_spacing_mm=waypoint_spacing_mm,
    )

    total_length = sum(
        float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1)))
        for p in all_passes if len(p.points) >= 2
    )
    return PaintRoute(
        region_id=region, passes=all_passes, connections=connections,
        unit='mm', spacing_mm=spray_width_mm,
        total_passes=len(all_passes), total_length_mm=total_length,
        spray_normal=mean_n.copy(),
    )


def generate_adaptive_grid_route(
    region: str,
    face_indices: np.ndarray,
    mesh: trimesh.Trimesh,
    up_axis: int,
    spray_width_mm: float,
    direction_offset: int = 0,
    waypoint_spacing_mm: float = 0.0,
    standoff_mm: float = 0.0,
    direction: str = 'horizontal',
) -> tuple['PaintRoute', np.ndarray]:
    """H×V intersection-grid adaptive route.

    Runs H and V grids internally, samples actual surface depth at every
    (row, col) intersection, and builds passes as polylines through those
    waypoints.  Returns (route, grid_pts) where grid_pts shape is (n_h, n_v, 3)
    — used by the viewer for the curved red grid replacing the flat spray plane.
    """
    basis_faces = _get_basis_faces(region, face_indices, mesh, up_axis)
    mean_n, pass_vec, step_vec = _compute_surface_basis(basis_faces, mesh, up_axis)
    # Always build in H-basis; direction swap handled when emitting passes.

    # Use all faces that face generally toward mean_n (same hemisphere filter
    # as the Conform/Mesh-Surface segment filter).  This prevents the
    # single-assignment classifier from shrinking the extent: boundary faces
    # (e.g. FRONT-leaning faces on the LEFT surface) are classified to an
    # adjacent region and absent from face_indices, but they are still
    # physically on the visible surface and must set the grid extent.
    fwd_face_ids = np.where(mesh.face_normals @ mean_n >= 0.0)[0]
    fwd_verts    = mesh.vertices[mesh.faces[fwd_face_ids].ravel()]
    verts        = mesh.vertices[mesh.faces[basis_faces].ravel()]
    global_depth = float((fwd_verts @ mean_n).max())

    # Extent and depth sampling from all forward-hemisphere faces.
    pass_min = float((fwd_verts @ pass_vec).min())
    pass_max = float((fwd_verts @ pass_vec).max())
    step_min = float((fwd_verts @ step_vec).min())
    step_max = float((fwd_verts @ step_vec).max())

    # Projections for per-cell depth sampling — forward-hemisphere faces.
    pass_proj = fwd_verts @ pass_vec
    step_proj = fwd_verts @ step_vec

    _step     = spray_width_mm * 0.85
    band_half = spray_width_mm * 2.0

    def _positions(lo, hi):
        if hi - lo <= _step:
            return [(lo + hi) / 2.0]
        first = lo + _step / 2.0
        return list(np.arange(first, hi + _step * 0.5, _step))

    h_steps = _positions(step_min, step_max)   # rows  — along step_vec
    v_steps = _positions(pass_min, pass_max)   # cols  — along pass_vec
    n_h, n_v = len(h_steps), len(v_steps)

    # Sample surface depth at every (row i, col j) grid intersection.
    # Precompute the row mask once per H row to avoid O(N*M*V) fully recomputed work.
    fwd_n     = fwd_verts @ mean_n   # depth projection for all forward-hemisphere verts
    grid_pts = np.empty((n_h, n_v, 3), dtype=float)
    for i, s in enumerate(h_steps):
        in_row    = np.abs(step_proj - s) <= band_half
        row_pass  = pass_proj[in_row]
        row_n     = fwd_n[in_row] if in_row.any() else None
        row_depth = float(row_n.max()) if row_n is not None else global_depth
        for j, p in enumerate(v_steps):
            if row_n is not None:
                in_col = np.abs(row_pass - p) <= band_half
                depth  = float(row_n[in_col].max()) if in_col.any() else row_depth
            else:
                depth = global_depth
            grid_pts[i, j] = (depth + standoff_mm) * mean_n + p * pass_vec + s * step_vec

    def _resample_anchored(pts: np.ndarray, spacing: float) -> np.ndarray:
        """Resample between each consecutive pair of grid intersection points,
        preserving the intersections themselves as mandatory waypoints."""
        if spacing <= 0 or len(pts) < 2:
            return pts
        out = [pts[0]]
        for k in range(len(pts) - 1):
            seg = resample_arc(pts[k:k + 2], spacing)
            out.extend(seg[1:])   # seg[0] already in out; seg[-1] = pts[k+1]
        return np.array(out, dtype=float)

    # Build toolpath passes for the selected direction.
    all_passes: list[PaintPass] = []
    if direction == 'horizontal':
        for i, s in enumerate(h_steps):
            is_forward = ((i + direction_offset) % 2 == 0)
            pts = grid_pts[i].copy()
            if not is_forward:
                pts = pts[::-1].copy()
            pts = _resample_anchored(pts, waypoint_spacing_mm)
            all_passes.append(PaintPass(
                id=i, region_id=region, direction=direction,
                points=pts, is_forward=is_forward, sub_index=0,
                slice_position=float(s),
            ))
    else:  # vertical
        for j, p in enumerate(v_steps):
            is_forward = ((j + direction_offset) % 2 == 0)
            pts = grid_pts[:, j].copy()
            if not is_forward:
                pts = pts[::-1].copy()
            pts = _resample_anchored(pts, waypoint_spacing_mm)
            all_passes.append(PaintPass(
                id=j, region_id=region, direction=direction,
                points=pts, is_forward=is_forward, sub_index=0,
                slice_position=float(p),
            ))

    _batch_normals_tangents(all_passes, mesh, mean_n)

    connections: list[Connection] = []
    for i in range(len(all_passes) - 1):
        conn_pts = np.array([
            all_passes[i].points[-1].copy(),
            all_passes[i + 1].points[0].copy(),
        ], dtype=float)
        if waypoint_spacing_mm > 0:
            conn_pts = resample_arc(conn_pts, waypoint_spacing_mm)
        connections.append(Connection(
            id=i, from_pass_id=all_passes[i].id,
            to_pass_id=all_passes[i + 1].id,
            points=conn_pts, is_air_move=False,
        ))

    total_length = sum(
        float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1)))
        for p in all_passes if len(p.points) >= 2
    )

    return PaintRoute(
        region_id=region, passes=all_passes, connections=connections,
        unit='mm', spacing_mm=spray_width_mm, total_passes=len(all_passes),
        total_length_mm=total_length, spray_normal=mean_n.copy(),
    ), grid_pts


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
    basis_faces = _get_basis_faces(region, face_indices, mesh, up_axis)
    mean_n, pass_vec, step_vec = _compute_surface_basis(basis_faces, mesh, up_axis)

    verts     = mesh.vertices[mesh.faces[basis_faces].ravel()]
    all_verts = mesh.vertices[mesh.faces[face_indices].ravel()]

    pass_min = float((all_verts @ pass_vec).min())
    pass_max = float((all_verts @ pass_vec).max())
    step_min = float((all_verts @ step_vec).min())
    step_max = float((all_verts @ step_vec).max())
    face_depth = float((verts @ mean_n).max()) + standoff_mm

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

