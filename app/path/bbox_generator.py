"""Generate flat spray-paint paths on a bounding-box face.

One call = one direction (horizontal OR vertical).
For crosshatch, the caller makes two calls and gets two separate routes.
"""
from __future__ import annotations
import numpy as np
from app.path.path_model import PaintPass, Connection, PaintRoute
from app.path.resampler import resample_arc, prune_collinear


def generate_bbox_route(
    region: str,
    bounds: tuple,
    spray_width_mm: float,
    up_axis: int,
    direction: str = 'horizontal',   # 'horizontal' | 'vertical'
    direction_offset: int = 0,       # 0=CW (default), 1=CCW (flip first pass)
    waypoint_spacing_mm: float = 0.0,  # 0 = no resampling; >0 = uniform waypoints
    standoff_mm: float = 0.0,          # outward offset from the face plane
    face_bounds: tuple | None = None,  # if set, use these bounds ONLY for face position
                                       # (bounds still controls pass width/height extent)
) -> PaintRoute:
    """Return a PaintRoute of parallel passes on the named bbox face.

    direction='horizontal' — passes sweep the wide axis, step the tall axis.
    direction='vertical'   — axes swapped (90° rotation of horizontal).

    face_bounds: when provided (e.g. the full mesh bbox), the face plane position is
    derived from face_bounds rather than bounds. This lets the standard zero-standoff
    position coincide with the mesh bounding box face (the blue wire cage) while the
    pass width/height is still clipped to the selected region's own extents.
    """
    if spray_width_mm <= 0:
        raise ValueError(f"spray_width_mm must be > 0, got {spray_width_mm}")
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    mins = [xmin, ymin, zmin]
    maxs = [xmax, ymax, zmax]

    if face_bounds is not None:
        fxmin, fxmax, fymin, fymax, fzmin, fzmax = face_bounds
        face_mins = [fxmin, fymin, fzmin]
        face_maxs = [fxmax, fymax, fzmax]
    else:
        face_mins, face_maxs = mins, maxs

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
        raise ValueError(f"Unknown region: {region!r}")

    face_axis, face_sign = face_map[region]
    face_pos = face_maxs[face_axis] if face_sign > 0 else face_mins[face_axis]
    face_pos += face_sign * standoff_mm   # shift outward by standoff distance

    # Horizontal base axes for each face
    if region in ('TOP', 'BOTTOM'):
        h_pass_axis = right_axis
        h_step_axis = fwd_axis
    elif region in ('FRONT', 'REAR'):
        h_pass_axis = right_axis
        h_step_axis = up_axis
    else:  # LEFT / RIGHT
        h_pass_axis = fwd_axis
        h_step_axis = up_axis

    if direction == 'vertical':
        pass_axis = h_step_axis
        step_axis = h_pass_axis
    else:  # horizontal (default)
        pass_axis = h_pass_axis
        step_axis = h_step_axis

    all_passes = _make_passes(
        face_axis, face_pos,
        step_axis=step_axis,
        pass_axis=pass_axis,
        step_spacing=spray_width_mm * 0.85,
        mins=mins, maxs=maxs,
        start_id=0,
        region=region,
        direction=direction,
        direction_offset=direction_offset,
        waypoint_spacing_mm=waypoint_spacing_mm,
    )

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
        for p in all_passes
        if len(p.points) >= 2
    )

    spray_normal = np.zeros(3)
    spray_normal[face_axis] = float(face_sign)

    return PaintRoute(
        region_id=region,
        passes=all_passes,
        connections=connections,
        unit='mm',
        spacing_mm=spray_width_mm,
        total_passes=len(all_passes),
        total_length_mm=total_length,
        spray_normal=spray_normal,
    )


# ── Multi-route merge ────────────────────────────────────────────────────────

def merge_routes(routes: list[PaintRoute]) -> PaintRoute:
    """Merge multiple single-region routes into one continuous path.

    Greedy nearest-neighbour ordering: from the current endpoint, pick the
    nearest end (start or finish) of remaining routes, flipping traversal
    direction if entering from the far end.  Air-move Connections bridge the
    gaps between segments.
    """
    if len(routes) <= 1:
        return routes[0]

    remaining = list(routes)
    ordered = [remaining.pop(0)]
    while remaining:
        last_pt = ordered[-1].passes[-1].points[-1]
        best_dist, best_idx, best_flip = float('inf'), 0, False
        for i, r in enumerate(remaining):
            d0 = float(np.linalg.norm(last_pt - r.passes[0].points[0]))
            d1 = float(np.linalg.norm(last_pt - r.passes[-1].points[-1]))
            if d0 < best_dist:
                best_dist, best_idx, best_flip = d0, i, False
            if d1 < best_dist:
                best_dist, best_idx, best_flip = d1, i, True
        r = remaining.pop(best_idx)
        ordered.append(_flip_route(r) if best_flip else r)

    all_passes: list[PaintPass] = []
    all_conns:  list[Connection] = []

    for seg_idx, route in enumerate(ordered):
        id_map: dict[int, int] = {}
        for p in route.passes:
            new_id = len(all_passes)
            id_map[p.id] = new_id
            all_passes.append(PaintPass(
                id=new_id, region_id=p.region_id, direction=p.direction,
                points=p.points.copy(), is_forward=p.is_forward,
                sub_index=p.sub_index, slice_position=p.slice_position,
            ))
        for c in route.connections:
            all_conns.append(Connection(
                id=len(all_conns),
                from_pass_id=id_map.get(c.from_pass_id, c.from_pass_id),
                to_pass_id=id_map.get(c.to_pass_id, c.to_pass_id),
                points=c.points.copy(), is_air_move=c.is_air_move,
            ))
        if seg_idx < len(ordered) - 1:
            all_conns.append(Connection(
                id=len(all_conns),
                from_pass_id=len(all_passes) - 1,
                to_pass_id=len(all_passes),
                points=np.array([
                    route.passes[-1].points[-1].copy(),
                    ordered[seg_idx + 1].passes[0].points[0].copy(),
                ], dtype=float),
                is_air_move=True,
            ))

    return PaintRoute(
        region_id='merged',
        passes=all_passes,
        connections=all_conns,
        unit=ordered[0].unit,
        spacing_mm=ordered[0].spacing_mm,
        total_passes=len(all_passes),
        total_length_mm=sum(r.total_length_mm for r in ordered),
        spray_normal=ordered[0].spray_normal,
    )


def _flip_route(route: PaintRoute) -> PaintRoute:
    """Reverse a route end-to-start for NN stitching."""
    passes = [
        PaintPass(
            id=p.id, region_id=p.region_id, direction=p.direction,
            points=p.points[::-1].copy(), is_forward=not p.is_forward,
            sub_index=p.sub_index, slice_position=p.slice_position,
        )
        for p in reversed(route.passes)
    ]
    conns = [
        Connection(
            id=c.id, from_pass_id=c.to_pass_id, to_pass_id=c.from_pass_id,
            points=c.points[::-1].copy(), is_air_move=c.is_air_move,
        )
        for c in reversed(route.connections)
    ]
    return PaintRoute(
        region_id=route.region_id, passes=passes, connections=conns,
        unit=route.unit, spacing_mm=route.spacing_mm,
        total_passes=route.total_passes, total_length_mm=route.total_length_mm,
        spray_normal=route.spray_normal,
    )


# ── Internal helper ──────────────────────────────────────────────────────────

def _make_passes(
    face_axis: int,
    face_pos: float,
    step_axis: int,
    pass_axis: int,
    step_spacing: float,
    mins: list,
    maxs: list,
    start_id: int,
    region: str,
    direction: str,
    direction_offset: int = 0,
    waypoint_spacing_mm: float = 0.0,
) -> list[PaintPass]:
    step_min = mins[step_axis]
    step_max = maxs[step_axis]
    pass_min = mins[pass_axis]
    pass_max = maxs[pass_axis]

    span = step_max - step_min
    if span <= step_spacing:
        step_positions = [(step_min + step_max) / 2.0]
    else:
        first = step_min + step_spacing / 2.0
        step_positions = list(np.arange(first, step_max, step_spacing))

    passes: list[PaintPass] = []
    for local_idx, step_pos in enumerate(step_positions):
        pass_id = start_id + local_idx
        is_forward = ((pass_id + direction_offset) % 2 == 0)

        pt_a = np.zeros(3, dtype=float)
        pt_b = np.zeros(3, dtype=float)
        pt_a[face_axis] = pt_b[face_axis] = face_pos
        pt_a[step_axis] = pt_b[step_axis] = step_pos
        pt_a[pass_axis] = pass_min
        pt_b[pass_axis] = pass_max

        pts = np.array([pt_a, pt_b], dtype=float)
        if not is_forward:
            pts = pts[::-1]

        if waypoint_spacing_mm > 0:
            pts = resample_arc(pts, waypoint_spacing_mm)
        pts = prune_collinear(pts)

        passes.append(PaintPass(
            id=pass_id,
            region_id=region,
            direction=direction,
            points=pts,
            is_forward=is_forward,
            sub_index=0,
            slice_position=float(step_pos),
        ))

    return passes
