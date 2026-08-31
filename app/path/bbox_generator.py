"""Generate flat spray-paint paths on a bounding-box face.

One call = one direction (horizontal OR vertical).
For crosshatch, the caller makes two calls and gets two separate routes.
"""
from __future__ import annotations
import numpy as np
from app.path.path_model import PaintPass, Connection, PaintRoute


def generate_bbox_route(
    region: str,
    bounds: tuple,
    spray_width_mm: float,
    up_axis: int,
    direction: str = 'horizontal',   # 'horizontal' | 'vertical'
) -> PaintRoute:
    """Return a PaintRoute of parallel passes on the named bbox face.

    direction='horizontal' — passes sweep the wide axis, step the tall axis.
    direction='vertical'   — axes swapped (90° rotation of horizontal).
    """
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
        raise ValueError(f"Unknown region: {region!r}")

    face_axis, face_sign = face_map[region]
    face_pos = maxs[face_axis] if face_sign > 0 else mins[face_axis]

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
        step_spacing=spray_width_mm,
        mins=mins, maxs=maxs,
        start_id=0,
        region=region,
        direction=direction,
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
        float(np.linalg.norm(p.points[-1] - p.points[0]))
        for p in all_passes
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
        is_forward = (pass_id % 2 == 0)

        pt_a = np.zeros(3, dtype=float)
        pt_b = np.zeros(3, dtype=float)
        pt_a[face_axis] = pt_b[face_axis] = face_pos
        pt_a[step_axis] = pt_b[step_axis] = step_pos
        pt_a[pass_axis] = pass_min
        pt_b[pass_axis] = pass_max

        pts = np.array([pt_a, pt_b], dtype=float)
        if not is_forward:
            pts = pts[::-1]

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
