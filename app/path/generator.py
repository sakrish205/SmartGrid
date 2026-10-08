"""Mesh Surface path generation: slice → stitch → boustrophedon."""
from __future__ import annotations
import logging
import numpy as np

_log = logging.getLogger(__name__)
from app.mesh.preprocessor import MeshData
from app.path.path_model import PaintPass, PaintRoute
from app.path import slicer as _slicer
from app.path import stitcher as _stitcher
from app.path.resampler import rdp_simplify, resample_arc

_RDP_EPSILON       = 0.3    # mm — removes triangle discretisation jaggies
_MIN_PASS_FRACTION = 0.10   # drop passes shorter than 10% of spray_width
_MIN_PASS_ABS_MM   = 1.0    # absolute floor so thin edges survive
_MAX_ANGLE_DEV_DEG = 65.0   # angle limit for short sub-passes only
_MAX_SUB_PER_LEVEL = 6      # max polylines per slice (holes/islands)

_NAMED_FACES = frozenset({'TOP', 'BOTTOM', 'FRONT', 'REAR', 'LEFT', 'RIGHT'})


def _arc_length(pts: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def _dominant_dir(pts: np.ndarray) -> np.ndarray:
    d = pts[-1] - pts[0]
    n = np.linalg.norm(d)
    return d / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])


def _filter_polylines(polylines: list, spray_width_mm: float) -> list:
    """Keep primary (longest) pass; drop corner clips and misaligned short sub-passes."""
    if not polylines:
        return polylines

    lengths = [_arc_length(np.asarray(p, dtype=float)) for p in polylines]
    order   = sorted(range(len(polylines)), key=lambda i: lengths[i], reverse=True)
    polylines = [polylines[i] for i in order]
    lengths   = [lengths[i]   for i in order]

    min_len     = max(spray_width_mm * _MIN_PASS_FRACTION, _MIN_PASS_ABS_MM)
    cos_limit   = np.cos(np.radians(_MAX_ANGLE_DEV_DEG))
    primary_dir = _dominant_dir(np.asarray(polylines[0], dtype=float))

    kept = [polylines[0]]
    for poly, arc_len in zip(polylines[1:], lengths[1:]):
        if arc_len < min_len:
            continue
        if arc_len < min_len * 4:
            d = _dominant_dir(np.asarray(poly, dtype=float))
            if abs(np.dot(d, primary_dir)) < cos_limit:
                continue
        kept.append(poly)
    return kept


def _build_planes_combined(
    mesh,
    region_face_indices: np.ndarray,
    up: int,
    spray_width_mm: float,
    direction: str,
) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """Slice planes for combined multi-face selections.

    horizontal → right_axis planes — each cuts FRONT+TOP+REAR in one polyline
    vertical   → fwd_axis planes   — each cuts LEFT+TOP+RIGHT in one polyline

    Wrap-around emerges naturally: trimesh segments from adjacent faces share
    corner vertices; stitch_segments chains them into one continuous polyline.
    """
    fwd   = (up + 1) % 3
    right = (up + 2) % 3
    slice_axis = right if direction == 'horizontal' else fwd

    plane_normal = np.zeros(3, dtype=float)
    plane_normal[slice_axis] = 1.0

    verts    = mesh.vertices[mesh.faces[region_face_indices].ravel()]
    axis_min = verts[:, slice_axis].min() - 0.001
    axis_max = verts[:, slice_axis].max() + 0.001
    step     = spray_width_mm * 0.85

    if (axis_max - axis_min) <= step:
        positions = [float((axis_min + axis_max) / 2.0)]
    else:
        first     = axis_min + step / 2.0
        positions = [float(p) for p in np.arange(first, axis_max, step)]

    planes = []
    for pos in positions:
        origin = np.zeros(3, dtype=float)
        origin[slice_axis] = pos
        planes.append((plane_normal.copy(), origin, pos))
    return planes


def generate_route(
    mesh_data: MeshData,
    region_id: str,
    region_face_indices: np.ndarray,
    spray_width_mm: float,
    waypoint_spacing_mm: float = 0.0,
    direction: str = 'horizontal',
) -> PaintRoute:
    """Mesh Surface: slice → stitch → filter → simplify → resample → boustrophedon.

    Single named face  — per-region slice axis (existing slicer logic).
    Combined selection — direction drives the wrap axis:
      horizontal: right_axis planes produce passes that wrap FRONT→TOP→REAR
      vertical:   fwd_axis   planes produce passes that wrap LEFT→TOP→RIGHT

    Standoff and normals are applied after this call by _offset_route_by_standoff
    in main_window.py, so points here sit directly on the mesh surface.
    """
    mesh = mesh_data.trimesh_mesh
    up   = mesh_data.up_axis

    if region_id in _NAMED_FACES:
        planes = _slicer.compute_slice_planes(
            mesh, region_face_indices, region_id, up, spray_width_mm, direction,
        )
        slice_region_id = region_id
    else:
        planes = _build_planes_combined(
            mesh, region_face_indices, up, spray_width_mm, direction,
        )
        slice_region_id = ''   # strict np.isin filter in slice_region

    passes   = []
    pass_id  = 0
    for i, (plane_normal, plane_origin, pos) in enumerate(planes):
        segs = _slicer.slice_region(
            mesh, region_face_indices,
            plane_normal, plane_origin,
            region_id=slice_region_id,
            up_axis=up,
        )
        if segs is None:
            continue

        polylines = _stitcher.stitch_segments(segs)
        polylines = _filter_polylines(polylines, spray_width_mm)

        for sub_i, pts in enumerate(polylines[:_MAX_SUB_PER_LEVEL]):
            pts = np.asarray(pts, dtype=float)
            if len(pts) < 2:
                continue
            pts = rdp_simplify(pts, _RDP_EPSILON)
            if waypoint_spacing_mm > 0.0:
                pts = resample_arc(pts, waypoint_spacing_mm)

            is_fwd = (i % 2 == 0)
            if not is_fwd:
                pts = pts[::-1]

            passes.append(PaintPass(
                id=pass_id, region_id=region_id,
                direction=direction, points=pts,
                is_forward=is_fwd, sub_index=sub_i,
                slice_position=pos,
                normals=None, tangent=None,
            ))
            pass_id += 1

    total_len = sum(_arc_length(p.points) for p in passes)
    return PaintRoute(
        region_id=region_id, passes=passes, connections=[],
        unit='mm', spacing_mm=spray_width_mm,
        total_passes=len(passes), total_length_mm=total_len,
    )


if __name__ == '__main__':
    # Self-check: box mesh, combined 5-face selection
    import trimesh as _tm
    from app.mesh.preprocessor import preprocess
    from app.mesh.regions import classify_regions

    box  = _tm.creation.box(extents=[200, 300, 100])
    data = preprocess(box, 'box.stl', up_axis=2)
    regions = classify_regions(data)

    # Combine all except BOTTOM (jig face)
    paint = {r: regions[r] for r in regions if r != 'BOTTOM' and len(regions[r]) > 0}
    combined = np.unique(np.concatenate(list(paint.values())).astype(np.int64))
    name = '+'.join(sorted(paint.keys()))

    route = generate_route(data, name, combined, spray_width_mm=30.0)
    assert route.total_passes > 0, 'no passes generated'
    for p in route.passes:
        assert len(p.points) >= 2, f'pass {p.id} has fewer than 2 points'
    print(f'self-check OK — {route.total_passes} passes, '
          f'{route.total_length_mm:.1f} mm total')
