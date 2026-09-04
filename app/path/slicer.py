"""Wraps trimesh.intersections.mesh_plane and filters by selected region triangles."""
from __future__ import annotations
import numpy as np
import trimesh
from app.mesh.preprocessor import MeshData


def compute_slice_config(
    region_id: str,
    up_axis: int,
    mean_face_normal: np.ndarray | None = None,
) -> dict:
    """Return plane_normal and slice_axis for the given region and up-axis.

    'Horizontal' passes run along the right axis and are spaced along:
      - fwd_axis  for TOP / BOTTOM faces (horizontal surfaces)
      - up_axis   for side faces (FRONT / REAR / LEFT / RIGHT)

    For arbitrary selections ('selection'), the dominant normal axis of the
    selected faces determines which branch applies.
    """
    fwd_axis = (up_axis + 1) % 3

    if region_id in ('TOP', 'BOTTOM'):
        slice_axis = fwd_axis
    elif region_id in ('FRONT', 'REAR', 'LEFT', 'RIGHT'):
        slice_axis = up_axis
    else:
        # Arbitrary selection: infer from dominant normal direction.
        if mean_face_normal is not None:
            dominant = int(np.argmax(np.abs(mean_face_normal)))
            slice_axis = fwd_axis if dominant == up_axis else up_axis
        else:
            slice_axis = up_axis   # fallback

    plane_normal = np.zeros(3, dtype=float)
    plane_normal[slice_axis] = 1.0
    return {'plane_normal': plane_normal, 'slice_axis': slice_axis}


def compute_slice_planes(
    mesh: trimesh.Trimesh,
    region_face_indices: np.ndarray,
    region_id: str,
    up_axis: int,
    spray_width_mm: float,
) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """Return list of (plane_normal, plane_origin, position) tuples.

    Planes are spaced at spray_width_mm intervals along the slice axis.
    Positions are computed from the region's own bounding box (not the full mesh).
    Planes are extended 0.001 mm beyond the region extents to catch boundary triangles.
    """
    # Compute mean of |normals| for the selected faces so arbitrary selections
    # can infer the correct slice axis without knowing the region name.
    mean_face_normal = np.abs(mesh.face_normals[region_face_indices]).mean(axis=0)
    cfg = compute_slice_config(region_id, up_axis, mean_face_normal=mean_face_normal)
    plane_normal: np.ndarray = cfg['plane_normal']
    slice_axis: int          = cfg['slice_axis']

    # Region bounding box along slice axis only
    region_verts = mesh.vertices[mesh.faces[region_face_indices].ravel()]
    axis_min = region_verts[:, slice_axis].min() - 0.001
    axis_max = region_verts[:, slice_axis].max() + 0.001

    span = axis_max - axis_min
    if span <= spray_width_mm:
        positions = [float((axis_min + axis_max) / 2.0)]
    else:
        first = axis_min + spray_width_mm / 2.0
        positions = [float(p) for p in np.arange(first, axis_max, spray_width_mm)]

    planes = []
    for pos in positions:
        origin = np.zeros(3, dtype=float)
        origin[slice_axis] = pos
        planes.append((plane_normal.copy(), origin, pos))

    return planes


_REGION_OUTWARD: dict[str, tuple[int, int]] = {}   # populated by _outward_sign()


def _outward_sign(region_id: str, up_axis: int) -> tuple[int, int] | None:
    """Return (axis_index, sign) for the outward-facing normal of a named region."""
    fwd_axis   = (up_axis + 1) % 3
    right_axis = (up_axis + 2) % 3
    table = {
        'TOP':    (up_axis,    +1),
        'BOTTOM': (up_axis,    -1),
        'FRONT':  (fwd_axis,  +1),
        'REAR':   (fwd_axis,  -1),
        'LEFT':   (right_axis, -1),
        'RIGHT':  (right_axis, +1),
    }
    return table.get(region_id)


def slice_region(
    mesh: trimesh.Trimesh,
    region_face_indices: np.ndarray,
    plane_normal: np.ndarray,
    plane_origin: np.ndarray,
    region_id: str = '',
    up_axis: int = 2,
) -> np.ndarray | None:
    """Intersect mesh with a plane; return only segments from the selected region.

    Returns shape (K, 2, 3) or None if no intersection within the region.

    Only keeps segments whose source face normal points toward the OUTSIDE of
    the selected region (e.g. upward for TOP), preventing paths from appearing
    on the underside of the mesh when the TOP surface is selected.
    """
    if len(region_face_indices) == 0:
        return None

    result = trimesh.intersections.mesh_plane(
        mesh,
        plane_normal=plane_normal,
        plane_origin=plane_origin,
        return_faces=True,   # CRITICAL: gives (segments, face_ids)
    )

    if result is None:
        return None

    segments, face_ids = result

    if segments is None or len(segments) == 0:
        return None

    # Filter to segments whose source triangle is in the selected region
    region_mask = np.isin(face_ids, region_face_indices)
    filtered = segments[region_mask]
    filtered_face_ids = face_ids[region_mask]

    if len(filtered) == 0:
        return None

    # Keep only segments on outward-facing faces for the named region.
    # This prevents paths on the underside when TOP is selected, or on the
    # back when FRONT is selected (the plane cuts both sides of a solid mesh).
    outward = _outward_sign(region_id, up_axis)
    if outward is not None:
        axis, sign = outward
        face_normals_here = mesh.face_normals[filtered_face_ids]
        outward_mask = (face_normals_here[:, axis] * sign) > 0.15
        if outward_mask.sum() > 0:  # only apply if filter keeps anything
            filtered = filtered[outward_mask]

    if len(filtered) == 0:
        return None

    # Remove degenerate zero-length segments
    lengths = np.linalg.norm(filtered[:, 1, :] - filtered[:, 0, :], axis=1)
    filtered = filtered[lengths > 1e-12]

    return filtered if len(filtered) > 0 else None
