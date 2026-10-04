"""First pass generation: ray-cast the outer skin and offset by standoff."""
from __future__ import annotations
import numpy as np
import trimesh


def first_pass_line(
    mesh: trimesh.Trimesh,
    outer_face_ids: np.ndarray,
    standoff_mm: float,
    pitch_mm: float,
    direction: str = 'horizontal',
    up_axis: int = 2,
    sample_spacing_mm: float | None = None,
) -> np.ndarray:
    """Return (N, 3) gun-space waypoints for the first pass on the outer skin.

    Builds a free-space sample line at pitch_mm/2 from the lower edge of the
    slicing axis, then ray-casts each sample point onto the outer skin submesh
    independently.  No plane-intersection fragments, no chain-picking.

    Raises ValueError if no rays hit the outer skin.
    """
    sub = mesh.submesh([outer_face_ids], append=True)

    # --- axis selection (unchanged) ---
    if direction == 'horizontal':
        axis = up_axis
    elif direction == 'vertical':
        remaining = [i for i in range(3) if i != up_axis]
        extents   = [float(sub.vertices[:, i].max() - sub.vertices[:, i].min())
                     for i in remaining]
        axis = remaining[int(np.argmax(extents))]
    else:
        raise ValueError(f"direction must be 'horizontal' or 'vertical', got {direction!r}")

    lo        = float(sub.vertices[:, axis].min())
    hi        = float(sub.vertices[:, axis].max())
    plane_pos = lo + pitch_mm / 2.0

    if plane_pos > hi:
        raise ValueError(
            f"first_pass_line: first plane at {plane_pos:.1f} mm exceeds "
            f"outer-skin extent {lo:.1f}-{hi:.1f} mm along axis {axis}"
        )

    spacing = sample_spacing_mm if sample_spacing_mm is not None else pitch_mm / 10.0

    # sweep axis = larger non-slicing extent; ray axis = smaller (depth direction)
    others        = [i for i in range(3) if i != axis]
    other_extents = [float(sub.vertices[:, i].max() - sub.vertices[:, i].min())
                     for i in others]
    sweep_axis = others[int(np.argmax(other_extents))]
    ray_axis   = others[int(np.argmin(other_extents))]

    sweep_lo = float(sub.vertices[:, sweep_axis].min())
    sweep_hi = float(sub.vertices[:, sweep_axis].max())
    ray_lo   = float(sub.vertices[:, ray_axis].min())
    ray_hi   = float(sub.vertices[:, ray_axis].max())
    margin   = max(1.0, (ray_hi - ray_lo) * 0.1)

    n_samples = max(2, int(np.ceil((sweep_hi - sweep_lo) / spacing)) + 1)
    sweep_vals = np.linspace(sweep_lo, sweep_hi, n_samples)

    origins = np.zeros((n_samples, 3))
    origins[:, axis]       = plane_pos
    origins[:, sweep_axis] = sweep_vals
    origins[:, ray_axis]   = ray_lo - margin          # start outside bounding box

    directions = np.zeros((n_samples, 3))
    directions[:, ray_axis] = 1.0                     # shoot inward (+ray_axis)

    print(f"first_pass_line: axis={axis} lo={lo:.1f} hi={hi:.1f} plane_pos={plane_pos:.1f}  "
          f"sweep_axis={sweep_axis} ray_axis={ray_axis}  attempted={n_samples}")

    hit_pts, index_ray, index_tri = sub.ray.intersects_location(
        origins, directions, multiple_hits=True
    )

    if len(hit_pts) == 0:
        raise ValueError(
            f"first_pass_line: no ray hits at slicing axis={axis} "
            f"plane_pos={plane_pos:.1f} mm (attempted {n_samples} rays)"
        )

    # keep closest hit per ray (smallest distance along ray direction from origin)
    along_ray = hit_pts[:, ray_axis] - origins[index_ray, ray_axis]
    order     = np.lexsort((along_ray, index_ray))          # sort by ray then distance
    s_rays  = index_ray[order]
    s_pts   = hit_pts[order]
    s_tris  = index_tri[order]

    _, first_idx = np.unique(s_rays, return_index=True)     # first (nearest) per ray
    best_pts  = s_pts[first_idx]
    best_tris = s_tris[first_idx]

    # order along sweep axis for a coherent toolpath line
    sweep_order = np.argsort(best_pts[:, sweep_axis])
    best_pts  = best_pts[sweep_order]
    best_tris = best_tris[sweep_order]

    survived = len(best_pts)
    dropped  = n_samples - survived
    print(f"first_pass_line: survived={survived}  dropped={dropped}")

    normals = sub.face_normals[best_tris]
    return best_pts + standoff_mm * normals
