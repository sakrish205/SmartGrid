"""Path quality validation: blocking checks and reporting metrics.

Blocking (raise ValueError):
  check_straightness  — max lateral deviation from start→end line
  check_collision     — any waypoint inside the mesh

Reporting (return float / dict — no exception):
  measure_standoff_deviation  — max |actual_dist - nominal|
  measure_surface_pitch       — surface-relative distance between adjacent passes
  measure_coverage            — fraction of region faces within spray_width/2 of any pass
  measure_boundary_gap        — distance from boundary edges to nearest pass waypoint
  check_tool_flip             — True if any consecutive normals flip >90°

Boundary detection concept from BF detectBoundary.m:
  edges shared by exactly 1 face = open boundary.
"""
from __future__ import annotations
import numpy as np
import trimesh
import trimesh.proximity as _prox
from scipy.spatial import cKDTree

from app.path.path_model import PaintPass, PaintRoute

# Blocking tolerance — path rejected if exceeded
_STRAIGHTNESS_TOL_MM = 5.0


# ---------------------------------------------------------------------------
# Blocking
# ---------------------------------------------------------------------------

def check_straightness(p: PaintPass, tol_mm: float = _STRAIGHTNESS_TOL_MM) -> None:
    """Raise ValueError if any waypoint deviates more than tol_mm from the
    straight line between the first and last point of the pass."""
    pts = p.points
    if len(pts) < 3:
        return
    axis = pts[-1] - pts[0]
    length = float(np.linalg.norm(axis))
    if length < 1e-9:
        return
    axis_unit = axis / length
    # Lateral deviation = |pt - proj| where proj is the foot on the axis line
    vecs = pts - pts[0]
    proj_len = vecs @ axis_unit
    proj_pts = pts[0] + np.outer(proj_len, axis_unit)
    deviations = np.linalg.norm(pts - proj_pts, axis=1)
    max_dev = float(deviations.max())
    if max_dev > tol_mm:
        raise ValueError(
            f"check_straightness: pass {p.id} (region={p.region_id}) "
            f"max lateral deviation {max_dev:.2f} mm > {tol_mm:.1f} mm tolerance"
        )


def check_collision(p: PaintPass, mesh: trimesh.Trimesh) -> None:
    """Raise ValueError if any waypoint is inside the mesh.
    Only valid on watertight meshes; silently skips open meshes."""
    if not mesh.is_watertight or len(p.points) == 0:
        return
    _CHUNK = 500
    inside = np.zeros(len(p.points), dtype=bool)
    for s in range(0, len(p.points), _CHUNK):
        inside[s:s + _CHUNK] = mesh.contains(p.points[s:s + _CHUNK])
    if inside.any():
        raise ValueError(
            f"check_collision: pass {p.id} (region={p.region_id}) "
            f"has {inside.sum()} waypoint(s) inside the mesh"
        )


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def measure_standoff_deviation(
    p: PaintPass,
    mesh: trimesh.Trimesh,
    nominal_standoff_mm: float,
) -> float:
    """Return max |actual_distance_to_surface - nominal_standoff_mm| in mm."""
    if len(p.points) == 0 or nominal_standoff_mm <= 0:
        return 0.0
    _, dists, _ = _prox.closest_point(mesh, p.points)
    return float(np.abs(dists - nominal_standoff_mm).max())


def measure_surface_pitch(
    passes: list[PaintPass],
    mesh: trimesh.Trimesh,
) -> dict:
    """Return {'min_mm', 'max_mm', 'mean_mm'} of surface-relative spacing
    between all adjacent pass pairs.

    Surface-relative pitch = closest distance between waypoints of pass i and
    pass i+1, measured along the mesh surface (approximated by closest-point
    distance in 3D).  Purely XYZ pitch is misleading on curved surfaces.
    """
    primary = [p for p in passes if p.sub_index == 0]
    if len(primary) < 2:
        return {'min_mm': 0.0, 'max_mm': 0.0, 'mean_mm': 0.0}

    pitches: list[float] = []
    for a, b in zip(primary, primary[1:]):
        if len(a.points) == 0 or len(b.points) == 0:
            continue
        tree = cKDTree(b.points)
        dists, _ = tree.query(a.points, k=1)
        pitches.append(float(dists.min()))

    if not pitches:
        return {'min_mm': 0.0, 'max_mm': 0.0, 'mean_mm': 0.0}
    return {
        'min_mm':  float(np.min(pitches)),
        'max_mm':  float(np.max(pitches)),
        'mean_mm': float(np.mean(pitches)),
    }


def measure_coverage(
    passes: list[PaintPass],
    mesh: trimesh.Trimesh,
    region_face_ids: np.ndarray,
    spray_width_mm: float,
) -> float:
    """Return fraction [0, 1] of region face centroids within spray_width_mm / 2
    of any pass waypoint.  Inspired by BF's coating-uniformity model."""
    if len(region_face_ids) == 0 or spray_width_mm <= 0:
        return 0.0

    all_pts = np.vstack([p.points for p in passes if len(p.points) > 0])
    if len(all_pts) == 0:
        return 0.0

    centroids = mesh.triangles_center[region_face_ids]
    tree = cKDTree(all_pts)
    dists, _ = tree.query(centroids, k=1)
    covered = (dists <= spray_width_mm / 2.0).sum()
    return float(covered) / len(centroids)


def measure_boundary_gap(
    passes: list[PaintPass],
    mesh: trimesh.Trimesh,
) -> float:
    """Return the max distance from any open-boundary vertex to the nearest
    pass waypoint.  Boundary detection adapted from BF detectBoundary.m:
    edges shared by exactly 1 face are open boundary edges."""
    if not hasattr(mesh, 'faces_unique_edges'):
        return 0.0

    # Boundary edges: face_unique_edge index appears exactly once across all faces
    edge_counts = np.bincount(
        mesh.faces_unique_edges.ravel(),
        minlength=len(mesh.edges_unique),
    )
    boundary_mask = edge_counts == 1
    if not boundary_mask.any():
        return 0.0

    boundary_verts = mesh.vertices[mesh.edges_unique[boundary_mask].ravel()]

    all_pts = np.vstack([p.points for p in passes if len(p.points) > 0])
    if len(all_pts) == 0:
        return 0.0

    tree = cKDTree(all_pts)
    dists, _ = tree.query(boundary_verts, k=1)
    return float(dists.max())


def check_tool_flip(p: PaintPass) -> bool:
    """Return True if any consecutive surface normals flip more than 90°.
    A flip indicates an orientation discontinuity — the robot would need to
    rotate the tool more than 90° between adjacent waypoints."""
    if p.normals is None or len(p.normals) < 2:
        return False
    dots = np.einsum('ij,ij->i', p.normals[:-1], p.normals[1:])
    return bool((dots < 0.0).any())


# ---------------------------------------------------------------------------
# Convenience: validate a full route, return structured report
# ---------------------------------------------------------------------------

def validate_route(
    route: PaintRoute,
    mesh: trimesh.Trimesh,
    nominal_standoff_mm: float,
    spray_width_mm: float,
    region_face_ids: np.ndarray | None = None,
    straightness_tol_mm: float = _STRAIGHTNESS_TOL_MM,
) -> dict:
    """Run all checks on a route. Raises on hard failures; returns metrics dict.

    Returns:
        {
          'standoff_deviation_mm': float,   # max over all passes
          'surface_pitch': {...},
          'coverage_fraction': float,       # requires region_face_ids
          'boundary_gap_mm': float,
          'tool_flips': [pass_id, ...],
        }
    """
    for p in route.passes:
        check_straightness(p, straightness_tol_mm)
        check_collision(p, mesh)

    max_dev = max(
        (measure_standoff_deviation(p, mesh, nominal_standoff_mm) for p in route.passes),
        default=0.0,
    )
    pitch = measure_surface_pitch(route.passes, mesh)
    coverage = (
        measure_coverage(route.passes, mesh, region_face_ids, spray_width_mm)
        if region_face_ids is not None else None
    )
    gap = measure_boundary_gap(route.passes, mesh)
    flips = [p.id for p in route.passes if check_tool_flip(p)]

    return {
        'standoff_deviation_mm': max_dev,
        'surface_pitch':         pitch,
        'coverage_fraction':     coverage,
        'boundary_gap_mm':       gap,
        'tool_flips':            flips,
    }
