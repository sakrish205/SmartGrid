"""Mesh Flow Phase 4 — path-planning-layer orientation utilities.

Formulas (quoted exactly from the spec, per the plan's formula-review gate):
  F25. Interpolated vertex normal: n = normalize(alpha*n1 + beta*n2 + gamma*n3)
  F32. Tangent projected into tool-normal plane: x_hat = normalize(t_hat - (t_hat.a_hat)a_hat)
  F33. Complete orthogonal tool frame: y_hat = normalize(a_hat x x_hat)
  F34. Rotation matrix: R = [x_hat y_hat a_hat] (columns)
  F35. Angular difference: theta = acos(clamp(a_i . a_(i+1), -1, 1))

F32-F34 are already correctly implemented, just inlined, in
app/export/olp_export.py::_get_tcp_orientation (lines 83-121: tz=-n is F30's a_hat,
tx is F32's x_hat via the same projection + Gram-Schmidt, ty=cross(tz,tx) is F33's
y_hat, R=column_stack([tx,ty,tz]) is F34). This module extracts that math into a
reusable path-planning utility; olp_export.py itself is NOT edited, so Mesh
Surface's export behavior stays frozen.
"""
from __future__ import annotations
import numpy as np
import trimesh

_NEAR_ZERO = 1e-6


def area_weighted_normal(pts: np.ndarray, mesh: trimesh.Trimesh) -> np.ndarray:
    """F25 — barycentric interpolation of VERTEX normals across the containing
    triangle, not the nearest-face normal app.path.local_normals.interpolate_normals
    already provides (that function stays nearest-face-only and untouched; this is a
    new, separate variant, not a replacement).

    Known limitation (documented, not silently assumed away): vertex normals are
    themselves an average over each vertex's adjacent faces, so this can blend
    across a genuinely sharp edge rather than preserving it — the spec's own
    implementation note for F25 ("do not blend across sharp edges or separate
    surfaces") is not enforced here; it would require edge-angle detection this
    function does not perform.
    """
    pts = np.asarray(pts, dtype=float)
    closest, _distances, triangle_ids = mesh.nearest.on_surface(pts)
    triangles = mesh.triangles[triangle_ids]                       # (N, 3, 3)
    bary = trimesh.triangles.points_to_barycentric(triangles, closest)   # (N, 3)
    vertex_ids = mesh.faces[triangle_ids]                           # (N, 3)
    vertex_normals = mesh.vertex_normals[vertex_ids]                # (N, 3, 3)
    blended = np.einsum('ij,ijk->ik', bary, vertex_normals)         # (N, 3)
    norms = np.linalg.norm(blended, axis=1, keepdims=True)
    norms = np.where(norms > _NEAR_ZERO, norms, 1.0)
    return blended / norms


def build_rotation_matrix(tangent: np.ndarray, pointing_axis: np.ndarray) -> np.ndarray:
    """F32-F34 — build R = [x_hat, y_hat, a_hat] (columns) from a path tangent and
    the gun pointing axis a_hat.

    `pointing_axis` must already be in the F30 convention (a_hat = normalize(
    q_surface - p_TCP), i.e. pointing FROM the tool INTO the surface — this is
    `-n` for an outward surface normal n, exactly what olp_export.py's `tz = -n`
    computes). This function takes pointing_axis as an explicit parameter — never
    a raw surface normal — so a caller must consciously apply that sign, rather
    than this utility silently assuming one convention.
    """
    a_hat = np.asarray(pointing_axis, dtype=float)
    a_hat = a_hat / np.linalg.norm(a_hat)
    t_hat = np.asarray(tangent, dtype=float)
    t_hat = t_hat / np.linalg.norm(t_hat)

    # F32: x_hat = normalize(t_hat - (t_hat.a_hat)a_hat)
    x_raw = t_hat - np.dot(t_hat, a_hat) * a_hat
    x_norm = np.linalg.norm(x_raw)
    if x_norm < _NEAR_ZERO:
        # Tangent (anti)parallel to the pointing axis — same fallback strategy
        # already used in olp_export.py::_get_tcp_orientation (lines 104-106).
        ref = np.array([0.0, 0.0, 1.0]) if abs(a_hat[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        x_raw = np.cross(a_hat, ref)
        x_norm = np.linalg.norm(x_raw)
    x_hat = x_raw / x_norm

    # F33: y_hat = normalize(a_hat x x_hat)
    y_hat = np.cross(a_hat, x_hat)
    y_hat = y_hat / np.linalg.norm(y_hat)

    # F34: R = [x_hat y_hat a_hat] (columns)
    return np.column_stack([x_hat, y_hat, a_hat])


def angular_difference(a_i: np.ndarray, a_j: np.ndarray) -> float:
    """F35 — theta = acos(clamp(a_i . a_j, -1, 1)), in radians.

    Generalizes the ad hoc dot<0 checks in validator.py::check_tool_flip and
    collision.py::check_orientation (which only flag a >90 degree flip) into a
    reusable continuous angle. Those two call sites are left unchanged here;
    Phase 6 is where a Mesh-Flow-aware alternative using this function is added.
    """
    a_i = np.asarray(a_i, dtype=float)
    a_j = np.asarray(a_j, dtype=float)
    dot = np.clip(np.dot(a_i / np.linalg.norm(a_i), a_j / np.linalg.norm(a_j)), -1.0, 1.0)
    return float(np.arccos(dot))
