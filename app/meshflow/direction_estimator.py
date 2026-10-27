"""Mesh Flow Phase 3 — direction estimator (R3).

Formulas (quoted exactly from the spec, per the plan's formula-review gate):
  F06. Project preferred direction onto reference plane: d' = d0 - (d0.n0) n0
  F07. Local scan frame: d_hat = normalize(d'); s_hat = normalize(n0 x d_hat)
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.path_model import MeshFlowRegion

_NEAR_ZERO = 1e-9


def _region_mean_normal(region: MeshFlowRegion, mesh: trimesh.Trimesh) -> np.ndarray:
    n = mesh.face_normals[region.face_indices].mean(axis=0)
    norm = np.linalg.norm(n)
    if norm < _NEAR_ZERO:
        raise ValueError(f"region {region.region_id!r}: mean face normal is near-zero/degenerate")
    return n / norm


def _pca_principal_axis(region: MeshFlowRegion, mesh: trimesh.Trimesh) -> np.ndarray:
    """Fallback d0 when no user_direction is supplied: principal axis of the region's
    face centroids (largest-eigenvalue eigenvector of their covariance). Genuinely new —
    no principal-axis estimator exists anywhere else in app/path or app/mesh today."""
    centroids = mesh.triangles_center[region.face_indices]
    centered = centroids - centroids.mean(axis=0)
    cov = centered.T @ centered
    eigvals, eigvecs = np.linalg.eigh(cov)
    return eigvecs[:, np.argmax(eigvals)]   # eigh sorts ascending — last column is dominant


def local_scan_frame(
    region: MeshFlowRegion,
    mesh: trimesh.Trimesh,
    user_direction: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """F06 + F07 — returns (d_hat, s_hat), the region-local primary pass direction and
    perpendicular spacing direction.

    If user_direction is supplied, it is F06-projected onto the region's mean-normal
    plane. Otherwise d0 falls back to the PCA principal axis of the region's face
    centroids (F06's own implementation note: "if d' is near zero, choose another
    preferred direction" — handled below by retrying with a different d0 candidate).
    """
    n0 = _region_mean_normal(region, mesh)

    if user_direction is not None:
        d0 = np.asarray(user_direction, dtype=float)
        d0_norm = np.linalg.norm(d0)
        if d0_norm < _NEAR_ZERO:
            raise ValueError("user_direction must be a nonzero vector")
        d0 = d0 / d0_norm
    else:
        d0 = _pca_principal_axis(region, mesh)

    # F06: d' = d0 - (d0.n0) n0
    d_prime = d0 - np.dot(d0, n0) * n0
    d_prime_norm = np.linalg.norm(d_prime)

    if d_prime_norm < _NEAR_ZERO:
        # d0 was (anti)parallel to n0 — F06's own note: choose another preferred
        # direction. Retry once with an arbitrary vector not parallel to n0.
        fallback = np.array([0.0, 0.0, 1.0]) if abs(n0[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        d_prime = fallback - np.dot(fallback, n0) * n0
        d_prime_norm = np.linalg.norm(d_prime)
        if d_prime_norm < _NEAR_ZERO:
            raise ValueError(f"region {region.region_id!r}: could not establish a non-degenerate scan frame")

    # F07: d_hat = normalize(d'); s_hat = normalize(n0 x d_hat)
    d_hat = d_prime / d_prime_norm
    s_hat = np.cross(n0, d_hat)
    s_hat = s_hat / np.linalg.norm(s_hat)
    return d_hat, s_hat


def estimate_primary_direction(
    region: MeshFlowRegion,
    mesh: trimesh.Trimesh,
    user_direction: np.ndarray | None = None,
) -> np.ndarray:
    """R3 — the region's primary pass direction (F06/F07's d_hat)."""
    d_hat, _s_hat = local_scan_frame(region, mesh, user_direction)
    return d_hat
