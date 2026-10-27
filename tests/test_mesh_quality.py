"""Mesh Flow Phase 4 exit-gate tests: F48/F49/F50 mesh-quality indicators,
focused on validating the F50 guaranteed-bound claim (not just the estimate).
"""
from __future__ import annotations
import numpy as np
import pytest
import trimesh

from app.path.mesh_quality import (
    _K_BAND_LIGHT,
    _K_BAND_NORMAL,
    _face_samples,
    _sample_mesh_faces,
    edge_length_cv,
    neighbor_normal_variation,
    simplification_deviation,
)


def _decimated_pair(subdivisions: int, radius: float, keep_fraction: float):
    original = trimesh.creation.icosphere(subdivisions=subdivisions, radius=radius)
    simplified = original.simplify_quadric_decimation(
        face_count=max(8, int(len(original.faces) * keep_fraction)),
    )
    return original, simplified


def test_f48_edge_length_cv_zero_on_uniform_mesh():
    # A regular icosphere has near-uniform edge lengths before decimation.
    mesh = trimesh.creation.icosphere(subdivisions=2, radius=10.0)
    cv = edge_length_cv(mesh, np.arange(len(mesh.faces)))
    assert 0.0 <= cv < 0.2


def test_f49_neighbor_normal_variation_higher_on_coarse_mesh():
    fine = trimesh.creation.icosphere(subdivisions=3, radius=10.0)
    coarse = trimesh.creation.icosphere(subdivisions=1, radius=10.0)
    fine_var = neighbor_normal_variation(fine, np.arange(len(fine.faces)))
    coarse_var = neighbor_normal_variation(coarse, np.arange(len(coarse.faces)))
    assert coarse_var > fine_var   # fewer, bigger facets bend more sharply between neighbors


def test_f50_bound_is_never_smaller_than_estimate():
    original, simplified = _decimated_pair(subdivisions=2, radius=20.0, keep_fraction=0.3)
    fwd, rev = simplification_deviation(original, simplified, tolerance_mm=2.0)
    assert fwd.bound_mm >= fwd.estimate_mm
    assert rev.bound_mm >= rev.estimate_mm


def test_f50_measurement_never_mutates_the_mesh():
    original, simplified = _decimated_pair(subdivisions=2, radius=20.0, keep_fraction=0.3)
    orig_verts = original.vertices.copy()
    orig_faces = original.faces.copy()
    simplification_deviation(original, simplified, tolerance_mm=2.0)
    np.testing.assert_array_equal(original.vertices, orig_verts)
    np.testing.assert_array_equal(original.faces, orig_faces)


def test_f50_finer_tolerance_scale_never_decreases_measured_deviation():
    """Monotonicity: a smaller measurement scale (denser sampling) must never
    find LESS deviation than a coarser one on the same mesh pair."""
    original, simplified = _decimated_pair(subdivisions=2, radius=20.0, keep_fraction=0.3)
    coarse_fwd, coarse_rev = simplification_deviation(original, simplified, tolerance_mm=20.0)
    fine_fwd, fine_rev = simplification_deviation(original, simplified, tolerance_mm=2.0)
    assert fine_fwd.estimate_mm >= coarse_fwd.estimate_mm - 1e-9
    assert fine_rev.estimate_mm >= coarse_rev.estimate_mm - 1e-9


def test_covering_radius_bounds_actual_sample_gaps():
    """Covering-radius soundness: for the 7-point normal band, no point in the
    triangle should be farther from its nearest of the 7 samples than
    k_band_normal * edge_length — checked by brute-force dense query grid."""
    rng = np.random.default_rng(0)
    for _ in range(20):
        tri = rng.uniform(-10, 10, size=(3, 3))
        samples, edge_len = _face_samples(tri)
        radius = _K_BAND_NORMAL * edge_len

        # Dense barycentric grid of query points across the triangle interior.
        a, b = np.meshgrid(np.linspace(0, 1, 15), np.linspace(0, 1, 15))
        mask = (a + b) <= 1.0
        u, v = a[mask], b[mask]
        query = tri[0] + u[:, None] * (tri[1] - tri[0]) + v[:, None] * (tri[2] - tri[0])

        dists = np.linalg.norm(query[:, None, :] - samples[None, :, :], axis=2).min(axis=1)
        assert dists.max() <= radius + 1e-9, (
            f"covering radius {radius:.4f} violated by {dists.max():.4f}"
        )


def test_covering_radius_light_band_bounds_actual_sample_gaps():
    rng = np.random.default_rng(1)
    for _ in range(20):
        tri = rng.uniform(-10, 10, size=(3, 3))
        v0, v1, v2 = tri
        centroid = (v0 + v1 + v2) / 3.0
        samples = np.array([v0, v1, v2, centroid])
        edge_len = max(np.linalg.norm(v1 - v0), np.linalg.norm(v2 - v1), np.linalg.norm(v0 - v2))
        radius = _K_BAND_LIGHT * edge_len

        a, b = np.meshgrid(np.linspace(0, 1, 15), np.linspace(0, 1, 15))
        mask = (a + b) <= 1.0
        u, v = a[mask], b[mask]
        query = v0 + u[:, None] * (v1 - v0) + v[:, None] * (v2 - v0)

        dists = np.linalg.norm(query[:, None, :] - samples[None, :, :], axis=2).min(axis=1)
        assert dists.max() <= radius + 1e-9


def test_large_triangle_sample_budget_keeps_runtime_bounded():
    """A mesh whose triangle size is far larger than the measurement scale must
    not explode into millions of samples — the shared-depth budget caps total
    sample count, trading bound tightness for runtime, never correctness."""
    big = trimesh.creation.icosphere(subdivisions=2, radius=500.0)
    scale_mm = 1.0   # deliberately far finer than this mesh's natural triangle size
    batch = _sample_mesh_faces(big, scale_mm, max_subdivision_depth=6)
    assert len(batch.points) < 500_000   # bounded, not combinatorially exploded
    assert np.all(batch.covering_radius > 0)   # bound still meaningful, just looser


def test_degenerate_zero_area_face_does_not_produce_nan_bound():
    mesh = trimesh.creation.box()
    # Inject a degenerate (zero-area) face by duplicating a vertex.
    faces = mesh.faces.copy()
    faces[0] = [faces[0][0], faces[0][0], faces[0][1]]
    degenerate = trimesh.Trimesh(vertices=mesh.vertices, faces=faces, process=False)
    simplified = mesh.copy()

    fwd, rev = simplification_deviation(degenerate, simplified, tolerance_mm=2.0)
    assert np.isfinite(fwd.estimate_mm) and np.isfinite(fwd.bound_mm)
    assert np.isfinite(rev.estimate_mm) and np.isfinite(rev.bound_mm)
