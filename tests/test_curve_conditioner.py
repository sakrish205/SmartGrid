"""Mesh Flow Phase 4 exit-gate test: curve_conditioner's F22-bounded smoothing."""
from __future__ import annotations
import numpy as np
import trimesh

from app.meshflow.curve_conditioner import resample_and_smooth
from app.path.curvature_metrics import approximate_curvature, curvature_change_penalty, smoothness_penalty


def test_resample_and_smooth_reduces_smoothness_penalty_on_noisy_zigzag():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    # Noisy zigzag path on the +Z face, well within the mesh's surface deviation bound.
    x = np.linspace(-150, 150, 20)
    y = np.where(np.arange(20) % 2 == 0, 5.0, -5.0)
    pts = np.column_stack([x, y, np.full(20, 200.0)])

    smoothed = resample_and_smooth(pts, max_surface_deviation_mm=50.0, mesh=mesh, arc_spacing_mm=10.0)
    assert smoothness_penalty(smoothed) < smoothness_penalty(pts)


def test_resample_and_smooth_never_exceeds_surface_deviation_bound():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    x = np.linspace(-150, 150, 20)
    y = np.where(np.arange(20) % 2 == 0, 5.0, -5.0)
    pts = np.column_stack([x, y, np.full(20, 200.0)])

    bound_mm = 8.0
    smoothed = resample_and_smooth(pts, max_surface_deviation_mm=bound_mm, mesh=mesh, arc_spacing_mm=10.0)
    _closest, dists, _tri = mesh.nearest.on_surface(smoothed)
    assert dists.max() <= bound_mm + 1e-6


def test_resample_and_smooth_keeps_endpoints_fixed():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    pts = np.array([[-150.0, 0.0, 200.0], [0.0, 10.0, 200.0], [150.0, 0.0, 200.0]])
    smoothed = resample_and_smooth(pts, max_surface_deviation_mm=50.0, mesh=mesh, arc_spacing_mm=10.0)
    np.testing.assert_allclose(smoothed[0], pts[0], atol=1e-6)
    np.testing.assert_allclose(smoothed[-1], pts[-1], atol=1e-6)


def test_curvature_change_penalty_f52():
    straight = np.array([[i * 10.0, 0, 0] for i in range(10)], dtype=float)
    curvatures = approximate_curvature(straight, arc_spacing_mm=5.0)
    assert curvature_change_penalty(curvatures) == 0.0 or curvature_change_penalty(curvatures) < 1e-9

    bent = np.array([[i * 10.0, (i * 10.0) ** 1.3 * 0.01, 0] for i in range(10)], dtype=float)
    bent_curvatures = approximate_curvature(bent, arc_spacing_mm=5.0)
    assert curvature_change_penalty(bent_curvatures) >= 0.0
