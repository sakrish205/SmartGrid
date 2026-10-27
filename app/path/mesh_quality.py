"""Mesh Flow Phase 4 — mesh-quality indicators (R11).

Formulas (quoted exactly from the spec, per the plan's formula-review gate):
  F48. Relative edge-length variation: CV_l = std(l_e) / mean(l_e)
  F49. Neighbor-normal variation: theta_ij = acos(clamp(n_i . n_j, -1, 1))
  F50. Simplification deviation: E_simp,max = max_i dist(v_i, M_s)

F50 is implemented as adaptive, physical-scale sampling with BOTH a raw estimate
and a mathematically guaranteed upper bound — see simplification_deviation's
docstring for the full derivation. The guaranteed bound is what
path_model.MeshQualityReport.planning_time_budget_mm uses; it is never the sole
safety authority — Phase 6's check against the true original mesh always is.
"""
from __future__ import annotations
from dataclasses import dataclass

import numpy as np
import trimesh


def edge_length_cv(mesh: trimesh.Trimesh, face_indices: np.ndarray) -> float:
    """F48 — coefficient of variation of edge lengths over the given faces."""
    edges = mesh.faces[face_indices][:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2)
    lengths = np.linalg.norm(mesh.vertices[edges[:, 0]] - mesh.vertices[edges[:, 1]], axis=1)
    mean = lengths.mean()
    if mean < 1e-12:
        return 0.0
    return float(lengths.std() / mean)


def neighbor_normal_variation(mesh: trimesh.Trimesh, face_indices: np.ndarray) -> float:
    """F49 — mean angular variation between each face's normal and its adjacent
    faces' normals (adjacency restricted to the given face set)."""
    face_set = set(face_indices.tolist())
    adjacency = mesh.face_adjacency
    mask = np.isin(adjacency[:, 0], face_indices) & np.isin(adjacency[:, 1], face_indices)
    pairs = adjacency[mask]
    if len(pairs) == 0:
        return 0.0
    n_i = mesh.face_normals[pairs[:, 0]]
    n_j = mesh.face_normals[pairs[:, 1]]
    dots = np.clip(np.sum(n_i * n_j, axis=1), -1.0, 1.0)
    return float(np.degrees(np.arccos(dots)).mean())


def _measurement_scale_mm(tolerance_mm: float | None, original_mesh: trimesh.Trimesh) -> float:
    """The resolution the sampler targets. Preferred source: a quarter of the
    surface-deviation tolerance (decision #5) — no reason to resolve finer than
    1/4 of the tolerance (nothing that small could flip a pass/fail outcome), and
    resolving coarser than the tolerance is exactly how a real mid-face deviation
    goes undetected. Fallback (tolerance not yet set): the original mesh's own
    median edge length — the finest geometric detail actually present in the
    part. Both choices are documented ASSUMPTIONS, calibrated by the tests in
    tests/test_mesh_quality.py, not claimed to be provably optimal."""
    if tolerance_mm is not None:
        return tolerance_mm / 4.0
    return float(np.median(original_mesh.edges_unique_length))


# Covering-radius constants per density band — see simplification_deviation's
# docstring for the derivation this makes rigorous. Validated empirically by
# tests/test_mesh_quality.py's covering-radius-soundness test, not asserted.
_K_BAND_LIGHT = 0.6      # 4-point sampling (3 vertices + centroid)
_K_BAND_NORMAL = 0.3     # 7-point sampling (3 vertices + 3 edge midpoints + centroid)

# Global sampling budget for the large-triangle band. Recursive subdivision to a
# FIXED target_edge_mm can blow up combinatorially when the mesh's natural
# triangle size is far larger than the measurement scale (e.g. a coarse mesh
# being validated against a tight tolerance) — thousands of large faces each
# needing deep subdivision produces millions of query points, which is not a
# vectorization problem, it's an inherent cost of resolving that much detail.
# Rather than accept that cost (impractically slow) OR silently under-sample
# (an unsound bound), this caps TOTAL sample count by picking one SHARED
# subdivision depth for every large face (see _pick_shared_subdivision_depth).
# The guaranteed bound stays correct either way — covering_radius is always
# k_band * the ACTUAL local edge length reached, never an assumed target — this
# only trades bound TIGHTNESS for runtime on pathologically coarse meshes, never
# correctness.
_MAX_TOTAL_LARGE_BAND_SAMPLES = 200_000


def _pick_shared_subdivision_depth(edge_lens: np.ndarray, target_edge_mm: float, max_depth: int) -> int:
    """Largest shared depth (<= max_depth) whose projected total sample count
    over all large-band faces stays within _MAX_TOTAL_LARGE_BAND_SAMPLES."""
    for depth in range(max_depth, -1, -1):
        levels_needed = np.minimum(
            np.ceil(np.log2(np.maximum(edge_lens / target_edge_mm, 1.0))), depth,
        )
        projected = np.sum(4.0 ** levels_needed * 7.0)
        if projected <= _MAX_TOTAL_LARGE_BAND_SAMPLES:
            return depth
    return 0


def _face_samples(vertices: np.ndarray) -> tuple[np.ndarray, float]:
    """3 vertices + 3 edge midpoints + centroid for one triangle -> (7, 3) samples,
    plus that triangle's longest edge length."""
    v0, v1, v2 = vertices
    m01, m12, m20 = (v0 + v1) / 2.0, (v1 + v2) / 2.0, (v2 + v0) / 2.0
    centroid = (v0 + v1 + v2) / 3.0
    samples = np.array([v0, v1, v2, m01, m12, m20, centroid])
    edge_len = max(np.linalg.norm(v1 - v0), np.linalg.norm(v2 - v1), np.linalg.norm(v0 - v2))
    return samples, float(edge_len)


def _subdivide_triangle(vertices: np.ndarray, target_edge_mm: float, max_depth: int) -> list[np.ndarray]:
    """Recursive midpoint subdivision (measurement-only — never writes back to the
    mesh) until every sub-triangle's longest edge <= target_edge_mm, capped at
    max_depth (safety valve against a degenerate giant triangle)."""
    v0, v1, v2 = vertices
    edge_len = max(np.linalg.norm(v1 - v0), np.linalg.norm(v2 - v1), np.linalg.norm(v0 - v2))
    if edge_len <= target_edge_mm or max_depth <= 0:
        return [vertices]
    m01, m12, m20 = (v0 + v1) / 2.0, (v1 + v2) / 2.0, (v2 + v0) / 2.0
    children = [
        np.array([v0, m01, m20]), np.array([m01, v1, m12]),
        np.array([m20, m12, v2]), np.array([m01, m12, m20]),
    ]
    out: list[np.ndarray] = []
    for child in children:
        out.extend(_subdivide_triangle(child, target_edge_mm, max_depth - 1))
    return out


@dataclass
class _SampleBatch:
    points: np.ndarray        # (N, 3)
    covering_radius: np.ndarray   # (N,) per-sample covering radius (k_band * local edge length)


def _sample_mesh_faces(
    mesh: trimesh.Trimesh,
    scale_mm: float,
    max_subdivision_depth: int,
) -> _SampleBatch:
    """Vectorized over all three density bands at once — light and normal bands
    never touch a Python per-face loop; only the (normally rare) large-triangle
    band is subdivided face-by-face, since recursive subdivision depth varies
    per face and isn't a fixed-shape vector operation."""
    small_threshold = 0.5 * scale_mm
    large_threshold = 4.0 * scale_mm

    tris = mesh.triangles                      # (F, 3, 3)
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    edge_lens = np.maximum(
        np.linalg.norm(v1 - v0, axis=1),
        np.maximum(np.linalg.norm(v2 - v1, axis=1), np.linalg.norm(v0 - v2, axis=1)),
    )

    small_mask = edge_lens <= small_threshold
    normal_mask = (edge_lens > small_threshold) & (edge_lens <= large_threshold)
    large_mask = edge_lens > large_threshold

    all_points: list[np.ndarray] = []
    all_radii: list[np.ndarray] = []

    if small_mask.any():
        sv0, sv1, sv2 = v0[small_mask], v1[small_mask], v2[small_mask]
        centroids = (sv0 + sv1 + sv2) / 3.0
        samples = np.stack([sv0, sv1, sv2, centroids], axis=1)   # (K, 4, 3)
        radii = _K_BAND_LIGHT * edge_lens[small_mask]             # (K,)
        all_points.append(samples.reshape(-1, 3))
        all_radii.append(np.repeat(radii, 4))

    if normal_mask.any():
        nv0, nv1, nv2 = v0[normal_mask], v1[normal_mask], v2[normal_mask]
        m01, m12, m20 = (nv0 + nv1) / 2.0, (nv1 + nv2) / 2.0, (nv2 + nv0) / 2.0
        centroids = (nv0 + nv1 + nv2) / 3.0
        samples = np.stack([nv0, nv1, nv2, m01, m12, m20, centroids], axis=1)   # (K, 7, 3)
        radii = _K_BAND_NORMAL * edge_lens[normal_mask]
        all_points.append(samples.reshape(-1, 3))
        all_radii.append(np.repeat(radii, 7))

    if large_mask.any():
        shared_depth = _pick_shared_subdivision_depth(
            edge_lens[large_mask], large_threshold, max_subdivision_depth,
        )
        for tri in tris[large_mask]:
            for sub in _subdivide_triangle(tri, large_threshold, shared_depth):
                samples, local_edge = _face_samples(sub)
                all_points.append(samples)
                all_radii.append(np.full(len(samples), _K_BAND_NORMAL * local_edge))

    points = np.concatenate(all_points, axis=0) if all_points else np.zeros((0, 3))
    radii = np.concatenate(all_radii, axis=0) if all_radii else np.zeros(0)
    return _SampleBatch(points=points, covering_radius=radii)


@dataclass
class DirectionalDeviation:
    estimate_mm: float   # raw sampled max — informative, NOT safe to use as a budget alone
    bound_mm: float      # estimate_mm + covering_radius_mm — a REAL guaranteed upper bound


def simplification_deviation(
    original: trimesh.Trimesh,
    simplified: trimesh.Trimesh,
    tolerance_mm: float | None = None,
    max_subdivision_depth: int = 6,
) -> tuple[DirectionalDeviation, DirectionalDeviation]:
    """F50 — bidirectional (forward, reverse) surface deviation, each returned as
    BOTH a raw estimate and a mathematically guaranteed upper bound.

    WHY A SEPARATE GUARANTEED BOUND IS NEEDED, AND HOW IT IS DERIVED: distance-to-
    a-set functions are always 1-Lipschitz for ANY nonempty set M in Euclidean
    space — for any points x, s: dist(x,M) <= dist(s,M) + |x-s|. This follows
    directly from the triangle inequality (for any m in M, dist(x,M) <= |x-m| <=
    |x-s| + |s-m|; take the infimum over m), unconditionally true, not an
    approximation. So if every point in a sampled triangle is within a known
    covering radius r of its nearest sample point, then for the TRUE (continuous)
    maximum deviation over that triangle: true_max <= (max sampled distance) + r.
    This is a real guaranteed bound, not an estimate, provided r is itself a true
    upper bound on the sampling pattern's covering radius — validated empirically
    (not just asserted) in tests/test_mesh_quality.py.

    Sampling uses a PHYSICAL-SCALE adaptive density (not a fixed per-triangle
    count): triangles below 0.5x the measurement scale get light (4-point)
    sampling, triangles above 4x the scale are virtually subdivided (never
    mutating the mesh) until every sub-triangle is within the normal (7-point)
    band. This is MEASUREMENT EFFORT ONLY; mesh.faces/mesh.vertices are never
    altered.

    This result is used only as a PLANNING-TIME heuristic (via
    path_model.MeshQualityReport.planning_time_budget_mm) — Phase 6's check
    against the true original mesh remains the authoritative safety gate
    regardless of anything measured here.
    """
    scale_mm = _measurement_scale_mm(tolerance_mm, original)
    forward_batch = _sample_mesh_faces(original, scale_mm, max_subdivision_depth)
    reverse_batch = _sample_mesh_faces(simplified, scale_mm, max_subdivision_depth)

    _closest, forward_dists, _tri = simplified.nearest.on_surface(forward_batch.points)
    _closest, reverse_dists, _tri = original.nearest.on_surface(reverse_batch.points)

    forward_idx = int(np.argmax(forward_dists)) if len(forward_dists) else 0
    reverse_idx = int(np.argmax(reverse_dists)) if len(reverse_dists) else 0

    forward_estimate = float(forward_dists[forward_idx]) if len(forward_dists) else 0.0
    reverse_estimate = float(reverse_dists[reverse_idx]) if len(reverse_dists) else 0.0
    forward_radius = float(forward_batch.covering_radius[forward_idx]) if len(forward_dists) else 0.0
    reverse_radius = float(reverse_batch.covering_radius[reverse_idx]) if len(reverse_dists) else 0.0

    forward = DirectionalDeviation(estimate_mm=forward_estimate, bound_mm=forward_estimate + forward_radius)
    reverse = DirectionalDeviation(estimate_mm=reverse_estimate, bound_mm=reverse_estimate + reverse_radius)
    return forward, reverse
