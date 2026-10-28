"""Mesh Flow Phase 3 exit-gate tests: region manager, direction/lane planning,
and the minimal core generator. Mesh Surface's generate_route() is untouched by
any of this — see tests/test_mesh_surface_baseline.py for that guarantee.
"""
from __future__ import annotations
import numpy as np
import pytest
import trimesh

from app.mesh.preprocessor import preprocess
from app.meshflow.core_generator import generate_mesh_flow_route
from app.meshflow.lane_planner import resolve_lane_pitch
from app.meshflow.open_decisions import LanePitchStrategy, OpenDecisionBlocked
from app.meshflow.region_manager import build_regions
from app.path import local_normals, surface_safety
from app.path.path_model import MeshFlowRegion, RegionConfidence


@pytest.fixture
def flat_panel_mesh():
    # Subdivided so face centroids give a representative PCA direction — a bare
    # 2-triangle box face is degenerate for principal-axis estimation.
    box = trimesh.creation.box(extents=[200, 300, 100])
    return box.subdivide().subdivide()


def test_build_regions_accepted_for_clean_flat_region(flat_panel_mesh):
    data = preprocess(flat_panel_mesh, 'panel.stl', up_axis=2)
    regions = build_regions(data, ['TOP'])
    assert len(regions) == 1
    assert regions[0].confidence is RegionConfidence.ACCEPTED   # spec §15.1 row 1


def test_generate_mesh_flow_route_serpentine_alternation(flat_panel_mesh):
    data = preprocess(flat_panel_mesh, 'panel.stl', up_axis=2)
    region = build_regions(data, ['TOP'])[0]

    route = generate_mesh_flow_route(
        data, region,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=30.0,
    )
    assert route.total_passes >= 2
    forwards = [p.is_forward for p in route.passes]
    assert forwards == [i % 2 == 0 for i in range(len(forwards))]   # F41 alternation
    for p in route.passes:
        assert len(p.points) >= 2


def test_generate_mesh_flow_route_on_curved_region():
    cyl = trimesh.creation.cylinder(radius=50.0, height=200.0, sections=64)
    data = preprocess(cyl, 'cyl.stl', up_axis=2)
    region = build_regions(data, ['FRONT'])[0]

    route = generate_mesh_flow_route(
        data, region,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=20.0,
    )
    assert route.total_passes > 0


def test_generate_mesh_flow_route_applies_standoff_and_normals(flat_panel_mesh):
    """F59/C1 — the TCP belongs off the surface, not on it (fixes the false
    near-miss flag every Mesh Flow pass used to trip once the background
    collision check ran against a configured standoff > 0)."""
    data = preprocess(flat_panel_mesh, 'panel.stl', up_axis=2)
    region = build_regions(data, ['TOP'])[0]

    route = generate_mesh_flow_route(
        data, region,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=30.0, standoff_mm=10.0,
    )
    assert route.total_passes > 0
    for p in route.passes:
        assert p.normals is not None
        assert p.normals.shape == p.points.shape
        _closest, dists, _tri = flat_panel_mesh.nearest.on_surface(p.points)
        assert np.allclose(dists, 10.0, atol=1e-6)


def test_generate_mesh_flow_route_zero_standoff_leaves_points_on_surface(flat_panel_mesh):
    data = preprocess(flat_panel_mesh, 'panel.stl', up_axis=2)
    region = build_regions(data, ['TOP'])[0]

    route = generate_mesh_flow_route(
        data, region,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=30.0,
    )
    for p in route.passes:
        assert p.normals is None


def test_generate_mesh_flow_route_standoff_never_lands_inside_curved_mesh():
    """Regression check for 'half inside, half outside' on curved geometry:
    a single region-mean-normal offset could place a waypoint on the wrong
    side of a curved surface. Per-point local normals, restricted to the
    region's own submesh, must never do that."""
    cyl = trimesh.creation.cylinder(radius=50.0, height=200.0, sections=64)
    data = preprocess(cyl, 'cyl.stl', up_axis=2)
    region = build_regions(data, ['FRONT'])[0]

    route = generate_mesh_flow_route(
        data, region,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=20.0, standoff_mm=10.0,
    )
    assert route.total_passes > 0
    all_points = np.concatenate([p.points for p in route.passes], axis=0)
    inside = surface_safety.inside_mesh_mask(all_points, cyl)
    assert inside is not None   # a trimesh.creation.cylinder is watertight
    assert not inside.any(), "some offset waypoints landed inside the mesh"


def test_generate_mesh_flow_route_standoff_distance_accurate_on_curved_mesh():
    cyl = trimesh.creation.cylinder(radius=50.0, height=200.0, sections=64)
    data = preprocess(cyl, 'cyl.stl', up_axis=2)
    region = build_regions(data, ['FRONT'])[0]

    route = generate_mesh_flow_route(
        data, region,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=20.0, standoff_mm=10.0,
    )
    for p in route.passes:
        _closest, dists, _tri = cyl.nearest.on_surface(p.points)
        assert np.allclose(dists, 10.0, atol=1.0), (
            f"standoff deviation too large on curved surface: {dists}"
        )


def test_interpolate_normals_restricted_to_region_ignores_a_closer_decoy_part():
    """Fix B, isolated at the mechanism level: on a multi-part assembly, a
    query point that happens to sit closer to an unrelated nearby part than
    to its own surface (exactly what resampling/simplification can produce
    near a part boundary) must NOT have its normal hijacked by that decoy
    once the lookup is restricted to the region's own submesh — this is the
    actual guarantee core_generator relies on by building region_mesh from
    region.face_indices before calling interpolate_normals."""
    panel = trimesh.creation.box(extents=[100.0, 100.0, 10.0])   # top face at z=5
    decoy = trimesh.creation.box(extents=[4.0, 4.0, 4.0])
    decoy.apply_translation([0.0, 0.0, 8.0])                      # decoy spans z=[6, 10]
    assembly = trimesh.util.concatenate([panel, decoy])

    # 0.6mm above the panel's own top surface, 0.4mm below the decoy's
    # bottom face -- closer to the decoy in a naive whole-mesh search.
    query_point = np.array([[0.0, 0.0, 5.6]])

    whole_mesh_normal = local_normals.interpolate_normals(query_point, assembly)[0]
    assert whole_mesh_normal[2] < 0, (
        "test setup sanity check failed: expected the decoy's downward-facing "
        "bottom normal to win the naive whole-mesh search"
    )

    restricted_normal = local_normals.interpolate_normals(query_point, panel)[0]
    assert restricted_normal[2] > 0.9, (
        "restricting the lookup to the panel's own submesh must always return "
        "the panel's own (upward) normal, never the decoy's"
    )


def test_generate_mesh_flow_route_self_corrects_a_partially_flipped_curved_region():
    """Fix C, honestly scoped: a local signed-distance check can only ever
    correct a normal that DISAGREES with its geometric neighborhood — on flat
    geometry, offsetting straight along even a wrong normal re-finds the same
    triangle (no disagreement possible, proven by direct reproduction: a
    whole-region flip on a flat box left every point ~10mm inside, undetected,
    because the points landed inside is_watertight's own boundary-tie blind
    spot, not because Fix C worked). Curvature is what breaks that symmetry —
    flip a PARTIAL subset of a curved region's faces (leaving real,
    correctly-oriented neighbors nearby) and confirm the large majority of
    waypoints end up correctly radially-outward anyway."""
    cyl = trimesh.creation.cylinder(radius=50.0, height=200.0, sections=64)
    data = preprocess(cyl, 'cyl.stl', up_axis=2)
    region = build_regions(data, ['FRONT'])[0]
    face_indices = region.face_indices.copy()

    faces = cyl.faces.copy()
    flip_set = face_indices[::4]   # flip 25% -- enough to prove real correction,
    faces[flip_set] = faces[flip_set][:, ::-1]   # not so much the mean normal degenerates
    flipped_cyl = trimesh.Trimesh(vertices=cyl.vertices, faces=faces, process=False)

    data_flipped = preprocess(flipped_cyl, 'cyl.stl', up_axis=2)
    region_flipped = MeshFlowRegion(region_id='FRONT', face_indices=face_indices)

    route = generate_mesh_flow_route(
        data_flipped, region_flipped,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=20.0, standoff_mm=10.0,
    )
    assert route.total_passes > 0
    bad, total = 0, 0
    for p in route.passes:
        radial = p.points[:, :2] / np.linalg.norm(p.points[:, :2], axis=1, keepdims=True)
        dots = np.einsum('ij,ij->i', p.normals[:, :2], radial)
        bad += int((dots < 0.5).sum())
        total += len(dots)
    assert bad / total < 0.25, (
        f"Fix C corrected too few of the partially-flipped waypoints: {bad}/{total} still wrong"
    )


def test_generate_mesh_flow_route_whole_flat_region_flip_is_fix_as_job_not_fix_c(flat_panel_mesh):
    """Documents the honest boundary of Fix C, as a regression guard against
    ever claiming more than it delivers: a WHOLE flat region flipped, with no
    correctly-oriented local neighbor to disagree with, is NOT something a
    local check can catch — app.mesh.loader's load-time repair (Fix A,
    tests/test_mesh_loader.py) is what actually has to prevent this, by
    construction, before core_generator ever sees the mesh."""
    data = preprocess(flat_panel_mesh, 'panel.stl', up_axis=2)
    region = build_regions(data, ['TOP'])[0]
    face_indices = region.face_indices.copy()

    faces = flat_panel_mesh.faces.copy()
    faces[face_indices] = faces[face_indices][:, ::-1]
    flipped_mesh = trimesh.Trimesh(vertices=flat_panel_mesh.vertices, faces=faces, process=False)
    data_flipped = preprocess(flipped_mesh, 'panel.stl', up_axis=2)
    region_flipped = MeshFlowRegion(region_id='TOP', face_indices=face_indices)

    route = generate_mesh_flow_route(
        data_flipped, region_flipped,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=30.0, standoff_mm=10.0,
    )
    assert route.total_passes > 0
    for p in route.passes:
        assert np.all(p.normals[:, 2] < 0), (
            "if this ever starts failing, Fix C's local check started correcting "
            "whole-flat-region flips somehow -- re-verify the mechanism before "
            "treating that as good news, since the geometry says it shouldn't be possible"
        )


def test_resolve_lane_pitch_from_width_overlap_raises_open_decision_blocked():
    with pytest.raises(OpenDecisionBlocked) as exc_info:
        resolve_lane_pitch(None, LanePitchStrategy.FROM_WIDTH_OVERLAP)
    assert exc_info.value.decision_no == 1


def test_resolve_lane_pitch_from_reference_raises_open_decision_blocked():
    with pytest.raises(OpenDecisionBlocked) as exc_info:
        resolve_lane_pitch(None, LanePitchStrategy.FROM_REFERENCE)
    assert exc_info.value.decision_no == 1


def test_resolve_lane_pitch_explicit_setting_requires_positive_value():
    assert resolve_lane_pitch(25.0, LanePitchStrategy.EXPLICIT_SETTING) == 25.0
    with pytest.raises(ValueError):
        resolve_lane_pitch(None, LanePitchStrategy.EXPLICIT_SETTING)
    with pytest.raises(ValueError):
        resolve_lane_pitch(-5.0, LanePitchStrategy.EXPLICIT_SETTING)
