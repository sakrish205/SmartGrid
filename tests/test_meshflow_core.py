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


def test_generate_mesh_flow_route_self_corrects_flipped_region_normals(flat_panel_mesh):
    """Fix C: even when the region's own nearest-triangle normal is wrong
    (winding flipped directly on the mesh here, bypassing loader-level
    repair entirely, to isolate Fix C from Fix A), the self-check against
    the full watertight mesh must still land every waypoint outside."""
    data = preprocess(flat_panel_mesh, 'panel.stl', up_axis=2)
    region = build_regions(data, ['TOP'])[0]
    face_indices = region.face_indices.copy()   # fixed before any winding change

    faces = flat_panel_mesh.faces.copy()
    faces[face_indices] = faces[face_indices][:, ::-1]   # flip TOP's winding
    flipped_mesh = trimesh.Trimesh(vertices=flat_panel_mesh.vertices, faces=faces, process=False)
    assert flipped_mesh.is_watertight

    # Reuse the ORIGINAL (correct) face membership directly instead of
    # re-classifying the flipped mesh -- classify_regions_with_confidence
    # would reject these faces from TOP entirely once their normals point
    # the wrong way, which would test the classifier, not core_generator.
    data_flipped = preprocess(flipped_mesh, 'panel.stl', up_axis=2)
    region_flipped = MeshFlowRegion(region_id='TOP', face_indices=face_indices)

    route = generate_mesh_flow_route(
        data_flipped, region_flipped,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=30.0, standoff_mm=10.0,
    )
    assert route.total_passes > 0
    all_points = np.concatenate([p.points for p in route.passes], axis=0)
    inside = surface_safety.inside_mesh_mask(all_points, flipped_mesh)
    assert inside is not None
    assert not inside.any(), "Fix C failed to self-correct flipped region normals"


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
