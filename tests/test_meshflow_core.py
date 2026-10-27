"""Mesh Flow Phase 3 exit-gate tests: region manager, direction/lane planning,
and the minimal core generator. Mesh Surface's generate_route() is untouched by
any of this — see tests/test_mesh_surface_baseline.py for that guarantee.
"""
from __future__ import annotations
import pytest
import trimesh

from app.mesh.preprocessor import preprocess
from app.meshflow.core_generator import generate_mesh_flow_route
from app.meshflow.lane_planner import resolve_lane_pitch
from app.meshflow.open_decisions import LanePitchStrategy, OpenDecisionBlocked
from app.meshflow.region_manager import build_regions
from app.path.path_model import RegionConfidence


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
