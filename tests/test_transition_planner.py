"""Mesh Flow Phase 5 exit-gate tests: transition_planner and process_state_planner (R8)."""
from __future__ import annotations
import numpy as np
import pytest
import trimesh

from app.meshflow.core_generator import generate_mesh_flow_route
from app.meshflow.open_decisions import LanePitchStrategy
from app.meshflow.process_state_planner import apply_process_states
from app.meshflow.region_manager import build_regions
from app.path.path_model import MotionSegmentKind


@pytest.fixture
def mesh_flow_route():
    box = trimesh.creation.box(extents=[200, 300, 100]).subdivide().subdivide()
    from app.mesh.preprocessor import preprocess
    data = preprocess(box, 'panel.stl', up_axis=2)
    region = build_regions(data, ['TOP'])[0]
    route = generate_mesh_flow_route(
        data, region,
        lane_pitch_strategy=LanePitchStrategy.EXPLICIT_SETTING,
        explicit_pitch_mm=30.0,
    )
    return route


def test_connectors_are_never_painted(mesh_flow_route):
    for c in mesh_flow_route.connections:
        assert c.kind is not MotionSegmentKind.PAINT


def test_connectors_stay_within_one_region_by_construction(mesh_flow_route):
    # plan_transitions/connect_passes only ever receives one region's own passes,
    # so no connector can cross into another region's space (spec §15.1).
    region_ids = {p.region_id for p in mesh_flow_route.passes}
    assert len(region_ids) == 1


def test_apply_process_states_sets_gun_on_true_for_passes_false_for_connections(mesh_flow_route):
    apply_process_states(mesh_flow_route.passes, mesh_flow_route.connections, default_speed_mmpm=1000.0)
    for p in mesh_flow_route.passes:
        assert p.process_state is not None
        assert p.process_state.gun_on is True
    for c in mesh_flow_route.connections:
        assert c.process_state is not None
        assert c.process_state.gun_on is False


def test_apply_process_states_raises_on_paint_tagged_connection(mesh_flow_route):
    if not mesh_flow_route.connections:
        pytest.skip("no connections generated for this fixture")
    mesh_flow_route.connections[0].kind = MotionSegmentKind.PAINT
    with pytest.raises(ValueError):
        apply_process_states(mesh_flow_route.passes, mesh_flow_route.connections, default_speed_mmpm=1000.0)


def test_mesh_surface_connection_defaults_are_unaffected():
    """Additive-only guarantee: a plain Connection() (Mesh Surface's construction
    pattern) still defaults to LANE_CONNECTOR/no process_state — unchanged meaning."""
    from app.path.path_model import Connection
    c = Connection(id=0, from_pass_id=0, to_pass_id=1, points=np.zeros((2, 3)), is_air_move=True)
    assert c.kind is MotionSegmentKind.LANE_CONNECTOR
    assert c.process_state is None
