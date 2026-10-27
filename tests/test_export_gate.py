"""Mesh Flow Phase 6 exit-gate tests: capability-based export classification
and the structural export-blocking contract."""
from __future__ import annotations

import numpy as np
import pytest
import trimesh

from app.meshflow.export_gate import ExportBlocked, ExportKind, _FORMAT_EXPORT_KIND, enforce_export_gate
from app.meshflow.open_decisions import (
    ABB_TOOL_AXIS_CONVENTION,
    AcceptanceTolerances,
    ProcessConstraints,
    ToolAxisConvention,
)
from app.path.path_model import PaintPass, PaintRoute


def _simple_route() -> PaintRoute:
    pts = np.array([[-150.0, 0.0, 200.0], [150.0, 0.0, 200.0]])
    p = PaintPass(id=0, region_id='TOP', direction='mesh_flow', points=pts,
                  is_forward=True, sub_index=0, slice_position=0.0,
                  normals=np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]))
    return PaintRoute(region_id='TOP', passes=[p], connections=[],
                       unit='mm', spacing_mm=30.0, total_passes=1, total_length_mm=100.0)


def test_format_classification_matches_capability_not_oem_label():
    """Regression-proofs this issue's own fix: a future contributor adding a
    new format must deliberately choose, not default into the wrong bucket."""
    robot_executable = {'abb_rapid', 'fanuc_ls', 'durr', 'robodk', 'vc', 'delmia', 'gcode'}
    geometry_only = {'cad_dxf'}
    assert set(_FORMAT_EXPORT_KIND) == robot_executable | geometry_only
    for fmt in robot_executable:
        assert _FORMAT_EXPORT_KIND[fmt] is ExportKind.ROBOT_EXECUTABLE
    for fmt in geometry_only:
        assert _FORMAT_EXPORT_KIND[fmt] is ExportKind.GEOMETRY_ONLY


@pytest.mark.parametrize('fmt', ['abb_rapid', 'fanuc_ls', 'durr', 'gcode', 'robodk', 'vc', 'delmia'])
def test_robot_executable_blocked_when_incomplete(fmt):
    """Explicitly includes gcode/robodk/vc/delmia alongside the three OEM
    formats — exactly what the first draft would have wrongly let through
    unvalidated as 'diagnostic'."""
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    route = _simple_route()
    with pytest.raises(ExportBlocked):
        enforce_export_gate(
            route, mesh, AcceptanceTolerances(), ABB_TOOL_AXIS_CONVENTION,
            _FORMAT_EXPORT_KIND[fmt],
        )


def test_robot_executable_blocked_when_hard_violation():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    pts = np.array([[-150.0, 0.0, 200.0], [150.0, 0.0, 2000.0]])
    p = PaintPass(id=0, region_id='TOP', direction='mesh_flow', points=pts,
                  is_forward=True, sub_index=0, slice_position=0.0)
    route = PaintRoute(region_id='TOP', passes=[p], connections=[],
                        unit='mm', spacing_mm=30.0, total_passes=1, total_length_mm=100.0)
    tolerances = AcceptanceTolerances(surface_deviation_mm=5.0)
    with pytest.raises(ExportBlocked):
        enforce_export_gate(route, mesh, tolerances, ABB_TOOL_AXIS_CONVENTION, ExportKind.ROBOT_EXECUTABLE)


def test_robot_executable_blocked_when_convention_unverified_even_if_passed():
    """The redundant safety net: proves a Fanuc/Dürr route can never pass
    regardless of how its other findings read."""
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    route = _simple_route()
    tolerances = AcceptanceTolerances(surface_deviation_mm=1.0, orientation_jump_deg=5.0)
    constraints = ProcessConstraints(standoff_range_mm=(-1.0, 1.0))
    unverified = ToolAxisConvention(controller='FANUC', verified=False)
    with pytest.raises(ExportBlocked):
        enforce_export_gate(
            route, mesh, tolerances, unverified, ExportKind.ROBOT_EXECUTABLE,
            process_constraints=constraints,
        )


def test_robot_executable_passes_and_proceeds_when_fully_validated():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    route = _simple_route()
    tolerances = AcceptanceTolerances(surface_deviation_mm=1.0, orientation_jump_deg=5.0)
    constraints = ProcessConstraints(standoff_range_mm=(-1.0, 1.0))
    report = enforce_export_gate(
        route, mesh, tolerances, ABB_TOOL_AXIS_CONVENTION, ExportKind.ROBOT_EXECUTABLE,
        process_constraints=constraints,
    )
    assert report.exportable is True


def test_geometry_only_never_raises_regardless_of_validation_status():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    route = _simple_route()
    report = enforce_export_gate(
        route, mesh, AcceptanceTolerances(), ABB_TOOL_AXIS_CONVENTION, ExportKind.GEOMETRY_ONLY,
    )
    assert report.exportable is False   # still honestly INCOMPLETE...
    # ...but enforce_export_gate did not raise, proving DXF is never blocked.
