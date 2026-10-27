"""Mesh Flow Phase 6 exit-gate tests: the three-status validation model.

Regression-proofs the exact vacuous-truth bug the first draft had: a report
with every tolerance unset must be INCOMPLETE/non-exportable, never a silent
PASSED-equivalent.
"""
from __future__ import annotations
import trimesh

from app.meshflow.open_decisions import (
    AcceptanceTolerances,
    ProcessConstraints,
    ToolAxisConvention,
    ABB_TOOL_AXIS_CONVENTION,
)
from app.meshflow.safety_validator import validate_route_hard_gates
from app.path.path_model import PaintPass, PaintRoute, ValidationStatus
import numpy as np


def _simple_route(points: np.ndarray, region_id: str = 'TOP') -> PaintRoute:
    p = PaintPass(id=0, region_id=region_id, direction='mesh_flow', points=points,
                  is_forward=True, sub_index=0, slice_position=0.0)
    return PaintRoute(region_id=region_id, passes=[p], connections=[],
                       unit='mm', spacing_mm=30.0, total_passes=1, total_length_mm=100.0)


def test_all_none_tolerances_produce_incomplete_not_passed():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    pts = np.array([[-150.0, 0.0, 200.0], [150.0, 0.0, 200.0]])
    route = _simple_route(pts)

    report = validate_route_hard_gates(
        route, mesh, AcceptanceTolerances(), ABB_TOOL_AXIS_CONVENTION, ProcessConstraints(),
    )
    assert report.status is ValidationStatus.INCOMPLETE
    assert report.exportable is False
    assert len(report.findings) > 0
    assert all(f.severity != 'hard' for f in report.findings)   # nothing actually FAILED, just unproven


def test_hard_geometric_violation_produces_blocked():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    # A point far outside the configured surface-deviation tolerance.
    pts = np.array([[-150.0, 0.0, 200.0], [150.0, 0.0, 2000.0]])
    route = _simple_route(pts)

    tolerances = AcceptanceTolerances(surface_deviation_mm=5.0)
    report = validate_route_hard_gates(route, mesh, tolerances, ABB_TOOL_AXIS_CONVENTION, ProcessConstraints())
    assert report.status is ValidationStatus.BLOCKED
    assert report.exportable is False
    assert any(f.severity == 'hard' and f.rule_or_formula == 'F56' for f in report.findings)


def test_fully_toleranced_clean_route_can_pass():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    pts = np.array([[-150.0, 0.0, 200.0], [150.0, 0.0, 200.0]])   # exactly on the +Z face
    route = _simple_route(pts)
    route.passes[0].normals = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])

    tolerances = AcceptanceTolerances(surface_deviation_mm=1.0, orientation_jump_deg=5.0)
    constraints = ProcessConstraints(standoff_range_mm=(-1.0, 1.0))
    report = validate_route_hard_gates(route, mesh, tolerances, ABB_TOOL_AXIS_CONVENTION, constraints)
    assert report.status is ValidationStatus.PASSED
    assert report.exportable is True


def test_unverified_tool_axis_convention_makes_orientation_checks_incomplete():
    mesh = trimesh.creation.box(extents=[400, 400, 400])
    pts = np.array([[-150.0, 0.0, 200.0], [150.0, 0.0, 200.0]])
    route = _simple_route(pts)
    route.passes[0].normals = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])

    tolerances = AcceptanceTolerances(surface_deviation_mm=1.0, orientation_jump_deg=5.0)
    constraints = ProcessConstraints(standoff_range_mm=(-1.0, 1.0))
    unverified = ToolAxisConvention(controller='FANUC', verified=False)
    report = validate_route_hard_gates(route, mesh, tolerances, unverified, constraints)
    assert report.status is ValidationStatus.INCOMPLETE
    assert any(f.rule_or_formula in ('F45', 'F59') and f.severity == 'incomplete' for f in report.findings)
