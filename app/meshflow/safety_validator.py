"""Mesh Flow Phase 6 — safety validator (R10, §20.1, §20.3 steps 7-9).

Runs every mandatory check and reports findings; never raises, never silently
treats a missing prerequisite as a pass. See path_model.ValidationStatus for
why INCOMPLETE and PASSED are kept as separate outcomes.
"""
from __future__ import annotations
import math

import trimesh

from app.meshflow.open_decisions import AcceptanceTolerances, ProcessConstraints, ToolAxisConvention
from app.path import collision as _collision
from app.path import surface_safety as _surface_safety
from app.path.orientation_frames import angular_difference
from app.path.path_model import (
    PaintRoute,
    RouteValidationReport,
    ValidationFinding,
    ValidationStatus,
)

_ORIENTATION_JUMP_FINDING_FORMULA = 'F45'


def _status_from_findings(findings: list[ValidationFinding]) -> ValidationStatus:
    if any(f.severity == 'hard' for f in findings):
        return ValidationStatus.BLOCKED
    if any(f.severity == 'incomplete' for f in findings):
        return ValidationStatus.INCOMPLETE
    return ValidationStatus.PASSED


def validate_route_hard_gates(
    route: PaintRoute,
    mesh: trimesh.Trimesh,
    tolerances: AcceptanceTolerances,
    tool_axis_convention: ToolAxisConvention,
    process_constraints: ProcessConstraints | None = None,
) -> RouteValidationReport:
    """R10, §20.1 precedence table, §20.3 steps 7-9 — runs, in order: target-
    surface membership (F56) -> inside/outside (F57) -> segment non-penetration
    (F58) -> standoff (F59/C6) -> orientation jump (F45) -> collision (F47)."""
    findings: list[ValidationFinding] = []
    process_constraints = process_constraints or ProcessConstraints()

    for p in route.passes:
        # F56 — target-surface membership
        if tolerances.surface_deviation_mm is None:
            findings.append(ValidationFinding(
                severity='incomplete', rule_or_formula='F56', pass_id=p.id, waypoint_index=None,
                message='surface_deviation_mm not set (decision #5)',
            ))
        else:
            dists = _surface_safety.closest_point_distance(p.points, mesh)
            bad = dists > tolerances.surface_deviation_mm
            if bad.any():
                findings.append(ValidationFinding(
                    severity='hard', rule_or_formula='F56', pass_id=p.id,
                    waypoint_index=int(bad.argmax()),
                    message=f'surface deviation {float(dists[bad].max()):.3f}mm exceeds tolerance',
                ))

        # F57 — inside/outside (C2: indeterminate on non-watertight, never assumed outside)
        inside = _surface_safety.inside_mesh_mask(p.points, mesh)
        if inside is None:
            findings.append(ValidationFinding(
                severity='incomplete', rule_or_formula='F57', pass_id=p.id, waypoint_index=None,
                message='mesh is not watertight — inside/outside classification unreliable (C2)',
            ))
        elif inside.any():
            findings.append(ValidationFinding(
                severity='hard', rule_or_formula='F57', pass_id=p.id,
                waypoint_index=int(inside.argmax()),
                message='waypoint lies inside the mesh solid',
            ))

        # F58 — segment non-penetration (exact ray intersection, not sampling)
        seg_results = [
            _surface_safety.segment_penetrates(p.points[i], p.points[i + 1], mesh)
            for i in range(len(p.points) - 1)
        ]
        if any(r is None for r in seg_results):
            findings.append(ValidationFinding(
                severity='incomplete', rule_or_formula='F58', pass_id=p.id, waypoint_index=None,
                message='segment/triangle intersection test unavailable for one or more segments',
            ))
        elif any(r is True for r in seg_results):
            findings.append(ValidationFinding(
                severity='hard', rule_or_formula='F58', pass_id=p.id,
                waypoint_index=seg_results.index(True),
                message='a path segment penetrates the mesh surface',
            ))

        # F59 — standoff (C6), gated on both a configured range AND a verified tool-axis convention
        if process_constraints.standoff_range_mm is None or not tool_axis_convention.verified or p.normals is None:
            findings.append(ValidationFinding(
                severity='incomplete', rule_or_formula='F59', pass_id=p.id, waypoint_index=None,
                message='standoff range not set (decision #4), or tool-axis convention unverified, '
                        'or per-waypoint normals not computed for this route',
            ))
        else:
            low, high = process_constraints.standoff_range_mm
            dists = _surface_safety.closest_point_distance(p.points, mesh)
            bad = (dists < low) | (dists > high)
            if bad.any():
                findings.append(ValidationFinding(
                    severity='hard', rule_or_formula='F59', pass_id=p.id,
                    waypoint_index=int(bad.argmax()),
                    message=f'standoff {float(dists[bad][0]):.3f}mm outside configured range [{low},{high}]',
                ))

        # F45 — orientation jump (via Phase 4's angular_difference), same gating as F59
        if tolerances.orientation_jump_deg is None or not tool_axis_convention.verified or p.normals is None:
            findings.append(ValidationFinding(
                severity='incomplete', rule_or_formula=_ORIENTATION_JUMP_FINDING_FORMULA,
                pass_id=p.id, waypoint_index=None,
                message='orientation_jump_deg not set (decision #5), or tool-axis convention unverified, '
                        'or per-waypoint normals not computed for this route',
            ))
        elif len(p.normals) >= 2:
            jumps = [
                math.degrees(angular_difference(p.normals[i], p.normals[i + 1]))
                for i in range(len(p.normals) - 1)
            ]
            max_jump = max(jumps)
            if max_jump > tolerances.orientation_jump_deg:
                findings.append(ValidationFinding(
                    severity='hard', rule_or_formula=_ORIENTATION_JUMP_FINDING_FORMULA, pass_id=p.id,
                    waypoint_index=jumps.index(max_jump),
                    message=f'orientation jump {max_jump:.1f} deg exceeds tolerance',
                ))

    # F47 — collision (reuses app.path.collision.detect_collisions unchanged)
    if not mesh.is_watertight:
        findings.append(ValidationFinding(
            severity='incomplete', rule_or_formula='F47', pass_id=None, waypoint_index=None,
            message='mesh is not watertight — hard-collision check unavailable',
        ))
    else:
        standoff_for_collision = (
            process_constraints.standoff_range_mm[0] if process_constraints.standoff_range_mm else 0.0
        )
        flagged, _max_depth = _collision.detect_collisions([route], mesh, standoff_for_collision)
        for pass_id, kind in flagged.items():
            if kind == 'collision':
                findings.append(ValidationFinding(
                    severity='hard', rule_or_formula='F47', pass_id=pass_id, waypoint_index=None,
                    message='collision detected (mesh.contains)',
                ))

    return RouteValidationReport(
        region_id=route.region_id,
        findings=findings,
        tolerances_used=tolerances,
        status=_status_from_findings(findings),
    )
