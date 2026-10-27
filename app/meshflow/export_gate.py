"""Mesh Flow Phase 6 — export gate.

Export kind is classified by what each OLP format can actually encode, not by
an OEM/generic label or an assumption about typical usage: G-code is directly
executable on real CNC/robot controllers; RoboDK has a documented "Run on
Robot" bridge to real hardware; DELMIA APT is a genuine NC program format that
gets post-processed into real machine code. Only CAD DXF carries pure 3D curve
geometry with no speed/trigger/frame data at all — the one format that is
GEOMETRY_ONLY by its own definition, not by assumption about its typical use.

No simulator-preview escape hatch: gating every motion-capable format fully
was decided over adding a confirmed SIMULATION_PREVIEW bypass. If an
in-progress preview workflow is needed, it should be solved by the viewer's
own live rendering (which never goes through an OLP exporter at all), not by
weakening this gate.
"""
from __future__ import annotations
from enum import Enum

from app.meshflow.open_decisions import AcceptanceTolerances, ProcessConstraints, ToolAxisConvention
from app.meshflow.safety_validator import validate_route_hard_gates
from app.path.path_model import PaintRoute, RouteValidationReport


class ExportKind(Enum):
    ROBOT_EXECUTABLE = 'robot_executable'   # encodes a complete motion program (position + motion
                                              # type + speed + process/trigger state) in a format real
                                              # controller-adjacent software can run or post-process
                                              # into running — gated fully by enforce_export_gate.
    GEOMETRY_ONLY = 'geometry_only'           # pure curve data, NO motion semantics at all — cannot
                                              # actuate anything by format definition. Currently
                                              # only cad_dxf.


_FORMAT_EXPORT_KIND: dict[str, ExportKind] = {
    'abb_rapid': ExportKind.ROBOT_EXECUTABLE,
    'fanuc_ls':  ExportKind.ROBOT_EXECUTABLE,
    'durr':      ExportKind.ROBOT_EXECUTABLE,
    'robodk':    ExportKind.ROBOT_EXECUTABLE,   # has a direct Run-on-Robot execution path
    'vc':        ExportKind.ROBOT_EXECUTABLE,   # Visual Components can drive real cell hardware
    'delmia':    ExportKind.ROBOT_EXECUTABLE,   # APT is a real post-processable NC program format
    'gcode':     ExportKind.ROBOT_EXECUTABLE,   # directly executable on real CNC/robot controllers
    'cad_dxf':   ExportKind.GEOMETRY_ONLY,      # pure curves — the only format with no motion semantics
}


class ExportBlocked(Exception):
    def __init__(self, report: RouteValidationReport):
        blocking = [f for f in report.findings if f.severity in ('hard', 'incomplete')]
        super().__init__(
            f"{len(blocking)} finding(s) block robot-executable export: "
            f"{[(f.severity, f.rule_or_formula) for f in blocking]}"
        )
        self.report = report


def enforce_export_gate(
    route: PaintRoute,
    mesh,
    tolerances: AcceptanceTolerances,
    tool_axis_convention: ToolAxisConvention,
    export_kind: ExportKind,
    process_constraints: ProcessConstraints | None = None,
) -> RouteValidationReport:
    """The ONLY sanctioned entry point between a Mesh Flow route and any
    exporter. For ROBOT_EXECUTABLE: raises ExportBlocked unless
    report.exportable AND tool_axis_convention.verified — both required
    explicitly, not just folded into the report, so a verified-convention
    check can never be bypassed by some other finding happening to read
    PASSED. For GEOMETRY_ONLY: never raises — there is no motion program in
    the output for an incomplete validation to make unsafe."""
    report = validate_route_hard_gates(route, mesh, tolerances, tool_axis_convention, process_constraints)
    if export_kind is ExportKind.ROBOT_EXECUTABLE:
        if not report.exportable or not tool_axis_convention.verified:
            raise ExportBlocked(report)
    return report
