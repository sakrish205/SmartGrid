"""Typed intermediate representation for a parsed reference OLP program.

Mesh Flow Phase 1 (see convertmdfiles/SmartGrid_Mesh_Flow_Specification_v4_Bundle/
SmartGrid_Mesh_Flow_Engineering_Specification_v4.md, R2/R9). Read-only: this module
and its parsers do not touch app/path/ or the live Mesh Surface generator.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Literal
import numpy as np


class ReferenceFrameType(Enum):
    OBJECT = 'object'   # Cartesian point
    AXIS = 'axis'        # joint-space point — not comparable to Cartesian XYZ+orientation data


@dataclass
class ProcessState:
    """Modal process state — applies to subsequent waypoints until next change (R8)."""
    gun_on: bool | None = None            # None = unknown/not yet established
    setbrush_index: int | None = None
    digital_outputs: dict[int, bool] = field(default_factory=dict)
    modal_notes: list[str] = field(default_factory=list)  # Preset[n], VEL/ACC labels — provenance only

    def copy(self) -> 'ProcessState':
        return ProcessState(
            gun_on=self.gun_on,
            setbrush_index=self.setbrush_index,
            digital_outputs=dict(self.digital_outputs),
            modal_notes=list(self.modal_notes),
        )


@dataclass
class ReferenceWaypoint:
    source_id: str                          # declared ID (P[32], p600, PT12...) — never == order
    exec_index: int                         # position in resolved execution order (R2)
    position: np.ndarray | None             # (3,) or None for modal-only/non-motion lines
    orientation: np.ndarray | None          # raw exporter-native orientation (not normalized here)
    frame_type: ReferenceFrameType
    move_type: str                          # "L","J","C","PaintL","LIN"... controller-native
    process_state: ProcessState
    speed_ref: str | float | None           # named (v600) or literal — never silently resolved
    is_motion: bool                         # False for Preset[n]/VEL/ACC/SETBRUSH/GUN/WAIT_*/DO[]=ON
    raw_line_refs: list[int] = field(default_factory=list)
    external_axes: dict | None = None       # \E1:=[...]/\E2:=[...] kept opaque (R9)


@dataclass
class ReferenceProgram:
    source_file: str
    controller: Literal['ABB', 'FANUC', 'DURR']
    waypoints: list[ReferenceWaypoint]      # in resolved exec_index order
    unreferenced_source_ids: list[str] = field(default_factory=list)  # orphaned declared points
    dead_procedures: list[str] = field(default_factory=list)          # declared but never called
    parse_warnings: list[str] = field(default_factory=list)

    def motion_waypoints(self) -> list[ReferenceWaypoint]:
        """Cartesian, position-bearing, actual-motion waypoints only (Phase 2 input)."""
        return [
            w for w in self.waypoints
            if w.is_motion and w.position is not None and w.frame_type is ReferenceFrameType.OBJECT
        ]
