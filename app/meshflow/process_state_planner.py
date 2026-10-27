"""Mesh Flow Phase 5 — process-state planner (R8).

Sets gun_on=True only for PAINT-kind segments, mirroring the modal-state
concept already modeled read-only in app.reference.reference_model.ProcessState
(Phase 1) and the gun_on-run grouping already proven in
app.path.segment_runs.split_contiguous_gun_on_runs (Phase 2/5) — same grouping
concept, now used generatively instead of diagnostically.
"""
from __future__ import annotations

from app.path.path_model import Connection, MotionSegmentKind, PaintPass, ProcessState


_TRANSIT_SPEED_FACTOR = 1.5   # connectors travel faster than paint passes


def apply_process_states(
    passes: list[PaintPass],
    connections: list[Connection],
    default_speed_mmpm: float,
) -> None:
    """R8 — mutates passes/connections in place: every PaintPass gets
    process_state.gun_on=True (it IS the paint segment); every Connection
    (LANE_CONNECTOR/REGION_TRANSIT/APPROACH/RETRACT — never PAINT) gets an
    explicit gun_on=False. Mesh Surface passes/connections are untouched by
    this function — it is never called on their objects."""
    for p in passes:
        p.process_state = ProcessState(gun_on=True, speed_mmpm=default_speed_mmpm)

    for c in connections:
        if c.kind is MotionSegmentKind.PAINT:
            # A connector should never be tagged PAINT — plan_transitions never
            # produces this, but guard explicitly rather than silently miscounting.
            raise ValueError(f"Connection {c.id} has MotionSegmentKind.PAINT — connectors must not paint")
        c.process_state = ProcessState(gun_on=False, speed_mmpm=default_speed_mmpm * _TRANSIT_SPEED_FACTOR)
