"""Contiguous gun_on-run grouping — relocated from
app/reference/reference_metrics.py (Mesh Flow Phase 2) so both the reference-
diagnostics side (Phase 2, read-only) and the generated-path side (Phase 5+,
generative) call one implementation instead of two that could drift. Behavior
is unchanged from the Phase 2 originals; reference_metrics.py re-imports these.
"""
from __future__ import annotations


def split_contiguous_gun_on_runs(waypoints: list) -> list[list]:
    """Group waypoints into contiguous gun_on==True runs ("lanes").

    Transit/approach/retract runs (gun_on False or unknown) are skipped — a lane
    is only ever a stretch where the process state was actually on.
    """
    runs: list[list] = []
    current: list = []
    for w in waypoints:
        if w.process_state.gun_on is True:
            current.append(w)
        else:
            if current:
                runs.append(current)
                current = []
    if current:
        runs.append(current)
    return runs


def split_contiguous_gun_on_runs_including_unknown(waypoints: list) -> list[list]:
    """Same grouping as split_contiguous_gun_on_runs but counts ALL runs (paint
    and transit) — used only to report how many transit runs were skipped."""
    runs: list[list] = []
    current: list = []
    current_state: bool | None = None
    for w in waypoints:
        state = w.process_state.gun_on is True
        if current and state != current_state:
            runs.append(current)
            current = []
        current.append(w)
        current_state = state
    if current:
        runs.append(current)
    return runs
