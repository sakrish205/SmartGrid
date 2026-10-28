# OLP Export — Standard Review

Reviewed SmartGrid's native OLP exporters (`app/export/olp_export.py`) directly against
the three OEM reference files in this folder — `Fanuc_Ref/S1_100.LS`, `Durr_Ref/B05.tt/Aleta_Ptras.tip`
+ `.tid`, and `Exercise_2.mod` — plus the neutral CSV format described in `meeting transcript.txt`
("sequence ID, trigger, segment type, pass ID").

## Fixed — matched to the reference files

| Format | What was fixed |
|---|---|
| Fanuc (`S1_100.LS`) | `CNT` now tightens at sharp corners instead of always `CNT100`; `Preset[n]` switches per region instead of staying at 1; `PAINT_PROCESS` block uses the reference's `:` separator with real computed `LAST_CYCLE_TIME` / `LAST_GUN_ON_TIME` / per-preset gun-on time; `LINE_TRACK` section added; program now opens/closes with synthetic `J` approach/retract moves (retreated along the real surface normal), matching `J P[1] ... J P[63]` in the reference. |
| Dürr (`Aleta_Ptras.tip/.tid`) | `SETOBJECT` derived from the source mesh filename instead of hardcoded `PART`; `SETBRUSH` pattern id varies per region instead of hardcoded `1`; a zone comment (`//REGION`) added per region block. |
| ABB (`Exercise_2.mod`) | One `PROC` per region (`mvFront`, `mvRear`, ...) called from `runJob()`, matching the reference's `mvHood`/`mvSideA` split, instead of one flat `mvPath()`; `wobj` name derived from the source filename. |
| Neutral CSV | Already matched the standard described in the meeting (`seq_id`, `Trigger`, `segment_type`, `pass_id`) — no change needed. |

Everything above is a value computed from data SmartGrid already has: the part's own
geometry (surface normals, corner angles), the path it already generated (region
boundaries, route order), or the input filename. Nothing here is guessed.

## Not fixed — present in the reference files, but not computable by SmartGrid

These five fields are genuinely part of what a real OEM program contains — they appear
in the reference files below — but the *correct value* for each one lives in a specific
robot and a specific factory that SmartGrid has never been told about. A plausible-looking
guess would be worse than an honest gap, because it would pass a visual review while
being physically meaningless on the real robot.

1. **Fanuc `CONFIG` field** (`S1_100.LS`, every `P[n]` block, e.g. `'N U T, 1, 0, 0'`)
   Records which of a 6-axis robot's several possible joint configurations (elbow up/down,
   wrist flip, turn number) reaches that point. Requires the exact kinematic model (DH
   parameters, joint limits) of the target robot and an inverse-kinematics solver.
   SmartGrid has no robot arm geometry at all — only the part mesh and the path on it.

2. **Fanuc `DO[]` digital-output lines** (`S1_100.LS` line 71: `DO[112]=ON ;`)
   A signal number wired into one specific paint cell's control system (e.g. conveyor
   or booth equipment). The number has meaning only inside that cell's own wiring — there
   is nothing about a waypoint or a mesh that could produce it.

3. **Dürr `WAIT_PAINTPOSITION(n)`** (`Aleta_Ptras.tip`, e.g. `WAIT_PAINTPOSITION(1900)`)
   A position along the paint line's conveyor track, tied to that specific line's layout,
   the robot's mounting position, and the conveyor's zero point. Not a function of the
   part's geometry or the path at all — it's a fact about a building, not a part.

4. **ABB external axes** (`Exercise_2.mod`, e.g. `PaintL p20,v600\E1:=[1,-690.1,2]\E2:=[1,708,0],...`)
   The angle/position of an external positioner or turntable that physically rotates the
   part during painting, synced to the robot's motion. SmartGrid treats the part as static
   in its own frame and has no model of whether a positioner even exists for a given cell.

5. **Robot-specific motion limits** (`Aleta_Ptras.tip`: `ACC(A3500)`, `VEL(V1000)`; `S1_100.LS`:
   joint-speed `%` values) — real acceleration/velocity ceilings for one specific robot
   model. SmartGrid has a generic speed setting but not that robot's actual physical limits.

### The dividing line

If a value can be computed from the part's own geometry or the path SmartGrid already
generated, it's fixable — and everything in that category has now been fixed. If a value
only has meaning inside a specific robot or a specific factory that was never described to
SmartGrid, it can only ever be supplied by the engineer — SmartGrid can carry it through to
the export as a configuration field, but it can never compute a correct number for it.

---
*Generated 2026-10-28, reviewing commits up to and including the Fanuc J approach/retract fix.*
