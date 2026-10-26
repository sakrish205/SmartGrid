# Mesh Flow — Phase 0 Rename Audit

Status: audit only. **No code changed in this phase.** This document records every place the
"Mesh Surface" identity (Mode 4) appears, for the Phase 8 rename rollout. See the full plan at
`convertmdfiles/SmartGrid_Mesh_Flow_Specification_v4_Bundle/SmartGrid_Mesh_Flow_Engineering_Specification_v4.md`
(§2 rename-scope table, §13).

Baseline behavior is locked in `tests/fixtures/mesh_surface_baseline.json`, guarded by
`tests/test_mesh_surface_baseline.py`. Phase 8 must leave that test passing unchanged — proof the
rename alone introduced no behavior change.

## Rename map

| Layer | File:line | Current value | Notes |
|---|---|---|---|
| UI radio label | `app/ui/ribbon.py:500` | `QRadioButton('Mesh Surface')` | `_mesh_radio` |
| UI tooltip | `app/ui/ribbon.py:508-510` | "Slices directly through the mesh triangles..." | |
| UI button group | `app/ui/ribbon.py:513-519` | `_mesh_radio` registered with id `1` in `_target_grp` | |
| Canonical dispatch key | `app/ui/ribbon.py:856-861` | `get_path_target()` returns `'mesh'` | the actual internal identifier |
| Mode-name string | `app/main_window.py:1012-1022` | `_build_params` sets `mode = 'Mesh Surface'` into `GenerationParams.path_mode` | written into every export artifact |
| Dispatch | `app/main_window.py:1057-1058` | `else: self._generate_mesh(spray_mm)` | reached when `path_target == 'mesh'` |
| Entry point | `app/main_window.py:1257-1278` | `_generate_mesh()` | |
| Status bar text | `app/main_window.py:1265` | `'Generating mesh surface paths…'` | |
| Help/about text | `app/main_window.py:1591,1643` | prose mentioning "Mesh Surface" | |
| Comments | `app/main_window.py:280,305` | "...for Mesh Surface passes", "...populate for Mesh Surface" | |
| Type comment | `app/path/path_model.py:22` | enumerates `'Mesh Surface'` as one of 4 valid `path_mode` values | plain `str` field, not an enum |
| Docstrings | `app/path/generator.py:1,106` | module/function docstrings name "Mesh Surface" | |
| Docstrings (dead code) | `app/path/geodesic_generator.py:1,3` | "...for Mesh Surface mode" | module is currently unreachable from the UI — do **not** wire it up in Phase 0 or 8 |
| Comment | `app/export/csv_export.py:117` | "Per-waypoint normal (Conform/Mesh Surface); falls back to route normal" | |
| Runtime export payload | `app/export/json_export.py`, `app/export/csv_export.py`, `app/export/olp_export.py` | each writes `params.path_mode` (`"Mesh Surface"` string) into generated files | blocked on open decision #6 (backward-compat policy) before changing — old exports/scripts may key off this literal string |
| Tests | `tests/test_generator.py::test_generate_returns_paint_route` | only test exercising `generate_route`; no identity assertions | update expected labels in Phase 8, add legacy-ID compat test if §12 decision #6 requires one |

## Explicitly out of scope

Do not rename generic uses of the phrase "mesh surface" as a geometry description — only the
identifier/label/dispatch set above. Not touched: Boundary Box, Face Grid / Adaptive, Face Grid /
Conform (Modes 1-3) — their own "mesh surface" references, if any, describe geometry, not this
mode's identity, and must be left alone.

## Open blocker for Phase 8

Per the plan's open-decisions list, item 6 ("canonical identifier + backward-compatibility
policy") is **not resolved here** — Phase 8 cannot safely rename the runtime string in
`GenerationParams.path_mode` (and therefore the exported-file payloads) until that decision is
made, since old saved projects/exports may already depend on the literal string `"Mesh Surface"`.
