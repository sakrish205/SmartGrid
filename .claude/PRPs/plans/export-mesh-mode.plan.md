# Plan: Export Mesh Mode

## Summary
Add a fourth export mode — **Export Mesh** — next to the existing Export JSON/CSV/OLP buttons. It writes the generated toolpath itself (spray passes + travel connections) as a 3D mesh file (`.stl`), by sweeping a small circular tube along every pass's and connection's waypoints. No source-part geometry is included — only the path, so an engineer can open the file in any mesh/CAD tool and see (and reuse) the exact toolpath shape.

## User Story
As a robot programmer / process engineer,
I want to export the generated spray path as a standalone 3D mesh file,
So that I can inspect, measure, or re-import the exact toolpath geometry in CAD/mesh tooling without needing the SmartGrid app or the original part mesh.

## Problem → Solution
Today the only portable path formats are JSON/CSV (point tables) and OLP robot programs (vendor-specific motion code) — none of them open as visible 3D geometry in a generic viewer/CAD tool. → Add an `Export Mesh` button that writes the path as tube geometry in a universally-readable `.stl` file.

## Metadata
- **Complexity**: Small
- **Source PRD**: N/A
- **PRD Phase**: N/A
- **Estimated Files**: 4 (1 new export module, 1 new test file, ribbon.py, main_window.py)

---

## UX Design

### Before
```
┌─ EXPORT ────────────┐
│ [Export] [Export]   │
│ [ JSON ] [ CSV  ]    │
│ [Export]             │
│ [ OLP  ]             │
└──────────────────────┘
```

### After
```
┌─ EXPORT ──────────────────────┐
│ [Export] [Export] [Export]    │
│ [ JSON ] [ CSV  ] [ OLP  ]    │
│ [Export]                      │
│ [ Mesh ]                      │
└────────────────────────────────┘
```

### Interaction Changes
| Touchpoint | Before | After | Notes |
|---|---|---|---|
| Ribbon Export group | 3 buttons (JSON/CSV/OLP) | 4 buttons (+ Mesh) | Same `_small_btn` style, disabled until a path exists |
| Export menu | 3 actions | 4 actions (+ "Export Mesh...") | Same `QFileDialog.getSaveFileName` pattern |
| Save dialog | — | "Export Mesh" dialog, filter `STL (*.stl)`, auto-appends `.stl` | Mirrors `_export_csv`/`_export_json` exactly |

---

## Mandatory Reading

| Priority | File | Lines | Why |
|---|---|---|---|
| P0 | [app/export/csv_export.py](app/export/csv_export.py:82) | 82-150 | Exact exporter function signature/docstring/error style to mirror |
| P0 | [app/path/path_model.py](app/path/path_model.py:37) | 37-80 | `PaintPass`/`Connection`/`PaintRoute` field names (`.points` is `(N,3)` mm) |
| P0 | [app/main_window.py](app/main_window.py:1799) | 1799-1818 | `_export_csv` — exact save-dialog/try-except pattern to mirror for `_export_mesh` |
| P1 | [app/ui/ribbon.py](app/ui/ribbon.py:683) | 683-694, 317-331 | Export button group + signal declarations |
| P1 | [tests/test_olp_export.py](tests/test_olp_export.py:21) | 21-80 | `sample_route` fixture pattern to mirror in the new test file |

## External Documentation

```
KEY_INSIGHT: trimesh.creation.sweep_polygon(polygon, path) sweeps a 2D shapely
Polygon along an (N,3) path array and returns a capped, watertight trimesh.Trimesh —
exactly the "tube along a polyline" primitive this feature needs.
APPLIES_TO: app/export/mesh_export.py's tube-building helper
GOTCHA: sweep_polygon needs >= 2 path points; a pass/connection with < 2 points
must be skipped, not passed in (it raises/produces degenerate geometry otherwise).

KEY_INSIGHT: shapely is already an installed runtime dependency — trimesh[easy]
(requirements.txt:5) pulls it in, and trimesh.creation / trimesh.intersections
already depend on it internally. Importing shapely.geometry.Point directly in the
new exporter is not a new dependency.
APPLIES_TO: app/export/mesh_export.py
GOTCHA: none — confirmed via `python -c "import shapely"` in this environment (2.1.2).

KEY_INSIGHT: trimesh.util.concatenate(list_of_meshes) merges multiple independent
Trimesh objects into one, and mesh.export(filepath) infers the file format from the
extension (.stl here). No manual STL writing needed.
APPLIES_TO: app/export/mesh_export.py's export_route_mesh()
GOTCHA: concatenate([]) on an empty list — guard against it (raise a clear
ValueError like the "Nothing to export" checks elsewhere) rather than letting
trimesh raise an opaque error.
```

---

## Patterns to Mirror

### EXPORTER_FUNCTION_SIGNATURE
// SOURCE: app/export/csv_export.py:82-92
```python
def export_route_csv(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
    speeds_map: dict | None = None,
) -> None:
    """Export routes to neutral CSV.
    ...
    """
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        ...
```
New exporter mirrors this shape but takes no `params`/`speeds_map` (pure geometry, no metadata) and no open file handle — it builds a mesh then calls `mesh.export(filepath)`.

### MAIN_WINDOW_EXPORT_SLOT
// SOURCE: app/main_window.py:1799-1817
```python
def _export_csv(self) -> None:
    if not self._current_routes:
        QMessageBox.warning(self, 'Nothing to export', 'Generate a path first.')
        return
    path, _ = QFileDialog.getSaveFileName(self, 'Export CSV', '', 'CSV (*.csv)')
    if not path:
        return
    if not path.lower().endswith('.csv'):
        path += '.csv'
    self.statusBar().showMessage('Exporting toolpath...')
    try:
        export_route_csv(
            self._current_routes, path,
            params=self._last_params,
        )
        self.statusBar().showMessage(f'Exported: {path}')
    except Exception as exc:
        QMessageBox.critical(self, 'Export error', str(exc))
        self.statusBar().showMessage('Export failed.')
```
`_export_mesh` follows this exactly (title `'Export Mesh'`, filter `'STL (*.stl)'`, extension `.stl`), calling `export_route_mesh(self._current_routes, path)` with no `params` kwarg.

### RIBBON_EXPORT_BUTTON
// SOURCE: app/ui/ribbon.py:683-694
```python
def _build_export(self) -> _Group:
    g = _Group('Export')
    self._exp_json_btn = _small_btn('Export\nJSON', _make_icon('export', 16))
    self._exp_csv_btn  = _small_btn('Export\nCSV',  _make_icon('export', 16))
    self._exp_olp_btn  = _small_btn('Export\nOLP',  _make_icon('export', 16))
    for b in (self._exp_json_btn, self._exp_csv_btn, self._exp_olp_btn):
        b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
        b.setFixedWidth(46)
    g.add(self._exp_json_btn)
    g.add(self._exp_csv_btn)
    g.add(self._exp_olp_btn)
    return g
```
Add `self._exp_mesh_btn = _small_btn('Export\nMesh', _make_icon('export', 16))` into the same tuple/loop/`g.add(...)` calls.

### RIBBON_SIGNAL_WIRING
// SOURCE: app/ui/ribbon.py:327-329, 741-743
```python
export_json         = Signal()
export_csv          = Signal()
export_olp          = Signal()
...
self._exp_json_btn.clicked.connect(self.export_json)
self._exp_csv_btn.clicked.connect(self.export_csv)
self._exp_olp_btn.clicked.connect(self.export_olp)
```
Add `export_mesh = Signal()` and `self._exp_mesh_btn.clicked.connect(self.export_mesh)`.

### RIBBON_ENABLE_DISABLE_LISTS
// SOURCE: app/ui/ribbon.py:921-924, 941-943, 950-953
Three places list all three export buttons together (initial disabled state, generating-lock, and `set_path_exists`). `self._exp_mesh_btn` must be added to each of the three tuples/calls alongside `self._exp_json_btn`/`self._exp_csv_btn`/`self._exp_olp_btn` — a bug fix class of its own if skipped (button would stay permanently enabled/disabled out of sync with the others).

### TEST_FIXTURE_PATTERN
// SOURCE: tests/test_olp_export.py:21-80
```python
@pytest.fixture
def sample_route() -> list[PaintRoute]:
    """Synthetic route with 2 passes and 1 connection."""
    p1_pts = np.array([[100.0, 200.0, 300.0], [150.0, 200.0, 300.0], [200.0, 200.0, 300.0]])
    ...
    route = PaintRoute(region_id='FRONT', passes=[...], connections=[...],
                        unit='mm', spacing_mm=50.0, total_passes=2,
                        total_length_mm=200.0, spray_normal=np.array([0.0, 0.0, 1.0]))
    return [route]
```
Reuse this exact fixture shape (copy, don't share across test files — existing convention).

---

## Files to Change

| File | Action | Justification |
|---|---|---|
| `app/export/mesh_export.py` | CREATE | New exporter: builds tube mesh from `PaintRoute.passes`/`.connections`, writes `.stl` |
| `tests/test_mesh_export.py` | CREATE | Unit test mirroring `test_olp_export.py`'s fixture/assert style |
| `app/ui/ribbon.py` | UPDATE | Add `export_mesh` signal, `_exp_mesh_btn`, wire into the 3 enable/disable lists |
| `app/main_window.py` | UPDATE | Import `export_route_mesh`, add `_export_mesh` slot, wire signal, add menu action |

## NOT Building
- No format-choice dialog (unlike OLP's multi-format picker) — mesh export is a single fixed `.stl` output, matching the JSON/CSV one-click pattern.
- No embedded metadata/header in the STL (binary STL has no real metadata field worth using) — purely geometric, per the request ("only the path").
- No UI control for tube radius — a sensible fixed default is used (see Task 1 GOTCHA). Can be exposed later if requested.
- No changes to `_export_olp_mesh_flow` (Mesh Flow's separate OLP export path) — Mesh Mode export reuses `self._current_routes` generically regardless of which path mode generated it, same as JSON/CSV already do.

---

## Step-by-Step Tasks

### Task 1: Create the mesh exporter
- **ACTION**: Create `app/export/mesh_export.py`
- **IMPLEMENT**:
  ```python
  """Export PaintRoute list as a standalone 3D mesh (.stl) — the path only,
  not the source part. Every pass and connection is swept into a thin tube
  so the toolpath is visible/reusable as ordinary mesh geometry in any
  CAD/mesh tool.
  """
  from __future__ import annotations

  import numpy as np
  import trimesh
  from shapely.geometry import Point

  from app.path.path_model import PaintRoute

  _DEFAULT_TUBE_RADIUS_MM = 2.0


  def _tube(points: np.ndarray, radius_mm: float) -> trimesh.Trimesh | None:
      """Sweep a small circular cross-section along points; None if too short to form a tube."""
      if points is None or len(points) < 2:
          return None
      poly = Point(0.0, 0.0).buffer(radius_mm, quad_segs=8)
      return trimesh.creation.sweep_polygon(poly, np.asarray(points, dtype=float))


  def export_route_mesh(
      routes: list[PaintRoute],
      filepath: str,
      tube_radius_mm: float = _DEFAULT_TUBE_RADIUS_MM,
  ) -> None:
      """Write every pass and connection in `routes` as tube geometry to `filepath` (.stl)."""
      tubes = []
      for route in routes:
          for p in route.passes:
              t = _tube(p.points, tube_radius_mm)
              if t is not None:
                  tubes.append(t)
          for c in route.connections:
              t = _tube(c.points, tube_radius_mm)
              if t is not None:
                  tubes.append(t)

      if not tubes:
          raise ValueError('No path geometry to export — generate a path first.')

      mesh = trimesh.util.concatenate(tubes)
      mesh.export(filepath)
  ```
- **MIRROR**: `EXPORTER_FUNCTION_SIGNATURE` pattern above (plain function, routes-list-in, filepath-out, no exception swallowing — caller handles errors)
- **IMPORTS**: `numpy`, `trimesh`, `shapely.geometry.Point`, `app.path.path_model.PaintRoute`
- **GOTCHA**: `sweep_polygon` needs ≥ 2 path points per segment — `_tube()` returns `None` and the caller skips it, so a degenerate 1-point pass/connection can't crash the export. Tube radius is intentionally geometry-independent (not derived from spray width) — 2 mm is a visually thin reference line that stays readable at typical panel scale; this is a deliberate simplification, not a correctness requirement.
- **VALIDATE**: `python -c "from app.export.mesh_export import export_route_mesh"` imports cleanly

### Task 2: Wire the ribbon button
- **ACTION**: Edit `app/ui/ribbon.py`
- **IMPLEMENT**:
  1. Add `export_mesh = Signal()` directly after `export_olp = Signal()` (ribbon.py:329)
  2. In `_build_export` (ribbon.py:683-694): add `self._exp_mesh_btn = _small_btn('Export\nMesh', _make_icon('export', 16))`, add it to the `for b in (...)` style loop and to the `g.add(...)` calls
  3. Near ribbon.py:743: add `self._exp_mesh_btn.clicked.connect(self.export_mesh)`
  4. Add `self._exp_mesh_btn` into all three places that currently list `self._exp_json_btn, self._exp_csv_btn, self._exp_olp_btn` together (ribbon.py ~922-924 initial-disabled block, ~942-943 generating-lock block, ~951-953 `set_path_exists`)
- **MIRROR**: `RIBBON_EXPORT_BUTTON`, `RIBBON_SIGNAL_WIRING`, `RIBBON_ENABLE_DISABLE_LISTS` patterns above
- **IMPORTS**: none new (Signal, `_small_btn`, `_make_icon` already imported in this file)
- **GOTCHA**: missing the button from even one of the three enable/disable lists leaves it permanently stuck enabled or disabled — grep `_exp_olp_btn` in ribbon.py after editing and confirm `_exp_mesh_btn` appears next to it everywhere it does
- **VALIDATE**: `grep -c "_exp_mesh_btn" app/ui/ribbon.py` should match the count of `_exp_olp_btn` occurrences

### Task 3: Wire main_window's export slot
- **ACTION**: Edit `app/main_window.py`
- **IMPLEMENT**:
  1. Add `from app.export.mesh_export import export_route_mesh` near the existing `from app.export.csv_export import export_route_csv` (main_window.py:28-31)
  2. After `self._ribbon.export_olp.connect(self._export_olp)` (main_window.py:599): add `self._ribbon.export_mesh.connect(self._export_mesh)`
  3. After `export_menu.addAction('Export OLP...', self._export_olp)` (main_window.py:680): add `export_menu.addAction('Export Mesh...', self._export_mesh)`
  4. Add a new method, placed next to `_export_csv` (main_window.py:1799-1817):
     ```python
     def _export_mesh(self) -> None:
         if not self._current_routes:
             QMessageBox.warning(self, 'Nothing to export', 'Generate a path first.')
             return
         path, _ = QFileDialog.getSaveFileName(self, 'Export Mesh', '', 'STL (*.stl)')
         if not path:
             return
         if not path.lower().endswith('.stl'):
             path += '.stl'
         self.statusBar().showMessage('Exporting toolpath...')
         try:
             export_route_mesh(self._current_routes, path)
             self.statusBar().showMessage(f'Exported: {path}')
         except Exception as exc:
             QMessageBox.critical(self, 'Export error', str(exc))
             self.statusBar().showMessage('Export failed.')
     ```
- **MIRROR**: `MAIN_WINDOW_EXPORT_SLOT` pattern above, exactly
- **IMPORTS**: `app.export.mesh_export.export_route_mesh`
- **GOTCHA**: none — this mirrors `_export_csv` 1:1, just dropping the `params=` kwarg
- **VALIDATE**: `python -c "import ast; ast.parse(open('app/main_window.py', encoding='utf-8').read())"` — syntax check

### Task 4: Unit test
- **ACTION**: Create `tests/test_mesh_export.py`
- **IMPLEMENT**: Copy the `sample_route` fixture from `tests/test_olp_export.py` (2 passes + 1 connection). Add:
  ```python
  def test_export_route_mesh_writes_stl(tmp_path, sample_route):
      out = tmp_path / "toolpath.stl"
      export_route_mesh(sample_route, str(out))
      assert out.exists()
      mesh = trimesh.load(str(out))
      assert len(mesh.faces) > 0
      assert mesh.is_watertight

  def test_export_route_mesh_raises_on_empty_routes():
      with pytest.raises(ValueError):
          export_route_mesh([], "unused.stl")
  ```
- **MIRROR**: `TEST_FIXTURE_PATTERN` above
- **IMPORTS**: `trimesh`, `pytest`, `app.export.mesh_export.export_route_mesh`, plus the same `PaintRoute`/`PaintPass`/`Connection` imports `test_olp_export.py` uses
- **GOTCHA**: `trimesh.load` on a single-mesh STL returns a `Trimesh` directly (not a `Scene`) because it's all one connected-component-free binary blob — no `force='mesh'` needed here (unlike `app/mesh/loader.py`'s OBJ-scene handling)
- **VALIDATE**: `python -m pytest tests/test_mesh_export.py -q` → 2 passed

---

## Testing Strategy

### Unit Tests
| Test | Input | Expected Output | Edge Case? |
|---|---|---|---|
| `test_export_route_mesh_writes_stl` | 2-pass + 1-connection sample route | `.stl` file exists, has faces, is watertight | No |
| `test_export_route_mesh_raises_on_empty_routes` | `[]` | `ValueError` | Yes — nothing to export |

### Edge Cases Checklist
- [x] Empty routes list → `ValueError` (Task 4, test 2)
- [x] Pass/connection with < 2 points → skipped via `_tube()` returning `None` (Task 1 GOTCHA) — not separately unit-tested given it mirrors the existing informal coverage level of `csv_export.py`/`json_export.py` (also untested directly); covered implicitly by the plan if revisited
- [ ] Very large route counts — not a concern; same O(n) sweep cost as existing CSV/JSON export

---

## Validation Commands

### Static Analysis
```bash
python -c "import ast; ast.parse(open('app/export/mesh_export.py', encoding='utf-8').read()); ast.parse(open('app/main_window.py', encoding='utf-8').read()); ast.parse(open('app/ui/ribbon.py', encoding='utf-8').read())"
```
EXPECT: no output, exit code 0

### Unit Tests
```bash
python -m pytest tests/test_mesh_export.py -q
```
EXPECT: all tests pass

### Full Test Suite
```bash
python -m pytest tests/ -q
```
EXPECT: no new failures (2 pre-existing unrelated failures in `test_generator.py` are expected — see repo history)

### Manual Validation
- [ ] Launch the app, load a mesh, generate a path (any mode)
- [ ] Click **Export Mesh** in the ribbon — a save dialog titled "Export Mesh" with filter `STL (*.stl)` appears
- [ ] Save, then reopen the `.stl` in the app itself (File → Open) or any STL viewer — confirm it shows thin tubes tracing the generated path, with no part geometry present
- [ ] Confirm the **Export Mesh** button is disabled before a path is generated and enabled after (same as JSON/CSV/OLP)

---

## Acceptance Criteria
- [ ] All 4 tasks completed
- [ ] All validation commands pass
- [ ] Tests written and passing
- [ ] No type errors (project has no static type checker configured — syntax check stands in)
- [ ] Matches UX design — 4th button in Export ribbon group + 4th Export-menu action

## Completion Checklist
- [ ] Code follows discovered patterns (exporter signature, main_window slot, ribbon button/signal/enable-disable triplication)
- [ ] Error handling matches codebase style (`ValueError` from the exporter, `QMessageBox.critical` in the UI slot)
- [ ] Tests follow `test_olp_export.py`'s fixture pattern
- [ ] No hardcoded values beyond the documented `_DEFAULT_TUBE_RADIUS_MM`
- [ ] No unnecessary scope additions (no format picker, no new dependency, no metadata embedding)
- [ ] Self-contained — no questions needed during implementation

## Risks
| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| `trimesh.creation.sweep_polygon` behaves differently on an older trimesh version than the one tested here (5.0.0) | Low | Medium | `requirements.txt` already pins `trimesh[easy]>=4.3.0`; `sweep_polygon` has existed since trimesh ~3.9, well under the floor — safe |
| Forgetting to add `_exp_mesh_btn` to one of the three enable/disable tuples in ribbon.py | Medium | Low (cosmetic — button stuck in wrong enabled state) | Called out explicitly as a GOTCHA in Task 2; grep-count validation step catches it |

## Notes
- Format choice: `.stl` (binary via trimesh's default writer) was chosen because it's the same format the app itself loads as its primary mesh input — an engineer can literally re-open the exported path file the same way they'd open a part mesh in SmartGrid, and it's universally supported by CAD/mesh tooling.
- "Only the path" (per the original request) is satisfied structurally: the exporter only ever touches `PaintRoute.passes`/`.connections`, never `self._model`/the loaded part mesh.
