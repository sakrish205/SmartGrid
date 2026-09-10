# SmartGrid — 3D Spray Path Generator

Offline toolpath planning for robotic spray operations over 3D mesh surfaces. Loads STL/OBJ, selects spray regions, generates a boustrophedon lawnmower toolpath, exports to JSON or CSV.

[![Python](https://img.shields.io/badge/Python-3.12-blue)](https://www.python.org/)
[![PySide6](https://img.shields.io/badge/GUI-PySide6-green)](https://pypi.org/project/PySide6/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

---

## Installation

```bash
pip install PySide6 pyvista pyvistaqt "trimesh[easy]" numpy scipy
```

Launch:

```bash
python main.py
```

Or run `run.bat` (Windows shell launcher).  
Or double-click `SmartGrid.pyw` (no console window).

---

## Workflow

1. **Open mesh** — File › Open or drag an STL / OBJ onto the window
2. **Set up-axis** — select which world axis is vertical (X / Y / Z)
3. **Select regions** — toggle face buttons (TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT) or click the bounding box in the viewport
4. **Set parameters** — pitch, unit, standoff, sweep direction
5. **Preview** — enable Grid to verify pass spacing before generating
6. **Generate Path**
7. **Export JSON or CSV**

---

## Controls

### SELECT

| Control | Function |
|---|---|
| **TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT** | Toggle bounding-box faces as spray regions. Multiple faces active simultaneously. |
| **All / None** | Select or deselect all six faces. |
| **Select Faces** | 3D pick mode — click directly on the bounding box. Camera orbit locked during pick to prevent accidental view changes. |

### PARAMETERS

| Control | Function |
|---|---|
| **Unit** | Display unit for all distance inputs: mm / cm / m / in / ft. Unit change converts the displayed value; physical distance is preserved. |
| **Spray Width (Pitch)** | Centre-to-centre distance between adjacent passes. Pass count: `n = ⌈region_span / pitch⌉`. |

### PATH MODE

Selects the toolpath generation algorithm.

#### Boundary Box

Flat lawnmower passes on the axis-aligned bounding-box face plane.

- Face plane fixed at the outermost mesh extent along the face normal axis: `face_pos = bbox_max[face_axis] + standoff_mm`
- Pass positions: `step_i = step_min + pitch/2 + i × pitch`, `i = 0…n−1`
- Direction alternation: `is_forward = (i + direction_offset) % 2 == 0`
- Connectors: straight-line transit between pass endpoints
- No mesh geometry queried during generation

Use for flat panels and sheet metal where the surface does not deviate from the bounding face plane.

#### Face Grid — Adaptive

Surface-following passes using shadow projection on a mean-normal spray-plane basis. No mesh slicing.

1. Compute mean outward normal `n̂` from all forward-facing region faces (`face_normals · face_axis > 0`).
2. Build orthonormal basis: `pass_vec = normalize(n̂ × up)`, `step_vec = normalize(pass_vec × n̂)`.
3. Project region vertices: `pass_proj = v·pass_vec`, `step_proj = v·step_vec`, `depth_proj = v·n̂`.
4. Divide step extent into bands (half-width = `pitch × 0.65`).
5. Per band: `row_depth = max(depth_proj)` of vertices within the band (outermost surface point).
6. Endpoint in world space: `P = (row_depth + standoff_mm)·n̂ + p_min·pass_vec + step_pos·step_vec`

Each pass is a straight line between its two endpoints. Spray plane tilts to match the surface orientation; pass elevation tracks the peak depth per row. Use for curved panels with a dominant normal direction (bonnets, bumpers, side panels). Not suited to surfaces with significant curvature along the sweep direction — paths will float mid-pass.

#### Face Grid — Conform

Trimesh plane-intersection slicing on all forward-facing faces of the selected region. Captures slopes and edge transitions that the face classifier does not assign to the named region.

1. Collect all faces where `face_normals[:, face_axis] × face_sign > 0` — broader than the bbox-classified subset.
2. Pass to `generator.generate_route()` (same trimesh plane-intersection pipeline as Mesh Surface).
3. Cutting planes spaced at `pitch` intervals from the selected-face bounding box, extended ±0.001 mm to catch boundary triangles.
4. Per plane: intersect mesh, filter to forward-facing face set, stitch segments via graph-walk (tolerance 10⁻⁶ m), apply RDP simplification (ε = 0.3 mm).
5. Boustrophedon order by plane index — holes and sub-passes do not disrupt the serpentine pattern.
6. Standoff applied post-generation: project each waypoint onto nearest mesh face, offset by `standoff_mm × face_normal`.

Differs from **Mesh Surface** in face scope: Mesh Surface uses only classifier-assigned faces; Conform uses all geometrically outward-facing faces. Use for parts with blended edges or compound curves where Mesh Surface paths stop short of the actual surface extent.

#### Mesh Surface

Trimesh plane-intersection slicing on the classifier-assigned region faces only.

1. Compute step-axis extent from the selected region's vertex coordinates.
2. Cutting planes at `step_i = step_min + pitch/2 + i × pitch`.
3. `trimesh.intersections.mesh_plane(plane_normal, plane_origin, return_faces=True)` per plane.
4. Filter segments to source face indices within the selected region set.
5. Stitch, simplify (RDP ε = 0.3 mm), filter corner fragments (min length = `max(pitch × 0.10, 5 mm)`), cap sub-passes at 6 per level.
6. Boustrophedon order, straight-line connectors.

Use for complex curved surfaces and parts with holes or cutouts. Paths lie on the mesh surface in both step and sweep directions.

#### Standoff

Lifts every toolpath point outward from the surface by a fixed distance.

```
P′ = P + standoff_mm × n̂
```

Applied after all path generation. In Boundary Box mode `n̂` is the face axis unit vector. In Face Grid mode `n̂` is the mean face normal. In Mesh Surface and Conform modes `n̂` is the nearest mesh face normal per waypoint. Default is 0 (paths on the surface).

### SWEEP

| Option | Effect |
|---|---|
| **↺ CW** | Pass 0 runs in the positive step axis direction. `direction_offset = 0`. |
| **↻ CCW** | Pass 0 runs in the negative step axis direction. `direction_offset = 1`. |

### WAYPOINTS

| Control | Function |
|---|---|
| **Waypoints** | Enables uniform resampling of each pass. Off = two endpoints per pass only. |
| **Interval** | Resampling spacing in the current unit. Export omits intermediate waypoints when Waypoints is unchecked. |

### PATH

| Button | Action |
|---|---|
| **Generate Path** | Runs the selected algorithm on all active regions. Background thread — UI stays live. Blue = forward pass, red = reversed pass, pink = connector. |
| **Clear Path** | Removes all generated paths and resets route statistics. |

### DISPLAY

| Control | Function |
|---|---|
| **Grid** | Overlays pitch-division lines on the selected regions. Updates live on parameter changes. |
| **Arrows** | Chevron tick marks along each pass showing travel direction. |
| **View Settings** | Colour and display picker — background, mesh, pass/connector/waypoint colours, line widths, mesh render style (Solid / Solid + Edges / Wireframe). |

### EXPORT

| Button | Output |
|---|---|
| **Export JSON** | Structured toolpath: metadata, per-pass 3D point arrays, connectors, lengths. |
| **Export CSV** | Flat table — one row per 3D point in execution order. |

---

## Export Schema

### JSON

```
output.json
├── version, author, generated_at
├── summary
│   ├── total_routes, total_passes, total_connections
│   ├── total_length_mm
│   └── regions, directions
└── routes[]
    ├── region_id, spacing_mm, total_length_mm
    ├── passes[]        → id, is_forward, length_mm, start, end, points[]
    └── connections[]   → id, from_pass_id, to_pass_id, is_air_move, length_mm, points[]
```

| Field | Meaning |
|---|---|
| `segment_type` | `pass` — spray gun ON; `connection` — transit move |
| `is_forward` | Sweep direction of the pass |
| `is_air_move` | `true` = gun off during transit |
| `points` | Ordered 3D waypoint list, world coordinates in mm |

### CSV

One row per point in execution order.

| Column | Content |
|---|---|
| `segment_type` | `pass` or `connection` |
| `region` | Face name (TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT) |
| `pass_id` | Pass index within route |
| `is_forward` | TRUE / FALSE |
| `length_mm` | Segment arc length — written at `pt_idx = 0` only |
| `pt_idx` | Point index within segment |
| `x, y, z` | World coordinates in mm (4 decimal places) |

All internal distances are in millimetres. Unit conversion is applied at the display layer only.

---

## File Formats

| Format | Import | Export |
|---|---|---|
| STL (binary or ASCII) | ✓ | — |
| OBJ (single- or multi-body) | ✓ | — |
| JSON (toolpath) | — | ✓ |
| CSV (toolpath) | — | ✓ |

Multi-body OBJ files are merged at load via `trimesh.load(force='mesh')`.

---

## Project Structure

```
SmartGrid/
├── SmartGrid.pyw
├── app/
│   ├── main_window.py             window, signal wiring, QThread workers
│   ├── ui/
│   │   ├── ribbon.py              ribbon bar — all controls
│   │   ├── viewer.py              PyVista / VTK viewport
│   │   └── view_settings_dialog.py
│   ├── mesh/
│   │   ├── loader.py              trimesh load + normal repair
│   │   ├── preprocessor.py        normals, centroids, bbox, adjacency (cached)
│   │   └── regions.py             face classification by normal direction
│   ├── path/
│   │   ├── bbox_generator.py      Boundary Box toolpath
│   │   ├── face_grid_generator.py Adaptive shadow-projection toolpath
│   │   ├── generator.py           Mesh Surface / Conform orchestration
│   │   ├── slicer.py              trimesh plane intersection + face filter
│   │   ├── stitcher.py            segment graph-walk → ordered polylines
│   │   ├── connector.py           pass-to-pass connectors
│   │   └── path_model.py          PaintPass, Connection, PaintRoute
│   └── export/
│       ├── json_export.py
│       └── csv_export.py
├── models/
│   └── mesh_model.py
└── tests/
```

---

## License

Apache 2.0 — © 2024 Saketha Krishna B S  
See [LICENSE](LICENSE) for full terms.
