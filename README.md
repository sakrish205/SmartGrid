# SmartGrid — 3D Spray Path Generator

Offline robot path planning for spray-paint operations over 3D mesh surfaces.  
Loads an STL/OBJ, selects spray regions, generates a complete lawnmower toolpath, exports to JSON or CSV.

[![Python](https://img.shields.io/badge/Python-3.12-blue)](https://www.python.org/)
[![PySide6](https://img.shields.io/badge/GUI-PySide6-green)](https://pypi.org/project/PySide6/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

---

## Installation

```bash
pip install PySide6 pyvista pyvistaqt "trimesh[easy]" numpy scipy
```

**Run:**

```bash
python main.py
```

Or double-click `SmartGrid.pyw` (no console window).

---

## Workflow

1. **Open mesh** — File › Open or drag an STL / OBJ onto the window
2. **Set up-axis** — choose which axis is vertical in your file (X / Y / Z)
3. **Select regions** — click the blue bounding box in 3D, or use the ribbon buttons
4. **Set parameters** — spray width, unit, standoff distance, sweep direction
5. **Preview** — enable Grid to visualise pass divisions before generating
6. **Generate** — click Generate Path
7. **Export** — Export JSON or Export CSV

---

## Ribbon Controls

### SELECT

| Control | Function |
|---|---|
| **TOP / BOT / FRT / REAR / LEFT / RIGHT** | Toggle individual bounding-box faces as spray regions. Multiple faces can be active simultaneously. |
| **All / None** | Select or deselect all six faces at once. |
| **Select Faces** | Enters 3D pick mode — click directly on the bounding box in the viewport. Camera orbiting is locked while active so accidental drags don't change the view. |

### PARAMETERS

| Control | Function |
|---|---|
| **Unit** | Sets the display unit for all distance inputs: mm / cm / m / in / ft. Switching unit converts the displayed value so the physical distance stays unchanged. |
| **Spray Width** | Step distance between adjacent passes (centre-to-centre). Determines pass count: `n = ⌈face_span / pitch⌉`. |

### PATH MODE

Selects the path generation algorithm. Choose based on surface geometry and required accuracy.

---

#### Boundary Box

Generates flat lawnmower passes on the axis-aligned bounding-box face plane. The face plane position is fixed at the outermost extent of the mesh along the face's normal axis.

**How it works:**

1. Identify the face plane (e.g. TOP → maximum Z coordinate of the mesh bounding box).
2. Apply standoff: `face_pos = bbox_max[face_axis] + standoff_mm`.
3. Compute pass count: `n = ⌈face_span / spray_width⌉`.
4. Place each pass at: `step_i = step_min + spray_width/2 + i × spray_width`.
5. Alternate direction (boustrophedon): `is_forward = (i + direction_offset) % 2 == 0`.
6. Connect pass endpoints with straight-line transit moves.

**Where to use:** flat panels, sheet metal, rectangular parts where the surface does not significantly deviate from the bounding face. Fastest mode — no mesh geometry queried during generation.

**Not suitable for:** curved or contoured surfaces — paths float above or intersect the geometry wherever the surface deviates from the flat bounding face.

---

#### Face Grid — Adaptive

Generates surface-following passes that tilt and contour to match the actual mesh geometry, without slicing the mesh. Uses shadow projection on a mean-normal basis.

**How it works:**

1. Compute the mean outward normal `n̂` of the selected faces (faces with a positive dot product against the face axis direction).
2. Build an orthonormal spray-plane basis: `pass_vec = normalize(n̂ × up)`, `step_vec = normalize(pass_vec × n̂)`.
3. Project all region vertices onto this basis: `pass_proj = v · pass_vec`, `step_proj = v · step_vec`, `depth_proj = v · n̂`.
4. Divide the step extent into bands of width = spray width (band half-width = `spray_width × 0.65`).
5. For each band, find the outermost vertex: `row_depth = max(depth_proj)` for vertices within the band. This is the shadow-projected surface peak.
6. Build endpoints in world space: `P = (row_depth + standoff_mm) × n̂ + p_min × pass_vec + step_pos × step_vec`.
7. Alternate direction by pass index. Connect with straight-line transit moves.

**Where to use:** moderately curved surfaces — bonnets, bumpers, panels with a dominant curvature direction. The spray plane tilts to match the surface orientation and pass elevation tracks the contour peak per row. Much faster than Conform.

**Limitation:** each pass is a straight line between its two endpoints. It does not follow the surface along the sweep direction. Significant curvature along the sweep axis will leave the path floating above the part mid-pass.

---

#### Face Grid — Conform

Slices the actual 3D mesh surface using all forward-facing faces of the selected region — not just the bbox-classified face subset. Uses the same trimesh plane-intersection pipeline as Mesh Surface mode, but casts a wider net to capture slopes and transitions that the classifier may not assign to a region.

**How it works:**

1. For each selected region, collect every face whose normal has a component in the outward direction: `np.where(mesh.face_normals[:, face_axis] * face_sign > 0)`. This includes sloped faces adjacent to the flat region that the normal-threshold classifier missed.
2. Pass these face indices into `generator.generate_route()` — the same trimesh plane-intersection pipeline used by Mesh Surface mode.
3. Cutting planes are spaced at `spray_width` intervals along the slice axis, derived from the selected faces' own bounding box (not the full mesh).
4. Each plane intersection is filtered to the selected forward-facing faces, stitched into polylines, RDP-simplified (ε = 0.3 mm), and boustrophedon-ordered.
5. Standoff is applied post-generation: each waypoint is projected onto the nearest mesh face and offset outward by `standoff_mm × face_normal`.

**How it differs from Mesh Surface:** Mesh Surface uses only the faces the region classifier assigned to a named face (TOP / FRONT / etc.). Conform uses all geometrically forward-facing faces, giving better coverage on parts with rounded edges or compound curves where the classifier boundary falls short.

**Where to use:** parts with blended transitions between faces, rounded edges, or compound curves where Adaptive paths visibly float mid-pass and Mesh Surface paths stop short of the actual surface extent.

**Limitation:** same performance cost as Mesh Surface — one `trimesh.mesh_plane` call per pass. On dense meshes generation takes a few seconds.

---

#### Mesh Surface

Generates passes by intersecting the triangulated mesh with equally spaced planes. Pass lines follow the actual mesh surface exactly.

**How it works:**

1. Compute the step-axis extent of the selected region from its vertex coordinates.
2. Place cutting planes at: `step_i = step_min + spray_width/2 + i × spray_width`.
3. For each plane, call `trimesh.intersections.mesh_plane(plane_normal, plane_origin, return_faces=True)` to get intersection segments and the triangle index each segment came from.
4. Filter segments: keep only those whose source triangle index is in the selected region's face set.
5. Stitch unordered segments into ordered polylines via a graph-walk on quantised endpoints (tolerance 1 × 10⁻⁶ m).
6. Apply RDP simplification (ε = 0.3 mm) to remove micro-jaggies from mesh triangulation.
7. Filter short fragments and diagonal corner clips; cap sub-passes per slice level at 6.
8. Sort and boustrophedon-order passes by slice level. Direction alternates by plane index, not pass count, so holes and sub-passes don't disrupt the serpentine pattern.
9. Connect consecutive passes with straight-line transit moves.

**Where to use:** complex curved surfaces, compound contours, parts with holes or cutouts. The paths are guaranteed to lie on the mesh surface — they follow curvature in both the step and sweep directions. Use this when Adaptive paths visibly float above the part.

**Limitation:** slower than the other modes on dense meshes (each plane queries the entire mesh). If the mesh has many faces, generation takes a few seconds.

---

**Standoff** — lifts every pass point outward by the specified distance along the spray normal. Applied after path generation:

```
P′ = P + standoff_mm × n̂
```

In Boundary Box mode `n̂` is the face's axis-aligned normal. In Face Grid mode `n̂` is the mean face normal. `0` places paths directly on the surface.

### SWEEP

| Option | Effect |
|---|---|
| **↺ CW** | Even-indexed passes run in the positive axis direction (forward). Pass 0 starts at `face_min + pitch/2`. |
| **↻ CCW** | Flips the starting direction — odd-indexed passes run forward instead. Equivalent to a `direction_offset = 1` in the boustrophedon alternation formula. |

### WAYPOINTS

| Control | Function |
|---|---|
| **Waypoints** checkbox | Enables intermediate waypoint resampling. When off, each pass has only its two endpoints. |
| **Interval** | Spacing between resampled waypoints along the pass. Set to 0 to disable resampling. Only active when the Waypoints checkbox is on — export excludes waypoints if unchecked. |

### PATH (Generate / Clear)

| Button | Action |
|---|---|
| **Generate Path** | Runs the selected path mode algorithm on all active regions. Runs in a background thread — UI stays responsive. Blue = forward pass, red = reversed pass, pink = connector. |
| **Clear Path** | Removes all generated paths from the viewport and resets route statistics. |

### DISPLAY

| Control | Function |
|---|---|
| **Grid** | Overlays pass-width division lines on the selected regions. Updates live when spray width or region selection changes. Useful for verifying coverage before generating. |
| **Arrows** | Shows chevron tick marks along each pass indicating travel direction. |
| **View Settings** | Opens the colour and display picker. Controls background colour, mesh colour, pass/connector/waypoint colours, line widths, and mesh display style (Solid / Solid + Edges / Wireframe). Changes apply live to the viewport. |

### EXPORT

| Button | Output |
|---|---|
| **Export JSON** | Structured toolpath file: metadata, per-pass 3D point arrays, connector points, lengths. Suitable for robot controller import scripts. |
| **Export CSV** | Flat table — one row per 3D point, in execution order. Readable in Excel or any robot preprocessor. |

---

## Path Generation — Algorithms

### Boundary Box mode

Pass positions along the step axis:

```
step_i = face_min + pitch/2 + i × pitch,   i = 0, 1, …, n-1
```

Direction alternation (boustrophedon):

```
is_forward = (i + direction_offset) % 2 == 0
```

`direction_offset = 0` for CW, `1` for CCW. Connectors are straight-line transit moves between the end of pass `i` and the start of pass `i+1`.

### Mesh Surface mode

1. Compute the region's bounding extent along the step axis from the vertices of all selected faces.
2. For each step position, call `trimesh.intersections.mesh_plane(plane_normal, plane_origin, return_faces=True)`.
3. Retain only segments whose source face index is in the selected region set.
4. Stitch unordered segments into ordered polylines via a graph-walk on quantised endpoints (tolerance 1 × 10⁻⁶ mm).
5. Alternate direction by plane index so sub-passes through holes don't break the serpentine pattern.

### Face Grid — Adaptive (shadow projection)

1. Compute the mean face normal of the selected region (`basis_normal`).
2. Project all region vertices onto the plane perpendicular to `basis_normal`.
3. Divide the projected extent into bands of width = spray width.
4. For each band, find the vertex with the maximum coordinate along `basis_normal` (the outermost point).
5. The pass elevation for that band is set to this depth — the path "shadows" the surface peak.

### Standoff offset

After generating any route, each 3D point `P` is shifted:

```
P′ = P + standoff_mm × n̂
```

where `n̂` is the unit outward normal of the spray plane. Applied uniformly to all passes and connectors.

---

## Export formats

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
    ├── passes[]   → id, is_forward, length_mm, start, end, points[]
    └── connections[]  → id, from_pass_id, to_pass_id, is_air_move, length_mm, points[]
```

**Robot controller key fields:**

| Field | Meaning |
|---|---|
| `segment_type` | `pass` = spray gun ON · `connection` = transit move |
| `is_forward` | Sweep direction for this pass |
| `is_air_move` | Gun off when `true` |
| `points` | Full 3D waypoint list in mm |

### CSV

One row per 3D point in execution order.

| Column | Content |
|---|---|
| `segment_type` | `pass` or `connection` |
| `region` | Bbox face (TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT) |
| `pass_id` | Pass index within route |
| `is_forward` | TRUE / FALSE |
| `length_mm` | Segment length — written only at `pt_idx = 0` |
| `pt_idx` | Point index within segment |
| `x, y, z` | World coordinates in mm (4 decimal places) |

---

## Project structure

```
SmartGrid/
├── SmartGrid.pyw              entry point (no console)
├── app/
│   ├── main_window.py         window, signal wiring, QThread workers
│   ├── ui/
│   │   ├── ribbon.py          single ribbon bar — all controls
│   │   ├── viewer.py          PyVista / VTK viewport and actor management
│   │   └── view_settings_dialog.py  colour + display picker
│   ├── mesh/
│   │   ├── loader.py          trimesh load + normal repair
│   │   ├── preprocessor.py    cached normals, centroids, bbox (computed once)
│   │   └── regions.py         face-to-region classification via face normals
│   ├── path/
│   │   ├── bbox_generator.py      flat lawnmower paths on bbox face planes
│   │   ├── face_grid_generator.py  adaptive shadow + conform mesh-surface paths
│   │   ├── generator.py           mesh-surface path orchestration
│   │   ├── slicer.py              trimesh plane intersection + face filtering
│   │   ├── stitcher.py            segment graph-walk → ordered polylines
│   │   ├── connector.py           pass-to-pass connectors
│   │   └── path_model.py          PaintPass, Connection, PaintRoute dataclasses
│   └── export/
│       ├── json_export.py
│       └── csv_export.py
├── models/
│   └── mesh_model.py          single source of truth for the loaded mesh
└── tests/                     pytest unit tests
```

---

## Supported file formats

| Format | Import | Export |
|---|---|---|
| STL (binary or ASCII) | ✓ | — |
| OBJ (single- or multi-body) | ✓ | — |
| JSON (toolpath) | — | ✓ |
| CSV (toolpath) | — | ✓ |

Multi-body OBJ files are merged into a single mesh at load time via `trimesh.load(force='mesh')`. All distances are stored and exported in millimetres; unit conversion is display-only.

---

## Tech stack

| Layer | Library |
|---|---|
| GUI | PySide6 (Qt 6 ≥ 6.6) |
| 3D rendering | PyVista + pyvistaqt (≥ 0.44) |
| Mesh I/O & repair | Trimesh (≥ 4.3) |
| Geometry / linear algebra | NumPy (≥ 1.26) |
| Spatial queries | SciPy (≥ 1.11) |

---

## License

Apache 2.0 — © 2024 Saketha Krishna B S  
See [LICENSE](LICENSE) for full terms.
