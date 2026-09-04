# SmartGrid — 3D Surface Grid & Spray Path Generator

> Automated offline path planning for spray-paint robots over automotive 3D mesh surfaces.  
> Built from scratch as an academic research project.

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)
[![Python](https://img.shields.io/badge/Python-3.12-blue)](https://www.python.org/)
[![PySide6](https://img.shields.io/badge/GUI-PySide6-green)](https://pypi.org/project/PySide6/)

---

## What it does

SmartGrid loads a 3D mesh (STL or OBJ), lets you select bounding box faces as spray regions, and generates a complete lawnmower robot path automatically. The toolpath exports to JSON or CSV for any robot controller.

Designed for automotive body panel painting — bumpers, bonnets, doors — where the robot needs a dense, gap-free coverage pattern with alternating pass directions.

---

## Screenshot

```
[ Ribbon bar: File | View | Surface Selection | Path Settings | Path | Statistics | Export ]
[                                                                                          ]
[                          3D Viewport (PyVista / VTK)                                    ]
[          Selected face highlighted yellow — passes shown in blue/red                    ]
[                                                                                          ]
[ Status bar: Loaded: GT86FrontBumper.stl | 48,320 triangles | 12 passes, 11 connections  ]
```

---

## Features

| Feature | Description |
|---|---|
| STL / OBJ loading | Drag-and-drop or File › Open. Trimesh repairs normals on load |
| Up-axis selection | Choose X / Y / Z at load time — not hardcoded |
| Bbox face selection | Click a face in 3D or use ribbon toggle buttons (TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT) |
| Select Faces mode | Click directly on the bounding box in 3D; camera freezes so you don't accidentally orbit |
| Lawnmower path | Passes alternate direction automatically (boustrophedon pattern) |
| Pitch control | Set spray width in mm / cm / m / in / ft — value converts without changing physical distance |
| CW / CCW sweep | Flip the starting direction of the whole route |
| Grid preview | Show the pass-width divisions on the face before generating |
| Direction arrows | Chevron tick marks along each pass show travel direction |
| Boundary Box mode | Flat paths on the bbox face — instant, no geometry needed |
| Mesh Surface mode | Paths follow actual mesh curvature via trimesh plane intersection |
| Pink connectors | Link end of each pass to start of the next (no air moves in bbox mode) |
| Export JSON | Full structured toolpath with metadata, pass details, and 3D points |
| Export CSV | One row per 3D point, compatible with Excel and any robot preprocessor |
| View Settings | Per-element colour and line width control, live preview |
| Non-blocking load | File reading runs in a QThread worker — UI never freezes |

---

## Tech stack

| Layer | Library | Version |
|---|---|---|
| GUI | PySide6 (Qt 6) | ≥ 6.6 |
| 3D rendering | PyVista + pyvistaqt | ≥ 0.44 |
| Mesh I/O & repair | Trimesh | ≥ 4.3 |
| Geometry math | NumPy | ≥ 1.26 |
| Spatial queries | SciPy | ≥ 1.11 |
| Language | Python | 3.12 |

---

## Installation

```bash
pip install PySide6 pyvista pyvistaqt "trimesh[easy]" numpy scipy
```

## Running

```bash
python main.py
```

---

## How to use

1. **Open a mesh** — File › Open or drag an STL / OBJ file onto the window
2. **Set up-axis** — choose which axis is UP in your file (default: Z)
3. **Select regions** — click a face of the blue bounding box, or use the ribbon toggle buttons. Selected faces turn yellow
4. **Set pitch** — enter spray width and choose your unit
5. **Preview** — tick **Grid** to see pass divisions on the face before generating
6. **Generate** — click **Generate Path**; blue = forward pass, red = reversed pass, pink = connector
7. **Adjust** — change pitch or flip CW/CCW direction and regenerate
8. **Export** — Export JSON or Export CSV from the ribbon

---

## Path generation algorithm

### Bbox mode (flat — instant)

1. Identifies the bbox face plane for the selected region (TOP, FRONT, etc.)
2. Computes step positions along the perpendicular axis:
   - First pass at `face_min + pitch / 2` (centred in first strip)
   - Subsequent passes every `pitch` mm
3. Each pass is a straight line sweeping the full face width
4. Passes alternate direction: even index = forward, odd index = reversed (lawnmower)
5. A connector is drawn as a straight line from the end of pass N to the start of pass N+1
6. The snake pattern means connectors alternate between the left and right edges of the face

### Mesh Surface mode (curved — uses geometry)

1. Computes slice plane positions using the region's actual bounding extent along the step axis
2. For each position, calls `trimesh.intersections.mesh_plane` with `return_faces=True`
3. Filters returned segments to only those whose source triangle belongs to the selected region
4. Stitches unordered segments into ordered polylines (graph-walk with quantised endpoints)
5. Alternates direction by plane index (not pass count) so holes / sub-passes don't break the pattern
6. Connects primary passes (sub_index = 0) with boundary-following connectors

### Multi-region

Multiple regions (e.g. TOP + FRONT together) each get an independent route. All routes are displayed simultaneously.

---

## Understanding the exports

### JSON

```
output.json
├── version, author, generated_at
├── summary
│   ├── total_routes, total_passes, total_connections
│   ├── total_length_mm
│   ├── regions        e.g. ["TOP", "FRONT"]
│   └── directions     e.g. ["horizontal"]
└── routes[]
    ├── route_index    0 = first route, 1 = second, ...
    ├── region_id      which bbox face
    ├── spacing_mm     pitch between passes
    ├── total_length_mm
    ├── passes[]
    │   ├── id, is_forward, length_mm
    │   ├── start / end    [x, y, z] convenience fields
    │   └── points         [[x,y,z], ...]  full point list
    └── connections[]
        ├── id, from_pass_id, to_pass_id, is_air_move, length_mm
        ├── start / end    [x, y, z]
        └── points         [[x,y,z], ...]
```

**Reading in Python:**

```python
import json

with open("output.json") as f:
    data = json.load(f)

for route in data["routes"]:
    for p in route["passes"]:
        print(p["start"], "→", p["end"], "  forward:", p["is_forward"])
```

**Robot controller key fields:**

| Field | Meaning |
|---|---|
| `segment_type` | `pass` = spray gun ON · `connection` = travel move |
| `is_forward` | True = sweep one way, False = reversed |
| `is_air_move` | True = gun off, lift away from surface |
| `points` | Full 3D path — more than 2 points on curved mesh paths |

---

### CSV

One row = one 3D point, written in route order.

| Column | What it contains |
|---|---|
| `segment_type` | `pass` = spraying · `connection` = travel between passes |
| `route_index` | Which route (0-based) |
| `region` | Bbox face: TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT |
| `direction` | `horizontal` or `vertical` |
| `pass_id` | Pass number within route |
| `is_forward` | TRUE = forward sweep, FALSE = reversed |
| `sub_index` | 0 = standard pass. >0 = extra chain through a hole |
| `conn_id` | Connection ID (blank on pass rows) |
| `is_air_move` | TRUE = gun off travel (blank on pass rows) |
| `length_mm` | Length of this segment — written only at `pt_idx = 0` |
| `pt_idx` | Point index within the segment (0 = start) |
| `x, y, z` | World coordinates in mm, 4 decimal places |

**Useful Excel filters:**

- `segment_type = pass`, sum `length_mm` at `pt_idx = 0` → total spray distance
- `is_forward = FALSE` → all reversed passes (verify lawnmower alternation)
- `route_index = 0` → first route only

---

## Project structure

```
SmartGrid/
├── main.py                        entry point
├── app/
│   ├── main_window.py             window layout, signal wiring, QThread workers
│   ├── ui/
│   │   ├── ribbon.py              single Ribbon bar (all controls)
│   │   ├── viewer.py              3D viewport — PyVista / VTK actor management
│   │   └── view_settings_dialog.py  colour + line-width picker
│   ├── mesh/
│   │   ├── loader.py              trimesh load + repair
│   │   ├── preprocessor.py        cached normals, centroids, bbox (computed once)
│   │   └── regions.py             face-to-region classification
│   ├── path/
│   │   ├── bbox_generator.py      flat lawnmower paths on bbox face planes
│   │   ├── generator.py           mesh-surface path orchestration
│   │   ├── slicer.py              trimesh plane intersection + face filtering
│   │   ├── stitcher.py            segment graph-walk → ordered polylines
│   │   ├── connector.py           pass-to-pass connectors
│   │   └── path_model.py          PaintPass, Connection, PaintRoute dataclasses
│   └── export/
│       ├── json_export.py
│       └── csv_export.py
├── models/
│   └── mesh_model.py              single source of truth for loaded mesh
└── tests/                         unit tests (pytest)
```

---

## Development process (academic)

This project was built iteratively from scratch as a research exercise in offline robot path planning. The process followed below is documented here for academic reproducibility.

### Build order

| Phase | What was built | Key decision |
|---|---|---|
| 1 | Mesh loading + preprocessing | `trimesh` for repair; cached `MeshData` to avoid recomputing normals |
| 2 | Region classification | Dot-product of face normals against bbox axes → TOP / BOTTOM / etc. |
| 3 | Segment stitcher | Graph-walk with quantised endpoints; handles holes and multi-chain slices |
| 4 | Slicer | `trimesh.intersections.mesh_plane` + face-index filter for region masking |
| 5 | Bbox flat path generator | 2-point passes, boustrophedon alternation, `step_min + pitch/2` centering |
| 6 | PySide6 + PyVista window | `QtInteractor` embedded in `QVBoxLayout`; all actors stored in `_actors` dict |
| 7 | VTK face picking | `vtkCellPicker` on invisible solid box; `vtkInteractorStyleUser` to freeze camera |
| 8 | Ribbon UI | Single `SmartRibbon(QWidget)` replaces sidebar; SVG icons rendered via `QSvgRenderer` |
| 9 | Non-blocking load | `_LoadWorker(QThread)` emits `progress` signal → status bar (no overlay) |
| 10 | Export | JSON (`json.JSONEncoder` subclass for `np.ndarray`) and flat CSV |

### Design principles applied

- **Single source of truth** — `MeshModel` owns the mesh; `MeshData` is read-only after preprocessing
- **Actor lifetime** — every VTK actor is immediately stored in `self._actors[key]`; never let them go out of scope
- **Thread safety** — PyVista / VTK is called only from the main Qt thread; workers emit signals carrying plain Python objects
- **No double-fire** — `QButtonGroup.idClicked` used instead of `QRadioButton.toggled` to emit `sweep_changed` exactly once per click
- **Signal isolation** — `blockSignals(True/False)` wraps programmatic UI updates that must not trigger downstream regeneration

### Known approximations (scope of MVP)

- Bbox mode places paths on the face plane, not the curved mesh surface
- Pass spacing is measured along a straight world axis, not arc-length along the surface
- Connectors in bbox mode are straight-line transit moves, not surface-following
- No robot kinematic constraints (reach envelope, joint limits, overspray overlap)

---

## License — Apache 2.0 (Academic)

This project is released under the **Apache License, Version 2.0**.

### What this means for academic use

You are free to:
- **Study** — read, run, and modify the code for research and learning
- **Cite** — use results or methods in papers (see attribution below)
- **Fork** — build derivative research tools on top of SmartGrid
- **Distribute** — share copies or modified versions, including in course materials

You must:
- **Retain the copyright notice** in every source file you distribute
- **State changes** — if you modify files, add a note saying what you changed
- **Include the license text** — keep this section (or a link to apache.org/licenses/LICENSE-2.0) in any redistribution

You must not:
- Remove or alter the `Copyright 2024 Saketha Krishna B S` notice
- Use the author's name to endorse derived work without permission

### How to maintain Apache 2 compliance in your fork

1. **Keep the header** — every `.py` file you distribute must retain or add:

   ```python
   # Copyright 2024 Saketha Krishna B S
   #
   # Licensed under the Apache License, Version 2.0 (the "License");
   # you may not use this file except in compliance with the License.
   # You may obtain a copy of the License at
   #
   #     http://www.apache.org/licenses/LICENSE-2.0
   ```

2. **Mark your changes** — for any file you modify, add a line below the header:

   ```python
   # Modified by <Your Name>, <Year> — <brief description of change>
   ```

3. **Add a NOTICE file** if you publish a packaged/installed version. It should contain:

   ```
   SmartGrid
   Copyright 2024 Saketha Krishna B S
   This product includes software developed by Saketha Krishna B S.
   ```

4. **Transitive dependencies** — all runtime libraries used (PySide6, PyVista, Trimesh, NumPy, SciPy) carry their own open-source licenses. They are compatible with Apache 2.0. If you redistribute a bundled executable (e.g. PyInstaller build), include their license texts in a `third_party_licenses/` folder.

### Academic citation

If you use SmartGrid in a paper or thesis, please cite:

```
Saketha Krishna B S. SmartGrid — 3D Surface Grid & Spray Path Generator. 2024.
https://github.com/sakrish205/SmartGrid
```

BibTeX:

```bibtex
@software{smartgrid2024,
  author  = {Saketha Krishna B S},
  title   = {SmartGrid — 3D Surface Grid \& Spray Path Generator},
  year    = {2024},
  url     = {https://github.com/sakrish205/SmartGrid},
  license = {Apache-2.0}
}
```

---

## Author

**Saketha Krishna B S**  
Email: girimech4305@gmail.com  
GitHub: [sakrish205](https://github.com/sakrish205)
