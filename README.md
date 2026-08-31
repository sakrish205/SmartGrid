# SurfaceCoat

A desktop tool for generating spray-paint robot paths over 3D mesh surfaces. Built for automotive body panels.

![SurfaceCoat screenshot](https://raw.githubusercontent.com/sakrish205/SurfaceCoat/main/screenshot.png)

---

## What it does

You load a 3D mesh (STL or OBJ), click the faces of the bounding box you want to spray, set a width, and the tool generates the robot path automatically. Export to JSON or CSV for your robot controller.

---

## Features

- Load STL / OBJ — drag-and-drop or File > Open
- Click the green bounding box faces to select spray regions (TOP, BOTTOM, FRONT, REAR, LEFT, RIGHT)
- Selected face turns yellow; use the sidebar checkboxes for quick multi-select
- Three pass directions — Horizontal, Vertical, or Both (crosshatch)
- Lawnmower pattern — passes alternate direction automatically
- Pink connectors link passes within a set; no connectors between horizontal and vertical sets
- Show Grid — preview the pass-width cell divisions on a selected face before generating
- Path target — Boundary Box (flat, instant) or Mesh Surface (follows actual curvature)
- Clear Path — removes all rendered paths without losing your region selection
- Export paths to JSON or CSV

---

## Pass colour legend

| Colour | Meaning |
|---|---|
| Blue | Horizontal pass — forward |
| Red | Horizontal pass — reversed |
| Green | Vertical pass — forward |
| Orange | Vertical pass — reversed |
| Pink | Connector between passes |

---

## Tech stack

| | |
|---|---|
| GUI | PySide6 (Qt 6) |
| 3D rendering | PyVista + pyvistaqt |
| Mesh processing | Trimesh |
| Geometry | NumPy + SciPy |
| Language | Python 3.12 |

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

1. Open an STL or OBJ file (File > Open or drag-and-drop)
2. Choose which axis is UP in the dialog (default: Z)
3. Click a green bounding-box face — it turns yellow. Use the sidebar checkboxes for multi-select
4. Set spray width. Choose Horizontal, Vertical, or Both
5. If Both is selected a second V width field appears for the perpendicular spacing
6. Tick **Show grid** to preview the pass divisions on the face
7. Choose Boundary Box (flat paths) or Mesh Surface (curved paths)
8. Click **GENERATE PATH**
9. Click **Clear Path** to remove paths without losing your selection
10. Export via File > Export JSON or Export CSV

---

## Understanding the exports

### JSON

The JSON file is structured in three levels: a top-level summary, then a list of routes, and inside each route a list of passes and connections.

```
generated file
├── version, author, generated_at
├── summary
│   ├── total_routes, total_passes, total_connections
│   ├── total_length_mm
│   ├── regions        e.g. ["TOP", "FRONT"]
│   └── directions     e.g. ["horizontal", "vertical"]
└── routes[]
    ├── route_index    0 = first route (e.g. horizontal), 1 = second (e.g. vertical)
    ├── region_id      which bbox face this route covers
    ├── direction      "horizontal" or "vertical"
    ├── spacing_mm     step distance between passes
    ├── total_length_mm
    ├── passes[]
    │   ├── id, direction, is_forward, length_mm
    │   ├── start      [x, y, z] — start point (convenience field)
    │   ├── end        [x, y, z] — end point (convenience field)
    │   └── points     [[x,y,z], [x,y,z], ...]  full point list
    └── connections[]
        ├── id, from_pass_id, to_pass_id, is_air_move, length_mm
        ├── start      [x, y, z]
        ├── end        [x, y, z]
        └── points     [[x,y,z], ...]
```

**Reading it in Python:**

```python
import json

with open("output.json") as f:
    data = json.load(f)

print(data["summary"])

for route in data["routes"]:
    print(f"Route {route['route_index']} — {route['region_id']} {route['direction']}")
    for pass_ in route["passes"]:
        start = pass_["start"]   # [x, y, z]
        end   = pass_["end"]     # [x, y, z]
        print(f"  Pass {pass_['id']} forward={pass_['is_forward']}  "
              f"length={pass_['length_mm']} mm")
        print(f"    from {start} to {end}")
```

**Key fields for a robot controller:**

- `segment_type` — `pass` means spray gun ON, `connection` means travel move
- `is_forward` — True = sweep one way, False = reversed (lawnmower flip)
- `is_air_move` — True = gun off, lift away from surface
- `start` / `end` on each pass — the two endpoints; `points` has the full path if it curves

---

### CSV

One row = one 3D point. Passes and connections are written in route order.

```
segment_type, route_index, region, direction, pass_id, is_forward,
sub_index, conn_id, is_air_move, length_mm, pt_idx, x, y, z
```

| Column | What it contains |
|---|---|
| `segment_type` | `pass` = spraying move. `connection` = travel between passes |
| `route_index` | Which route. 0 = first (horizontal), 1 = second (vertical for crosshatch) |
| `region` | Bbox face: TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT |
| `direction` | `horizontal` or `vertical`. Blank on connection rows |
| `pass_id` | Pass number within the route |
| `is_forward` | TRUE = forward sweep, FALSE = reversed |
| `sub_index` | 0 for standard passes. Greater than 0 for extra chains through holes |
| `conn_id` | Connection ID. Blank on pass rows |
| `is_air_move` | TRUE = gun off travel. Blank on pass rows |
| `length_mm` | Length of this segment. Written only on the first point (`pt_idx=0`). Blank on subsequent points |
| `pt_idx` | Point index within the segment. 0 = start, 1 = end for a straight pass |
| `x, y, z` | World coordinates in mm, rounded to 4 decimal places |

**Reading it in Python:**

```python
import csv

with open("output.csv", newline="") as f:
    rows = list(csv.DictReader(f))

# All spray moves only
spray_rows = [r for r in rows if r["segment_type"] == "pass"]

# Horizontal passes only
h_rows = [r for r in rows if r["direction"] == "horizontal"]

# Vertical passes only
v_rows = [r for r in rows if r["direction"] == "vertical"]

# One row per pass with its length (filter to start points)
pass_starts = [r for r in rows
               if r["segment_type"] == "pass" and r["pt_idx"] == "0"]
total_spray_length = sum(float(r["length_mm"]) for r in pass_starts)
print(f"Total spray length: {total_spray_length:.1f} mm")
```

**Useful Excel filters:**

- Filter `segment_type = pass` then sum `length_mm` at `pt_idx = 0` → total spray distance
- Filter `direction = horizontal` or `direction = vertical` → isolate each pass set
- Filter `route_index = 0` → first generated route only
- Filter `is_forward = FALSE` → all reversed passes (to check lawnmower alternation)

---

## Project structure

```
SurfaceCoat/
├── main.py
├── app/
│   ├── main_window.py         top-level window and signal wiring
│   ├── ui/
│   │   ├── viewer.py          3D viewport (PyVista)
│   │   ├── surface_panel.py   region checkboxes
│   │   ├── parameter_panel.py spray width, direction, generate/clear buttons
│   │   └── status_panel.py    pass count and length display
│   ├── mesh/
│   │   ├── loader.py          file loading and repair
│   │   ├── preprocessor.py    cached normals, centroids, bounding box
│   │   └── regions.py         face-to-region classification
│   ├── path/
│   │   ├── bbox_generator.py  flat paths on bbox face planes
│   │   ├── generator.py       mesh-surface path orchestration
│   │   ├── slicer.py          trimesh plane intersections
│   │   ├── connector.py       pass-to-pass connectors
│   │   └── path_model.py      PaintPass, Connection, PaintRoute data classes
│   └── export/
│       ├── json_export.py
│       └── csv_export.py
├── models/
│   └── mesh_model.py
└── tests/                     11 unit tests, all passing
```

---

## Author

**Saketha Krishna B S**
