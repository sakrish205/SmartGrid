# SurfaceCoat — 3D Spray-Paint Path Generator

> Desktop tool for automatically generating spray-paint robot paths over 3D mesh surfaces.
> Built for automotive body panels and similar curved geometry.

![SurfaceCoat screenshot](https://raw.githubusercontent.com/sakrish205/SurfaceCoat/main/screenshot.png)

---

## Features

- **Load any STL / OBJ mesh** — drag-and-drop or File > Open
- **Click bounding-box faces** to select spray regions (TOP, BOTTOM, FRONT, REAR, LEFT, RIGHT)
- **Lawnmower passes** — alternating blue / red horizontal sweeps following the surface
- **Vertical passes (crosshatch)** — enable vertical width to generate a second perpendicular set of passes (green / orange), creating a full crosshatch pattern
- **Pink connectors** between passes for clear robot-path readability
- **Spray width control** — horizontal width sets the pass spacing; optional vertical width generates perpendicular passes at its own spacing
- **Show Grid** — overlay a grey cell grid on selected faces to preview pass divisions before generating
- **Path target** — generate paths on the flat Boundary Box plane or on the actual Mesh Surface
- **Pick Faces** — click individual mesh triangles for fine-grained selection
- **Clear Path** — remove rendered paths without losing region selection
- **3D orientation axes** in the top-right viewport corner
- **Export** to JSON or CSV for downstream robot tooling (ROS, RoboDK, etc.)

---

## Pass colour legend

| Colour | Meaning |
|---|---|
| Blue | Horizontal pass — forward direction |
| Red | Horizontal pass — reverse direction |
| Green | Vertical pass — forward direction |
| Orange | Vertical pass — reverse direction |
| Pink | Connector between passes |

---

## Tech Stack

| Layer | Library |
|---|---|
| GUI | PySide6 (Qt 6) |
| 3D rendering | PyVista + pyvistaqt |
| Mesh processing | Trimesh |
| Geometry math | NumPy + SciPy |
| Language | Python 3.12 |

---

## Installation

```bash
pip install PySide6 pyvista pyvistaqt "trimesh[easy]" numpy scipy
```

---

## Running

```bash
python main.py
```

---

## Workflow

1. **Open** an STL or OBJ file (File > Open or drag-and-drop).
2. **Select up-axis** in the dialog that appears (default: Z).
3. **Click a green bounding-box face** — it turns yellow to confirm selection. Use the sidebar checkboxes for quick multi-face selection.
4. **Set spray width** in the Path Settings panel.
   - *Horizontal width* (required) — spacing between horizontal passes.
   - *Vertical width* (optional) — enables a perpendicular set of passes at this spacing (crosshatch).
5. *(Optional)* Check **Show grid** to preview the pass-width cell divisions on the selected face.
6. Choose **Boundary Box** (flat, instant) or **Mesh Surface** (follows curvature, slower).
7. Click **GENERATE PATH** — passes and connectors appear on the model.
8. Click **Clear Path** to remove the paths without losing your region selection.
9. **Export** via File > Export JSON or Export CSV.

---

## Project Structure

```
SurfaceCoat/
├── main.py                    # Entry point
├── app/
│   ├── main_window.py         # Top-level window, signal wiring
│   ├── ui/
│   │   ├── viewer.py          # PyVista QtInteractor wrapper
│   │   ├── surface_panel.py   # Region checkboxes + Pick Faces
│   │   ├── parameter_panel.py # Spray width, path target, Generate / Clear buttons
│   │   └── status_panel.py    # Pass count / length display
│   ├── mesh/
│   │   ├── loader.py          # trimesh load + repair
│   │   ├── preprocessor.py    # Cached normals, centroids, bbox
│   │   └── regions.py         # Auto face → region classification
│   ├── path/
│   │   ├── bbox_generator.py  # Flat paths on bbox face planes (horiz + vert crosshatch)
│   │   ├── generator.py       # Mesh-surface path orchestration
│   │   ├── slicer.py          # trimesh plane intersections
│   │   ├── connector.py       # Pass-to-pass boundary connectors
│   │   └── path_model.py      # PaintPass / Connection / PaintRoute dataclasses
│   └── export/
│       ├── json_export.py
│       └── csv_export.py
├── models/
│   └── mesh_model.py          # Single source of truth for mesh state
└── tests/                     # 11 pytest tests (all passing)
```

---

## Author

Sakeetharan — [GitHub](https://github.com/sakrish205/SurfaceCoat)
