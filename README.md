# SmartGrid — 3D Spray Path Generator

**Mechanical Engineering | Manufacturing Automation | Robotics**

Offline toolpath planning for robotic spray operations over 3D mesh surfaces. Loads STL/OBJ models, selects spray regions, generates a boustrophedon lawnmower toolpath, maintains spray-gun standoff, and exports robot-ready paths to JSON or CSV.

SmartGrid focuses on the **manufacturing process-planning problem** of generating systematic spray-paint trajectories for flat, curved, blended, and complex 3D component surfaces.

[![Python](https://img.shields.io/badge/Python-3.12-blue)](https://www.python.org/)
[![PySide6](https://img.shields.io/badge/GUI-PySide6-green)](https://pypi.org/project/PySide6/)
[![Version](https://img.shields.io/badge/version-1.3-informational)]()
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

---

## Installation

### Requirements

- Python 3.12

Install dependencies:

```bash
pip install PySide6 pyvista pyvistaqt "trimesh[easy]" numpy scipy
```

Launch:

```bash
python main.py
```

Or run `run.bat` on Windows.

---

## Engineering Problem

Robotic spray painting requires controlled coverage, consistent pass spacing, suitable spray-gun standoff, and an ordered trajectory that follows the workpiece geometry.

SmartGrid addresses this by converting a **3D mesh of the manufactured component into a structured robotic spray toolpath**.

### Our Approach

```text
3D Mesh
   ↓
Workpiece / Region Selection
   ↓
Spray Parameters
   ↓
Surface-Based Path Generation
   ↓
Boustrophedon Coverage
   ↓
Standoff Control
   ↓
Path Processing
   ↓
3D Validation
   ↓
JSON / CSV Export
```

---

## Workflow

1. **Open mesh** — File › Open or drag an STL / OBJ onto the window
2. **Set up-axis** — select which world axis is vertical (X / Y / Z)
3. **Select regions** — toggle TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT or click the bounding box in the viewport
4. **Set parameters** — pitch, unit, standoff, sweep direction
5. **Preview** — enable Grid to verify pass spacing before generating
6. **Generate Path**
7. **Export JSON or CSV**

---

## Path Generation

SmartGrid provides four toolpath generation modes for different workpiece geometries.

### Boundary Box

Flat lawnmower passes on the axis-aligned bounding-box face plane.

- Uses the outermost mesh extent along the face normal.
- Passes are spaced by the specified pitch.
- Direction alternates between passes.
- Straight-line connectors join pass endpoints.
- No mesh geometry is queried during generation.

**Best for:** flat panels and sheet-metal surfaces that do not deviate significantly from the bounding face plane.

### Face Grid — Adaptive

Surface-following passes using shadow projection on a mean-normal spray-plane basis.

- Calculates the mean outward normal of the selected forward-facing region.
- Builds a pass/step coordinate basis.
- Projects region vertices into the spray-plane coordinate system.
- Divides the step extent into pitch-based bands.
- Tracks the outermost surface depth for each band.
- Applies the specified standoff.

**Best for:** curved panels with a dominant normal direction, such as bonnets, bumpers, and side panels.

**Limitation:** straight passes can float on surfaces with significant curvature along the sweep direction.

### Face Grid — Conform

Trimesh plane-intersection slicing on all geometrically outward-facing faces of the selected region.

- Uses cutting planes at pitch intervals.
- Intersects the mesh and stitches resulting segments.
- Applies RDP simplification.
- Maintains boustrophedon ordering.
- Applies standoff using the nearest mesh-face normal.

**Best for:** blended edges and compound curves where classifier-based paths may stop short of the actual surface extent.

### Mesh Surface

Trimesh plane-intersection slicing on the classifier-assigned region faces.

- Generates cutting planes at pitch intervals.
- Intersects the mesh and filters segments to the selected region.
- Stitches and simplifies the resulting paths.
- Removes small corner fragments.
- Uses boustrophedon ordering and straight-line connectors.

**Best for:** complex curved surfaces and parts with holes or cutouts.

---

## Boustrophedon Toolpath

The spray passes alternate direction to create a continuous lawnmower-style trajectory:

```text
Pass 1  ───────────────────→
                              │
Pass 2  ←────────────────────
                              │
Pass 3  ───────────────────→
                              │
Pass 4  ←────────────────────
```

The centre-to-centre spacing between adjacent passes is controlled by **Spray Width (Pitch)**.

This provides systematic surface coverage while reducing unnecessary repositioning between adjacent passes.

---

## Standoff Control

SmartGrid offsets every generated toolpath point outward from the reference surface by the specified spray-gun standoff.

```text
P′ = P + standoff_mm × n̂
```

- `P` — original toolpath point
- `P′` — standoff-adjusted point
- `standoff_mm` — required distance
- `n̂` — relevant outward surface normal

Normal selection depends on the path mode:

| Path Mode | Normal |
|---|---|
| Boundary Box | Face-axis unit vector |
| Face Grid | Mean face normal |
| Mesh Surface | Nearest mesh-face normal |
| Conform | Nearest mesh-face normal |

Default standoff is `0`.

---

## Controls

| Control | Function |
|---|---|
| **TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT** | Toggle bounding-box faces as spray regions |
| **Select Faces** | 3D pick mode for selecting regions |
| **Unit** | mm / cm / m / in / ft |
| **Spray Width (Pitch)** | Centre-to-centre distance between adjacent passes |
| **Sweep** | CW / CCW starting direction |
| **Waypoints** | Enable uniform resampling |
| **Interval** | Waypoint resampling spacing |
| **Grid** | Display pitch-division lines |
| **Arrows** | Show travel direction |
| **Generate Path** | Generate the selected toolpath |
| **Clear Path** | Remove generated paths |
| **View Settings** | Configure 3D display and mesh rendering |
| **Export JSON / CSV** | Export the generated trajectory |

---

## Path Processing

Generated geometry is processed into an ordered trajectory using:

- Segment stitching
- RDP simplification
- Corner-fragment filtering
- Boustrophedon ordering
- Pass-to-pass connectors
- Optional uniform waypoint resampling

The **Generate Path** operation runs in a background thread so the UI remains responsive.

---

## 3D Visualization

The integrated PyVista/VTK viewer provides visual validation of:

- Workpiece mesh
- Selected spray regions
- Pitch grid
- Spray passes
- Travel direction
- Connectors
- Waypoints
- Mesh rendering and display settings

This allows the generated trajectory to be inspected before export.

---

## Robot Path Export

SmartGrid exports the generated trajectory in two formats:

### JSON

Structured toolpath containing:

- Metadata and summary
- Routes and regions
- Spray passes
- Pass direction
- Pass lengths
- 3D waypoints
- Connections / transit moves
- Air-move information

### CSV

One row per trajectory point containing:

- Segment type
- Region
- Pass ID
- Direction
- Segment length
- Point index
- X, Y, Z coordinates

All internal distances are maintained in millimetres; display/input unit conversion does not change the physical distance.

> **Integration note:** SmartGrid is a **toolpath planning system**, not a complete robot controller. JSON/CSV output provides an intermediate trajectory representation for downstream robot-controller integration.

---

## File Formats

| Format | Import | Export |
|---|---:|---:|
| STL (binary or ASCII) | ✓ | — |
| OBJ (single- or multi-body) | ✓ | — |
| JSON (toolpath) | — | ✓ |
| CSV (toolpath) | — | ✓ |

Multi-body OBJ files are merged at load via `trimesh.load(force='mesh')`.

---

## Technology Stack

### Engineering

- 3D CAD / mesh geometry
- Surface-normal analysis
- Robotic process planning
- Spray-path planning
- Surface coverage planning
- Standoff control
- Manufacturing automation

### Software

- Python 3.12
- PySide6
- PyVista / VTK
- Trimesh
- NumPy
- SciPy

---

## Applications

- Robotic spray painting
- Automotive body-panel coating
- Industrial component coating
- Sheet-metal finishing
- Automated manufacturing cells
- Surface-treatment process planning
- Robotic manufacturing research

---

## Advantages

- **Offline operation** — no cloud or paid API dependency
- **Geometry-driven planning** — paths are generated from 3D workpiece geometry
- **Multiple path strategies** — supports flat, curved, blended, and complex surfaces
- **Controlled pitch and standoff**
- **3D path visualization and validation**
- **JSON/CSV robot-path export**
- **Modular architecture**

---

## Limitations

SmartGrid currently focuses on **geometric toolpath generation**. It does not itself provide:

- Robot-specific inverse kinematics
- Robot-controller post-processing
- Collision detection and avoidance
- Dynamic robot reachability analysis
- Spray deposition physics
- Paint-thickness prediction
- Paint-flow modelling
- Complete robot-cell simulation

The Adaptive Face Grid method can float on surfaces with significant curvature along the sweep direction because individual passes are straight.

---

## Future Scope

- Robot-specific post-processors for industrial robot platforms
- TCP orientation and inverse-kinematics integration
- Collision and reachability checking
- Adaptive spray pitch and speed optimisation
- Spray deposition / coating-thickness modelling
- Robot simulation and digital-twin integration
- CAD/CAM and manufacturing-cell integration

---

## Project Structure

```text
SmartGrid/
├── app/
│   ├── main_window.py
│   ├── ui/
│   │   ├── ribbon.py
│   │   ├── viewer.py
│   │   └── view_settings_dialog.py
│   ├── mesh/
│   │   ├── loader.py
│   │   ├── preprocessor.py
│   │   └── regions.py
│   ├── path/
│   │   ├── bbox_generator.py
│   │   ├── face_grid_generator.py
│   │   ├── generator.py
│   │   ├── slicer.py
│   │   ├── stitcher.py
│   │   ├── connector.py
│   │   └── path_model.py
│   └── export/
│       ├── json_export.py
│       └── csv_export.py
├── models/
│   └── mesh_model.py
└── tests/
```

---

## Engineering Significance

SmartGrid connects **3D component geometry** with **automated manufacturing process planning**:

```text
3D COMPONENT
     ↓
SURFACE GEOMETRY
     ↓
SPRAY PARAMETERS
     ↓
TOOLPATH PLANNING
     ↓
ROBOT TRAJECTORY
     ↓
AUTOMATED COATING
```

The project is therefore best positioned as a **Mechanical Engineering + Manufacturing Automation + Robotics** application, with Python and computational geometry providing the implementation.

---

## License

Apache 2.0 — © 2026 Saketha Krishna B S

See [LICENSE](LICENSE) for full terms.
