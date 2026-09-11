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

SmartGrid provides four toolpath generation modes for different workpiece geometries. Each mode makes a different trade-off between speed, accuracy, and surface fidelity.

---

### Boundary Box

The fastest mode. Passes are computed purely from the axis-aligned bounding box of the selected region — the mesh geometry is never queried after region selection.

**How it works:**

1. The selected region (e.g. TOP) maps to a face axis and sign. The spray plane is fixed at `bbox_max[face_axis] + standoff_mm` — the outermost extent of the mesh along that axis.
2. The two remaining world axes become the *pass axis* (left–right sweep direction) and the *step axis* (spacing direction). For TOP/BOTTOM the step axis is forward; for side faces it is the up axis.
3. Pass positions are evenly spaced from `step_min + pitch/2` to `step_max` at intervals of `pitch`. This centres the passes within the region extent.
4. Each pass is a straight line from `pass_min` to `pass_max` at the computed step position, fixed on the spray plane.
5. Passes alternate direction (boustrophedon): even-index passes run forward, odd-index passes run in reverse.
6. Connectors are straight-line air moves between the end of one pass and the start of the next.

```python
# core: bbox_generator.py — _make_passes
face_pos = bbox_max[face_axis] + standoff_mm          # fixed face plane
step_positions = np.arange(step_min + pitch/2, step_max, pitch)
for i, step_pos in enumerate(step_positions):
    is_forward = (i + direction_offset) % 2 == 0      # boustrophedon
    pt_a[face_axis] = pt_b[face_axis] = face_pos
    pt_a[step_axis] = pt_b[step_axis] = step_pos
    pt_a[pass_axis] = pass_min;  pt_b[pass_axis] = pass_max
```

> **Boustrophedon traversal** — Choset, H. (2000). *Coverage of Known Spaces: The Boustrophedon Cellular Decomposition*. Autonomous Robots, 9(3), 247–253.

**Strengths:** Near-instant generation. Predictable, uniform pass spacing. No mesh dependency.  
**Limitation:** All passes lie on the same flat plane. On a curved surface the spray gun distance varies continuously — the path floats above valleys and dips below peaks.  
**Best for:** flat panels, stamped sheet metal, and surfaces where deviation from the bounding face plane is within acceptable standoff tolerance.

---

### Face Grid — Adaptive

Extends Boundary Box by tilting the spray plane to match the actual surface orientation and tracking the outermost surface depth per row. Each pass is still a straight line — no mesh slicing.

**How it works:**

1. The mean outward normal `mean_n` is computed from all forward-facing region faces (faces whose normal has a positive component along the region's face axis). This gives the dominant surface tilt.
2. An orthonormal basis is built: `pass_vec = normalize(mean_n × up)` gives the left–right sweep direction in the tilted plane; `step_vec = normalize(pass_vec × mean_n)` gives the step direction perpendicular to both.
3. All region vertices are projected onto this basis: `pass_proj = v · pass_vec`, `step_proj = v · step_vec`, `depth_proj = v · mean_n`.
4. For each step position, a band of vertices within ±`pitch × 0.65` is collected. The outermost depth in that band (`max(depth_proj)`) becomes the spray plane depth for that row — the pass elevation tracks the actual surface peak.
5. World-space endpoints are reconstructed: `P = row_depth × mean_n + p_min × pass_vec + step_pos × step_vec`.
6. Standoff is added along `mean_n` at point construction time.

```python
# core: face_grid_generator.py — generate_face_grid_route
mean_n   = normalize(mesh.face_normals[forward_faces].mean(axis=0))
pass_vec = normalize(cross(mean_n, up))        # left-right sweep axis
step_vec = normalize(cross(pass_vec, mean_n))  # step axis

for step_pos in step_positions:
    band = verts[|step_proj - step_pos| <= pitch * 0.65]
    row_depth = (band @ mean_n).max() + standoff_mm
    pt_a = row_depth*mean_n + p_min*pass_vec + step_pos*step_vec
    pt_b = row_depth*mean_n + p_max*pass_vec + step_pos*step_vec
```

> **Orthonormal basis projection** — standard linear algebra (cross product basis construction). No single attribution.

**Strengths:** Fast (no mesh slicing). Spray plane tilts to match the surface — correct standoff at the peak of each row. Step spacing is measured in surface coordinates, not world axis coordinates.  
**Limitation:** Each pass is still a straight line. On a surface with significant curvature *along* the sweep direction (e.g. a strongly curved bumper corner), the gun is too close at the ends and too far at the midpoint.  
**Best for:** bonnets, door panels, bumpers — curved surfaces with a single dominant normal where per-row depth tracking is sufficient.

---

### Face Grid — Conform

A hybrid of Adaptive and Mesh Surface. Uses Adaptive's tilted mean-normal basis to compute correct arc-length step spacing, but then cuts actual trimesh plane intersections through the mesh for exact path geometry.

**How it works:**

1. Same mean-normal basis as Adaptive: `mean_n`, `pass_vec`, `step_vec` computed from forward-facing region faces.
2. Step extent is measured along `step_vec` from the selected region's vertices (not a world axis), so step spacing is true arc-length distance on a tilted surface.
3. Cutting planes are oriented with `plane_normal = step_vec` (perpendicular to the step direction) — this is the key difference from Mesh Surface, which always uses a world-axis normal.
4. `trimesh.intersections.mesh_plane` intersects the full mesh against each tilted cutting plane. Returned segments are filtered to the classifier-assigned face set only.
5. Segments are stitched into ordered polylines via a graph-walk on quantised endpoints (tolerance 10⁻⁶ m). Multiple chains per plane level (holes, discontinuities) are handled naturally.
6. Short corner fragments and misaligned sub-passes are dropped. RDP simplification (ε = 0.3 mm) removes jaggies from triangle discretisation.
7. Uniform standoff is applied as `pts += standoff_mm × mean_n` — a single vector shift, not a per-point nearest-face snap.

```python
# core: face_grid_generator.py — generate_conform_route
mean_n, pass_vec, step_vec = _compute_surface_basis(forward_faces, mesh)

for plane_idx, step_pos in enumerate(step_positions):
    # Cutting plane perpendicular to the step direction (tilted, not axis-aligned)
    segments, face_ids = trimesh.intersections.mesh_plane(
        mesh, plane_normal=step_vec,
        plane_origin=step_pos * step_vec, return_faces=True)
    segments = segments[np.isin(face_ids, face_indices)]  # classifier faces only
    chains = _stitch_segments(segments)                   # graph-walk → polylines
    pts = rdp_simplify(chain, eps=0.3)                    # remove micro-jaggies
    pts += standoff_mm * mean_n                           # uniform standoff
```

> **Triangle–plane intersection** — via [Trimesh](https://trimesh.org/) (Dawson-Haggerty et al.). Standard computational geometry.  
> **RDP simplification** — Ramer, U. (1972). *CGIP 1(3)*. Douglas & Peucker (1973). *Cartographica 10(2)*.  
> **Tilted-basis cutting planes** (mean-normal as slice normal) — original combination, not drawn from any published method.

**Strengths:** Paths lie exactly on the mesh surface (not floating). Step spacing is correct arc-length even on tilted surfaces. Handles holes and discontinuities.  
**Limitation:** Slower than Adaptive (one mesh intersection per pass). Standoff is a uniform vector shift — not per-point nearest-face normal projection, so on highly curved surfaces the standoff direction is approximate.  
**Best for:** blended edges, compound curves, and parts where Adaptive paths float mid-pass and Mesh Surface paths stop short of the actual surface extent.

---

### Mesh Surface

The most geometrically faithful mode. Axis-aligned cutting planes intersect the mesh and paths are filtered to exactly the classifier-assigned face set. Every waypoint lies on the actual mesh surface.

**How it works:**

1. The slice axis is determined by region and up-axis: TOP/BOTTOM use the forward axis; side faces (FRONT/REAR/LEFT/RIGHT) use the up axis. Cutting planes are always axis-aligned (world coordinates).
2. Plane positions are spaced at `pitch` intervals from the region's own bounding box along the slice axis, extended ±0.001 mm to ensure boundary triangles are caught.
3. Each `trimesh.intersections.mesh_plane` call returns all segments where the plane cuts the mesh, plus the source face ID of each segment.
4. A two-stage outward-face filter is applied: first, segments whose source face normal does not point outward for the selected region are dropped (prevents paths appearing on the underside of the mesh). Second, a largest-gap cluster analysis on segment heights drops internal ribs and supports that survived the normal filter.
5. Remaining segments are stitched into ordered polylines by a graph-walk on quantised endpoints. Multiple chains per level represent holes (e.g. sunroof cutouts) — they become separate sub-passes, not connected across the gap.
6. Sub-passes shorter than `max(pitch × 10%, 5 mm)` or whose direction deviates more than 65° from the primary pass are dropped as corner fragments.
7. RDP simplification (ε = 0.3 mm) removes discretisation jaggies. Up to 6 sub-passes per slice level are kept.
8. Boustrophedon ordering by plane index — even planes forward, odd planes reversed. Holes within a level do not disrupt the serpentine pattern.

```python
# core: generator.py — generate_route  +  slicer.py
for plane_idx, step_pos in enumerate(step_positions):
    segments, face_ids = trimesh.intersections.mesh_plane(
        mesh, plane_normal=step_normal,
        plane_origin=origin, return_faces=True)
    # Stage 1: region filter
    segments = segments[np.isin(face_ids, region_faces)]
    # Stage 2: outward-face filter + largest-gap cluster (slicer.py)
    segments = slice_region(mesh, region_faces, plane_normal, origin, region_id, up_axis)
    polylines = stitcher.stitch(segments)              # graph-walk → chains
    polylines = _filter_polylines(polylines, pitch)    # drop corner fragments
    pts = rdp_simplify(pts, eps=0.3)                   # RDP ε = 0.3 mm
    is_forward = (plane_idx % 2 == 0)                  # boustrophedon
```

> **Triangle–plane intersection** — via [Trimesh](https://trimesh.org/) (Dawson-Haggerty et al.). Standard computational geometry.  
> **RDP simplification** — Ramer, U. (1972). *CGIP 1(3)*. Douglas & Peucker (1973). *Cartographica 10(2)*.  
> **Boustrophedon traversal** — Choset, H. (2000). *Coverage of Known Spaces*. Autonomous Robots, 9(3), 247–253.

**Strengths:** Most accurate surface tracking — paths follow the mesh in both axes. Handles holes and cutouts correctly. Per-point standoff via nearest mesh-face normal.  
**Limitation:** Slowest mode (one mesh intersection per pass). On tilted surfaces, world-axis step spacing slightly overestimates true arc-length distance.  
**Best for:** complex curved surfaces, parts with holes or cutouts, and any surface where path accuracy is more important than generation speed.

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
