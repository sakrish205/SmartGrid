<p align="center">
  <img src="app/ui/icons/logo.png" height="140">
</p>

# SmartGrid — Robotic Spray Toolpath Planner

**Computational Manufacturing · Robotic Process Planning · 3D Geometry**

A desktop application that converts any 3D mesh (STL / OBJ / STEP) into a robot-ready spray-paint toolpath — entirely offline, no cloud dependency. Built to demonstrate the full engineering stack: geometric algorithms, real-time 3D visualization, collision detection, and multi-format robot OLP export.

SmartGrid solves the **manufacturing process-planning problem** of generating systematic, coverage-guaranteed spray trajectories over arbitrary 3D surfaces — from flat sheet metal to complex compound curves — with four path strategies, per-waypoint dynamic speed control, and automatic standoff-aware collision detection.

[![Python](https://img.shields.io/badge/Python-3.12-blue)](https://www.python.org/)
[![PySide6](https://img.shields.io/badge/GUI-PySide6-green)](https://pypi.org/project/PySide6/)
[![Version](https://img.shields.io/badge/version-1.4-informational)]()
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

---

## Recent Improvements (v1.4)

- **Full surface coverage** — all three surface-following modes (Conform, Mesh Surface, Adaptive) now use a forward-hemisphere face filter (`face_normals · mean_n ≥ 0`) instead of the single-assignment region classifier. Boundary and transition faces that the classifier assigns to an adjacent region are included, eliminating missing coverage at region edges and on top surfaces.
- **Adaptive grid intersection waypoints** — every H×V grid intersection point is preserved as a mandatory anchor in the exported path. Waypoint resampling operates per-segment (`_resample_anchored`) so intersections are never replaced by evenly-spaced interpolations.
- **Always-visible grid intersection dots** — the Adaptive Face Grid mode always renders the H×V intersection points as yellow dots in the 3D viewer (independent of the Waypoints toggle), alongside the red curved surface grid.
- **Background path generation** — all four path modes run in a background `QThread` worker; the UI remains responsive during generation and shows a status-bar progress message.
- **Correct arrow orientation** — travel-direction chevron marks use `normalize(seg_dir × face_normal)` as the perpendicular, so arrows lie in the face plane and are visible from the spray direction on every face.
- **Standoff=0 collision fix** — hard collision detection (`mesh.contains`) is skipped when standoff is 0; surface-touching paths no longer produce false red highlights.

---

## Installation

### Requirements

- Python 3.12

Install dependencies:

```bash
pip install -r requirements.txt
```

Or individually:

```bash
pip install PySide6 pyvista pyvistaqt "trimesh[easy]" numpy scipy "gmsh>=4.11.0"
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
3D Mesh (STL / OBJ / STEP)
   ↓
Up-Axis Selection
   ↓
Region Selection
   ↓
Spray Parameters
   ↓
Surface-Based Path Generation
   ↓
Boustrophedon Coverage
   ↓
Standoff Control
   ↓
Collision Detection
   ↓
3D Validation
   ↓
JSON / CSV / OLP / G-code Export
```

---

## Workflow

1. **Open mesh** — File › Open or drag an STL / OBJ / STEP file onto the window
2. **Set up-axis** — select which world axis is vertical (X / Y / Z)
3. **Region selection** — toggle TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT or click the bounding box in the viewport
4. **Set parameters** — pitch, unit, standoff, spray speed, sweep direction
5. **Preview** — enable Grid to verify pass spacing before generating
6. **Generate Path** — runs in a background thread; collisions highlighted automatically
7. **Export** — JSON, CSV, OLP (RoboDK / Visual Components / DELMIA APT), or G-code

---

## Region Selection & Multi-Face

### How region selection works

When a mesh is loaded SmartGrid's face classifier (`app/mesh/regions.py`) assigns every triangle to the region whose axis-aligned face normal it most closely matches — TOP, BOTTOM, FRONT, REAR, LEFT, or RIGHT — based on the user-selected up-axis. This classification happens once at load time and is cached in `MeshData`.

Selecting a region adds its label to `_selected_regions` (a `set[str]`). Deselecting removes it. **Multiple regions can be active at the same time.**

There are two ways to select:

| Interaction | What it does |
|---|---|
| Click a bounding-box face in the 3D viewport | Toggles that region on / off |
| Click a ribbon checkbox (TOP / BOTTOM / etc.) | Same toggle, driven from the sidebar |
| **All** button | Adds all six regions |
| **None** button | Clears all regions |

### Multi-region path generation

When **Generate Path** is pressed, SmartGrid iterates over every entry in `_selected_regions` and generates one `PaintRoute` per region:

```python
# simplified from main_window.py
for region_id in sorted(self._selected_regions):
    faces = self._model.get_region_faces(region_id)   # trimesh face indices for this region
    if len(faces) > 0:
        route = generate_route(mesh_data, region_id, faces, spray_mm)
        routes.append(route)
```

`get_region_faces(region_id)` returns the precomputed `np.ndarray` of face indices for that region. Each region's route is generated independently using its own face set — slice planes are computed from that region's bounding box, not the whole mesh.

All routes are collected into `_current_routes`, passed to `viewer.show_route()` as a single list, and exported together. The export files (JSON / CSV / OLP / G-code) include a `region` column / field so each waypoint remains traceable to its source region.

**Example — bonnet + two side rails:**

```
Selected: TOP + LEFT + RIGHT
→ generates 3 PaintRoutes, each path contained within its own region's faces
→ viewer shows all 3 simultaneously (blue/orange passes, pink connectors)
→ one JSON export: route[0]=TOP, route[1]=LEFT, route[2]=RIGHT
```

### Face classification algorithm

The classifier in `app/mesh/regions.py` assigns each triangle to a region by comparing its outward normal against the six canonical axis directions:

```python
# simplified from regions.py
region_normals = {
    'TOP':    +up,   'BOTTOM': -up,
    'FRONT':  +fwd,  'REAR':   -fwd,
    'RIGHT':  +right,'LEFT':   -right,
}
for face_idx, normal in enumerate(mesh.face_normals):
    best = max(region_normals, key=lambda r: np.dot(normal, region_normals[r]))
    region_faces[best].append(face_idx)
```

Triangles on sharp edges or 45° blends are assigned to whichever canonical direction wins the dot-product comparison. For complex curved surfaces the assignment is approximate — compound-curve regions like blended edges or fenders may benefit from Mesh Surface or Face Grid — Conform mode, which respect the actual mesh geometry regardless of how regions are classified.

### Select Faces mode

The **Select Faces** toolbar button suspends camera rotation (replaces the VTK interactor style with `vtkInteractorStyleUser`) so that a click lands cleanly on the bounding box without spinning the view. The underlying mechanism — clicking a bbox solid actor to identify the region — is the same as in default navigation mode; Select Faces just makes the targeting easier on dense meshes.

---

## Path Generation

SmartGrid provides four toolpath generation modes for different workpiece geometries. Each mode makes a different trade-off between speed, accuracy, and surface fidelity.

### Mode comparison

| Mode | Mesh query | Path accuracy | Speed | Best for |
|---|---|---|---|---|
| **Boundary Box** | None — bbox only | Flat plane (floats on curves) | Instant | Flat panels, sheet metal |
| **Face Grid — Adaptive** | Normal stats only | Tilted plane, per-row depth tracking | Fast | Bonnets, doors, gently curved panels |
| **Face Grid — Conform** | Trimesh plane intersection | On-surface, tilted cutting planes | Medium | Blended edges, compound curves |
| **Mesh Surface** | Trimesh plane intersection | On-surface, axis-aligned cutting planes | Slower | Complex surfaces, parts with holes |

All four modes produce the same output format (`PaintRoute` / `PaintPass`), support multi-region selection, and export identically.

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
3. The extent of the spray region is computed from all forward-hemisphere faces — `fwd_face_ids = where(face_normals @ mean_n >= 0)` — not just the classifier-assigned subset, so boundary and transition faces contribute to the bounding extent.
4. All forward-hemisphere vertices are projected onto the basis: `pass_proj = v · pass_vec`, `step_proj = v · step_vec`, `depth_proj = v · mean_n`.
5. For each step position, a band of vertices within ±`pitch × 0.65` is collected. The outermost depth in that band (`max(depth_proj)`) becomes the spray plane depth for that row — the pass elevation tracks the actual surface peak.
6. World-space endpoints are reconstructed: `P = row_depth × mean_n + p_min × pass_vec + step_pos × step_vec`.
7. Each pass is a polyline through the H×V grid intersection points `grid_pts[i, j]` sampled directly on the 3D surface. Uniform waypoint resampling uses `_resample_anchored`, which resamples each segment independently so every grid intersection is preserved as a mandatory anchor in the exported path.
8. Standoff is added along `mean_n` at point construction time.

```python
# core: face_grid_generator.py — generate_adaptive_grid_route
mean_n   = normalize(mesh.face_normals[forward_faces].mean(axis=0))
pass_vec = normalize(cross(mean_n, up))        # left-right sweep axis
step_vec = normalize(cross(pass_vec, mean_n))  # step axis

fwd_face_ids = np.where(mesh.face_normals @ mean_n >= 0.0)[0]  # full hemisphere extent
fwd_verts    = mesh.vertices[mesh.faces[fwd_face_ids].ravel()]

for step_pos in step_positions:
    band = fwd_verts[|step_proj - step_pos| <= pitch * 0.65]
    row_depth = (band @ mean_n).max() + standoff_mm
    # grid_pts[i, j] = surface depth sample at each (row, col) intersection
    pass_pts = grid_pts[row_idx]                     # H×V intersection waypoints
    pass_pts = _resample_anchored(pass_pts, spacing) # resample per-segment, pin intersections
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
4. `trimesh.intersections.mesh_plane` intersects the full mesh against each tilted cutting plane. Returned segments are filtered using a forward-hemisphere mask — `face_normals[seg_face_ids] @ mean_n >= 0.0` — rather than the classifier-assigned face set. This ensures boundary and transition faces (which the classifier assigns to adjacent regions) are included, giving complete surface coverage at region edges.
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
    # Forward-hemisphere filter — includes boundary faces the classifier excludes
    mask = mesh.face_normals[face_ids] @ mean_n >= 0.0
    segments = segments[mask]
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
4. An outward-face filter is applied via `_outward_sign(region_id, up_axis)`: for regions with a known outward axis (FRONT/REAR/LEFT/RIGHT/TOP/BOTTOM), a face-normal dot-product threshold replaces the classifier membership test — segments whose source face normal does not point outward are dropped. This forward-hemisphere approach includes boundary and transition faces that the classifier assigns to adjacent regions, giving complete coverage. A subsequent largest-gap cluster analysis on segment heights drops any internal ribs or supports that survived the normal filter.
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
    # Primary filter: forward-hemisphere normal check (slicer.py — slice_region)
    # _outward_sign() returns the axis and sign for the selected region;
    # segments are kept when face_normals[face_id, axis] * sign > -0.25
    segments = slice_region(mesh, region_faces, plane_normal, origin, region_id, up_axis)
    # slice_region also runs a largest-gap cluster check to drop internal ribs
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

## Collision Detection

After every path generation SmartGrid automatically checks all spray passes against the loaded mesh and flags any passes that would cause interference.

**Two severity levels:**

| Level | Condition | Viewer colour |
|---|---|---|
| **Hard collision** | Pass waypoint is inside the mesh volume | Red `#FF1744` |
| **Near miss** | Closest surface distance < standoff × 0.5 | Orange `#FF9100` |

The hard collision check (`mesh.contains`) is also skipped when standoff is 0 — surface-touching paths produce false positives from the ray-cast boundary case. The near-miss check is skipped for the same reason.

**Algorithm — two-tier check per pass:**

```python
# core: collision.py — detect_collisions
for route in routes:
    for p in route.passes:
        pts = p.points                          # all waypoints in this pass

        # Tier 1 — hard collision: any point inside the mesh volume
        inside = mesh.contains(pts)             # ray-cast per point
        if inside.any():
            flagged[p.id] = 'collision'
            continue                            # no need to check tier 2

        # Tier 2 — near miss: closest surface distance below danger threshold
        if standoff_mm > 0:
            _, dists, _ = closest_point(mesh, pts)
            if (dists < standoff_mm * 0.5).any():
                flagged[p.id] = 'near_miss'
```

Flagged passes are highlighted in the 3D viewer immediately after generation. The status bar reports the count:

```
Path generation complete — 12 passes, 11 connections.  ⚠ 1 collision(s), 2 near-miss(es) — shown red/orange  |  Suggested standoff: 25 mm
```

**Implementation:** uses `trimesh.proximity.closest_point` and `mesh.contains` — no additional dependency.

---

## STEP Import

SmartGrid supports CAD files in STEP (`.step`, `.stp`) format in addition to STL and OBJ. STEP files are tessellated using the [gmsh](https://gmsh.info/) CAD kernel at load time.

```bash
# gmsh is included in requirements.txt
pip install "gmsh>=4.11.0"
```

The tessellation uses gmsh's OpenCASCADE kernel (`occ.importShapes`) and produces a watertight triangle mesh that is then processed identically to an imported STL. No CAD geometry is retained after load — all downstream path generation works on the tessellated mesh.

**Supported:** STEP AP203 / AP214 (single-body and multi-body assemblies). IGES files can also be tessellated using the same code path by changing the extension.

---

## Spray Speed

Speed is configured in the **Export OLP** dialog, not in the Parameters panel, with two modes:

| Mode | Behaviour |
|---|---|
| **Auto** | Per-waypoint speed derived from local path curvature — straight segments run at `max_mmpm` (2 000 mm/min), tight turns slow to `min_mmpm` (300 mm/min); values tuned via `_SPEED_CURVE` in `main_window.py` |
| **Custom** | Fixed speed for every waypoint; user-entered value |

Speed is written into every OLP export format:

| Format | Usage |
|---|---|
| DELMIA APT | `FEDRAT/value,MMPM` written only when speed changes (auto), or once per pass block (custom) |
| G-code | `G1 F{speed} X Y Z` on the first or any changed-speed waypoint; bare `G1 X Y Z` otherwise |
| RoboDK CSV | 7th column `speed_mmpm` on every spray pass row |
| Visual Components CSV | `speed_mmpm` column on every spray pass row; blank on connectors |
| JSON / neutral CSV | Stored in `GenerationParams.paint_speed_mmpm` |

Connector (air) moves are always written as rapid/travel — speed control applies to spray passes only.

---

## Controls

| Control | Function |
|---|---|
| **TOP / BOTTOM / FRONT / REAR / LEFT / RIGHT** | Toggle bounding-box faces as spray regions |
| **All / None** | Select or deselect all regions at once |
| **Select Faces** | 3D pick mode for selecting regions |
| **Unit** | mm / cm / m / in / ft |
| **Spray Width (Pitch)** | Centre-to-centre distance between adjacent passes |
| **Standoff** | Spray-gun offset from the mesh surface (mm) |
| **Direction** | H (horizontal passes, vertical step) / V (vertical passes, horizontal step) |
| **Sweep** | CW / CCW starting direction |
| **Waypoints** | Enable uniform resampling |
| **Interval** | Waypoint resampling spacing |
| **Grid** | Display pitch-division lines |
| **Arrows** | Show travel direction as chevron tick marks |
| **Generate Path** | Generate the selected toolpath |
| **Clear Path** | Remove generated paths |
| **View Settings** | Configure 3D display and mesh rendering colours |
| **Export JSON / CSV / OLP** | Export the generated trajectory |

---

## Path Processing

Generated geometry is processed into an ordered trajectory using:

- Segment stitching
- RDP simplification
- Corner-fragment filtering
- Boustrophedon ordering
- Pass-to-pass connectors
- Optional uniform waypoint resampling

The **Generate Path** operation runs in a background thread so the UI remains responsive during computation.

---

## 3D Visualization

The integrated PyVista/VTK viewer provides visual validation of:

- Workpiece mesh (light theme, Office-style UI)
- Selected spray regions (highlighted bounding-box faces)
- Pitch grid
- Spray passes — forward (blue) / reverse (orange)
- Hard collisions (red) / near-misses (orange)
- Travel direction (chevron tick marks, perpendicular to the face normal for correct orientation on all faces)
- Connectors (pink)
- Waypoints (gold dots)
- Adaptive Face Grid: red curved surface grid and always-visible yellow grid intersection dots (mandatory export anchors)
- Mesh rendering and display settings

This allows the generated trajectory to be inspected before export.

---

## Robot Path Export

### JSON

Structured toolpath containing:

- Metadata and summary (source file, mode, regions, speed, standoff, generated-at timestamp)
- Routes and regions
- Spray passes with direction flag
- Pass lengths
- 3D waypoints
- Connections / transit moves
- Air-move information

### CSV

One row per trajectory point:

| Column | Content |
|---|---|
| `seq_id` | Global execution order |
| `Trigger` | `ON` = spray gun firing, `OFF` = air travel |
| `segment_type` | `pass` or `connection` |
| `pass_id` | Pass or connection number |
| `pt_idx` | Point index within segment |
| `X`, `Y`, `Z` | TCP position in mm |
| `NX`, `NY`, `NZ` | Outward surface normal unit vector |
| `length_mm` | Segment total length (first point only) |
| `region` | Face region label |
| `is_forward` | `True` / `False` for passes; blank for connections |

### OLP Formats

Four robot-OLP formats are exported via **Export OLP**:

#### RoboDK
7-column CSV (no header row): `X,Y,Z,NX,NY,NZ,speed_mmpm` — spray passes only. Drag-drop into RoboDK via *Utilities › Import Curve*. `NX/NY/NZ` = outward surface normal (RoboDK uses it as the curve approach direction).

#### Visual Components
CSV with header `seq_id,X,Y,Z,NX,NY,NZ,Trigger,speed_mmpm`. Spray passes have `Trigger=ON` with normals and speed; connector moves have `Trigger=OFF` with blank normals and blank speed.

#### DELMIA APT
APT text file. Spray passes use `GOTO/X,Y,Z,I,J,K` where `I,J,K` = tool Z axis = `-spray_normal` (points into the surface). Connector moves use `RAPID/X,Y,Z`. `FEDRAT/value,MMPM` is written only when speed changes (auto mode) or once per pass (custom mode); spray gun written as `SPINDL/ON` and `SPINDL/OFF`.

#### G-code (CNC / Robot)
Standard G-code `.nc` file compatible with CNC and open robot controllers:

```gcode
%
O0001 (SMARTGRID TOOLPATH)
G21 G90 G94
( Region: TOP )
( Pass 0 )
M8                         ; spray gun ON
G1 F1000.0
G1 X10.2340 Y20.4560 Z5.6780
...
M9                         ; spray gun OFF
( Connection: ... )
G0 X12.0000 Y25.0000 Z6.0000
M30
%
```

| Statement | Meaning |
|---|---|
| `G21` | mm mode |
| `G90` | absolute positioning |
| `G94` | feed per minute |
| `M8` / `M9` | spray gun on / off (standard coolant codes, repurposed) |
| `G1 F{speed}` | spray pass at configured speed |
| `G0` | rapid connector move |
| `M30` | end of program |

Tool orientation (A, B, C axes) is not written — these are controller-specific and require a post-processor for each robot platform.

All internal distances are maintained in millimetres; display/input unit conversion does not change the physical distance.

> **Integration note:** SmartGrid is a **toolpath planning system**, not a complete robot controller. JSON/CSV/OLP/G-code output provides an intermediate trajectory representation for downstream robot-controller integration.

---

## File Formats

| Format | Import | Export |
|---|---:|---:|
| STL (binary or ASCII) | ✓ | — |
| OBJ (single- or multi-body) | ✓ | — |
| STEP (AP203 / AP214) | ✓ | — |
| JSON (toolpath) | — | ✓ |
| CSV (toolpath, neutral) | — | ✓ |
| CSV (RoboDK curve import) | — | ✓ |
| CSV (Visual Components) | — | ✓ |
| APT (DELMIA) | — | ✓ |
| G-code `.nc` (CNC / Robot) | — | ✓ |

Multi-body OBJ files are merged at load via `trimesh.load(force='mesh')`. STEP files are tessellated via gmsh at load time.

---

## Technology Stack

### Engineering

- 3D CAD / mesh geometry
- Surface-normal analysis
- Robotic process planning
- Spray-path planning
- Surface coverage planning
- Standoff control
- Collision detection
- Manufacturing automation

### Software

- Python 3.12
- PySide6 (Qt6)
- PyVista / VTK
- Trimesh
- NumPy
- SciPy
- gmsh (STEP/IGES CAD tessellation)

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
- **Controlled pitch, standoff, and spray speed**
- **Collision detection** — hard collisions and near-misses flagged automatically after generation
- **STEP / CAD import** — direct CAD file support via gmsh tessellation
- **G-code export** — paths usable on any CNC or open robot controller
- **3D path visualization and validation**
- **JSON / CSV / OLP / G-code robot-path export**
- **Modular architecture**

---

## Limitations

SmartGrid currently focuses on **geometric toolpath generation**. It does not itself provide:

- Robot-specific inverse kinematics
- Robot-controller post-processing (post-processors for specific platforms)
- Dynamic robot reachability analysis
- Spray deposition physics
- Paint-thickness prediction
- Paint-flow modelling
- Complete robot-cell simulation

The Adaptive Face Grid method can float on surfaces with significant curvature along the sweep direction because individual passes are straight.

Collision detection uses per-point ray casting (`mesh.contains`) — on very dense meshes with many flagged passes this can be slow. A signed-distance field approach would improve throughput for high-polygon models.

G-code tool orientation (A, B, C rotary axes) is not written — a platform-specific post-processor is required to map spray-normal vectors to robot joint angles.

---

## Future Scope

- Robot-specific post-processors for industrial robot platforms (KUKA KRL, ABB RAPID, Fanuc TP)
- TCP orientation and inverse-kinematics integration
- Adaptive spray pitch and speed optimisation
- Spray deposition / coating-thickness modelling
- Robot simulation and digital-twin integration
- CAD/CAM and manufacturing-cell integration
- Geodesic connectors (shortest path along mesh surface instead of straight air moves)

---

## Project Structure

```text
SmartGrid/
├── main.py
├── requirements.txt
├── app/
│   ├── main_window.py
│   ├── ui/
│   │   ├── ribbon.py
│   │   ├── viewer.py
│   │   └── view_settings_dialog.py
│   ├── mesh/
│   │   ├── loader.py          — STL / OBJ / STEP load + repair
│   │   ├── preprocessor.py    — normals, centroids, bbox cache
│   │   └── regions.py         — face classifier (TOP/BOTTOM/etc.)
│   ├── path/
│   │   ├── path_model.py      — PaintPass, Connection, PaintRoute, GenerationParams
│   │   ├── bbox_generator.py
│   │   ├── face_grid_generator.py
│   │   ├── generator.py       — Mesh Surface orchestration
│   │   ├── slicer.py
│   │   ├── stitcher.py
│   │   ├── connector.py
│   │   ├── resampler.py
│   │   └── collision.py       — detect_collisions() — hard collision + near-miss
│   └── export/
│       ├── json_export.py
│       ├── csv_export.py
│       └── olp_export.py      — RoboDK, Visual Components, DELMIA APT, G-code
└── models/
    └── mesh_model.py
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
COLLISION CHECKING
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
