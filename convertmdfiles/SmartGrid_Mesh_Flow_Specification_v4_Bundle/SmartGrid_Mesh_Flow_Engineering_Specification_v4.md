SMARTGRID

# Mesh Flow

Mode 4 --- Detailed Engineering Specification

Reference-derived rules • Vector mathematics • Confidence assessment •
Implementation roadmap

| Document status \| Scope \| Code changes \|

| --- \| --- \| --- \|

| Proposed specification; not yet validated against a complete
  mesh/reference pair \| Mode 4 path planning and rename strategy \|
  None. This document does not modify the SmartGrid repository. \|

Name decision: "Mesh Flow" is intended as both the user-facing Mode 4
name and the proposed canonical mode identity.

# 1. Executive summary

Mesh Flow is the proposed replacement name for SmartGrid Mode 4,
previously labelled "Mesh Surface." The name change should not be
limited to the visible menu label: when implementation is approved, the
mode's canonical identifier, dispatch/configuration mappings, exports or
metadata that encode the mode, documentation, and tests should be
reviewed together. The actual code is not changed as part of this
specification.

The engineering objective is to generate smooth, coverage-oriented robot
paths over a 3D mesh. The trajectory should follow meaningful body
curvature while avoiding small triangle-induced wiggles; preserve a
structured raster where practical; reverse direction between successive
lanes when appropriate; compute tool orientation separately from travel
direction; and distinguish painting moves from approach, retract,
transition, and repositioning moves.

The OLP review notes provide evidence of region-specific paths,
alternating raster passes, explicit program execution order, tool
orientation, and process-state changes. They do not prove that one
global algorithm or one fixed spacing works for all surfaces. The
equations below are candidate engineering methods, not claims that every
formula was present in the engineer's software.

| Confidence label \| Meaning in this document \|

| --- \| --- \|

| High \| Directly stated in the supplied OLP analysis or a standard
  geometric identity; still subject to implementation/unit checks. \|

| Medium \| Reasonable engineering inference from observed reference
  patterns; needs comparison with additional reference paths. \|

| Low / unvalidated \| Proposed design choice or parameter whose
  correctness depends on the actual mesh, process, robot, and validation
  data. \|

# 2. Mode name and rename scope

Selected name: Mesh Flow. The requested change is a rename proposal, not
merely a display-text replacement.

| Layer to review when implementing \| Recommended treatment \| Risk if
  overlooked \|

| --- \| --- \| --- \|

| UI display label \| Show "Mesh Flow" wherever Mode 4 is named to
  users. \| UI may show mixed old/new names. \|

| Canonical mode identifier \| Choose one stable identifier,
  e.g. `mesh_flow`, after searching current naming conventions. \|
  Dispatch logic may still select the old mode. \|

| Mode registry / factory / dispatch \| Update all registrations and
  selection mappings consistently. \| Mode may fail to instantiate or
  call the wrong generator. \|

| Configuration and saved projects \| Add a backward-compatible alias or
  migration if the old name/ID is persisted. \| Existing
  projects/settings may stop loading. \|

| Exports and metadata \| Decide whether generated output records the
  algorithm name; keep file-format compatibility. \| Downstream tools or
  scripts may depend on the old label. \|

| Documentation, help, logs \| Update terminology and distinguish legacy
  references where needed. \| Troubleshooting becomes ambiguous. \|

| Tests and fixtures \| Update expected labels and add a legacy-ID
  compatibility test if relevant. \| Rename appears successful but
  breaks regression tests. \|

Do not blindly rename every occurrence of the words "mesh" or "surface."
Search for the specific Mode 4 enum/key, UI label, factory mapping,
settings schema, serialized values, tests, and documentation. Preserve
unrelated uses of "mesh surface" as a geometry description.

# 3. What the OLP references support

| Reference \| Source-reported observations \| Engineering implication
  \| Confidence \|

| --- \| --- \| --- \| --- \|

| ABB `Exercise_2.mod` --- Mazda MX-5 \| 60 targets in two procedures:
  `mvHood()` and `mvSideA()`; hood has 24 target calls; alternating
  passes and lateral X changes around 115--130 mm; side has distinct
  regions and large transitions; quaternion and external-axis data are
  present. \| Use region-specific scan frames; parse procedure execution
  order; preserve external axes and transitions where the output format
  supports them. \| High for supplied analysis; medium for
  generalization. \|

| FANUC `S1_100.LS` --- Wrangler cab \| 63 positions; linear and joint
  moves; gun-off and subsequent movements; W/P/R orientation; variable
  speeds; numeric point order differs from execution order. \| Execution
  order and process state must be parsed explicitly. Do not sort points
  by ID and assume that is the trajectory. \| High for supplied
  analysis. \|

| DÜRR `Aleta_Ptras.tid` + `.tip` \| 175 point records; complex early
  region; region around P31--P78 has X lanes near 1630, 1780, 1930,
  2080, 2230, 2380, 2530 mm; approximate 150 mm lane pitch; Z spans
  roughly −30 to 870--940 mm; direction alternates; Y varies along a
  lane. \| Strongest evidence for a local serpentine raster whose 3D
  points adapt to the body. 150 mm is specific to that region, not a
  default for every part. \| High for the reported region; medium for
  generalization. \|

| DÜRR `xpos_test.tid` + `.tip` \| 92 point records but companion
  program executes five named targets. \| Only executed targets define
  this test trajectory; unused definitions must not be counted as a
  complete path. \| High for supplied analysis. \|

| DÜRR `MAIN_MAIN_R11.tip` \| Orchestrates setup, tool/object selection,
  conveyor tracking, main path call, return and cleanup. \| Treat
  orchestration separately from geometric path generation. \| High for
  supplied analysis. \|

| Reference image \| Separate hood, side, and boundary/transition path
  behaviors are described. \| Region segmentation is a stronger
  candidate than one global raster across the whole part. \| Medium:
  visual interpretation, not a numeric path proof. \|

Evidence limitation: this document uses the supplied individual-file
analysis text as the reference record. It does not independently reparse
every robot file or execute the generator. Confidence values therefore
describe support in the supplied analysis and engineering
plausibility---not a claim of 100% verification.

# 4. Design rules in simple and technical terms

## R1 --- Segment the part into regions

Simple explanation: Simple: do not treat the whole vehicle body as one
flat scanning area.

Technical definition: Technical: partition the target surface into
connected regions with compatible local scan direction, curvature,
accessibility, and process intent. Region boundaries may come from user
selection, normals/curvature thresholds, mesh connectivity, or
reference-defined zones.

Confidence: Confidence: High that reference programs are
region-specific; Medium for any automatic segmentation method.

Implementation: Implementation: first support explicit/user-selected
regions or conservative geometric segmentation. Store each region's
frame, boundary, pass direction, pitch, and process state. Avoid
splitting solely on noisy triangle normals.

Validation: Validation: compare region boundaries and transitions with
reference programs; test connected and disconnected surfaces.

## R2 --- Recover actual execution order

Simple explanation: Simple: follow the order the robot program executes,
not the order points appear in a file.

Technical definition: Technical: parse motion instructions and resolve
each referenced target to its position, orientation, external axes,
speed, and process-state context. Build an ordered motion graph/list.

Confidence: Confidence: High; the supplied FANUC and DÜRR examples
explicitly demonstrate this distinction.

Implementation: Implementation: separate target-definition parsing from
instruction parsing. Represent each move as a typed segment: paint,
approach, retract, transit, joint move, or unknown.

Validation: Validation: reproduce known program waypoint sequences and
compare the parsed ordered target list against the source program.

## R3 --- Estimate a primary pass direction per region

Simple explanation: Simple: decide the main direction for the long
passes locally.

Technical definition: Technical: estimate a unit vector from reference
pass tangents, user direction, region principal axis, or a projected
preferred direction. Do not assume global X/Y/Z axes are correct for
every part.

Confidence: Confidence: High that references use different directions by
region; Medium for the best automatic estimator.

Implementation: Implementation: prefer reference/user direction if
supplied; otherwise estimate a stable principal direction from region
geometry and evaluate candidate directions against coverage and
collision constraints.

Validation: Validation: compare estimated direction with reference
passes in a common coordinate frame.

## R4 --- Define lateral pass spacing

Simple explanation: Simple: shift to the next lane by a controlled
distance.

Technical definition: Technical: compute the perpendicular separation
between neighboring pass centerlines in the local scan frame. If coating
width and overlap are known, pitch may be derived from them; otherwise
treat pitch as an explicit parameter or estimate it from the reference.

Confidence: Confidence: High for the local approximately 150 mm DÜRR
lane spacing reported; Low for any universal pitch value.

Implementation: Implementation: distinguish `reference_pitch_mm`,
`effective_spray_width_mm`, and `overlap_ratio`. Do not assume the 150
mm reference spacing is valid for other guns, parts, or process
settings.

Validation: Validation: measure lane spacing in the source coordinate
frame and assess coverage gaps/overlap.

## R5 --- Alternate direction between lanes where appropriate

Simple explanation: Simple: travel down one lane, shift sideways, then
travel back along the next lane.

Technical definition: Technical: use a serpentine order to reduce
unnecessary return travel. For lane index k, the nominal direction sign
is (-1)\^k, while the actual lane curve remains surface-adaptive.

Confidence: Confidence: High for the ABB hood and DÜRR raster regions;
Medium as a global default.

Implementation: Implementation: apply alternating direction only within
a region where lanes form a coherent raster. Insert explicit transition
moves between regions; do not reverse direction blindly across
disconnected patches.

Validation: Validation: compare travel length, transition count, and
exact lane order against references.

## R6 --- Follow meaningful 3D curvature, not triangle noise

Simple explanation: Simple: the path should bend with the real body
shape, but not wiggle because the mesh triangles are uneven.

Technical definition: Technical: create each lane as a 3D curve on/near
the target surface; smooth the sampled curve with constraints on surface
deviation, curvature, and process tolerances. Filter mesh noise without
erasing important edges or features.

Confidence: Confidence: Medium; supported as the intended behavior, but
the smoothing thresholds need mesh-specific validation.

Implementation: Implementation: smooth in arc-length space; preserve
region boundaries and protected features; constrain deviation from the
target surface. Avoid smoothing only by averaging points without a
surface-error bound.

Validation: Validation: measure RMS and maximum surface deviation,
curvature spikes, and reference-path deviation.

## R7 --- Calculate tool orientation independently from travel direction

Simple explanation: Simple: the robot's movement direction and the gun's
pointing direction are different things.

Technical definition: Technical: estimate a stable surface normal and
construct a tool frame using the desired pointing axis and projected
path tangent. Convert the frame using the robot/controller's actual
tool-axis convention.

Confidence: Confidence: High that reference files contain orientations;
Medium for reconstructing orientation from mesh normals alone.

Implementation: Implementation: prefer valid reference poses where
available; otherwise use area-weighted or interpolated normals, a
consistent tangent frame, and orientation continuity checks. Confirm
quaternion/WPR conventions and tool calibration.

Validation: Validation: test known poses, normal direction, quaternion
normalization, angular jumps, and robot reachability.

## R8 --- Keep painting and transition moves separate

Simple explanation: Simple: do not paint during every movement between
lanes or regions.

Technical definition: Technical: each segment must carry motion type,
paint enable state, speed, and optional brush/tool state. A connector
between passes is not automatically a painting segment.

Confidence: Confidence: High; source programs contain gun/brush state
and motion/speed instructions.

Implementation: Implementation: maintain a process-state timeline
independent of geometry. Set gun-off for approach/retract/transit unless
a process-specific rule says otherwise.

Validation: Validation: verify state transitions and ensure no
unintended paint-on connectors.

## R9 --- Preserve coordinate frames and external axes

Simple explanation: Simple: coordinates are meaningful only in the right
robot/model coordinate system.

Technical definition: Technical: transform mesh points, TCP points, and
normals consistently between mesh, work-object, robot base, and tool
frames; retain external-axis values if relevant.

Confidence: Confidence: High that ABB target definitions include
external axes and frames matter; Medium until frame metadata is checked
end to end.

Implementation: Implementation: store frame IDs and transforms
explicitly; never compare raw XYZ values across unrelated robot programs
without normalization.

Validation: Validation: round-trip known reference points through
transforms and check units/handedness.

## R10 --- Validate safety and coverage before export

Simple explanation: Simple: a visually smooth path is not necessarily
safe or complete.

Technical definition: Technical: evaluate standoff, orientation jumps,
surface error, coverage, collisions, reachability, and controller
constraints before exporting.

Confidence: Confidence: High as a safety/quality requirement; exact
thresholds are project/process dependent.

Implementation: Implementation: return validation findings with severity
and waypoint/segment IDs; fail export on hard safety violations rather
than silently fixing them.

Validation: Validation: unit tests, mesh regression suite, reference
comparisons, simulator checks, and eventual robot-cell validation.

# 5. Formula reference appendix

Notation: bold letters are 3D vectors; \|\|v\|\| is vector length; a hat
denotes a unit vector; dot and cross denote dot and cross products. All
coordinates in a calculation must use the same frame and length unit.
Formula confidence describes suitability as a candidate implementation
method, not proof that the reference engineer used that exact equation.

### F01. Vector from A to B

v = b − a

Simple meaning: The direction from point A to point B.

Technical meaning: Subtract position vectors in a shared coordinate
frame.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Used for tangents, segment lengths and direction
estimates.

### F02. Vector length

\|\|v\|\| = sqrt(vx² + vy² + vz²)

Simple meaning: How long a vector is.

Technical meaning: Euclidean norm of a 3D vector.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Use for distances; reject non-finite values.

### F03. Unit vector

v_hat = v / \|\|v\|\|, if \|\|v\|\| \> ε

Simple meaning: Turn a vector into a direction of length one.

Technical meaning: Normalize nonzero vectors with a numerical tolerance
ε.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Guard zero-length segments; use a fallback or report an
error.

### F04. Dot product

a · b = ax bx + ay by + az bz

Simple meaning: Measures directional alignment.

Technical meaning: For unit vectors, the result is cosine of their
angle.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Clamp computed cosine to \[−1, 1\] before inverse
cosine.

### F05. Cross product

a × b

Simple meaning: Creates a vector perpendicular to both inputs.

Technical meaning: Right-handed 3D cross product; order determines sign.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Check near-parallel inputs before normalizing.

### F06. Project preferred direction onto reference plane

d' = d0 − (d0 · n0) n0

Simple meaning: Remove the part of the direction pointing through the
plane.

Technical meaning: Projection onto a plane with unit normal n0.

Confidence: High for planar projection; Medium as a scan-frame choice..
Evidence / basis: Standard geometric identity or proposed engineering
metric; not evidence that the OLP engineer used this exact equation.

Implementation: Normalize n0 first; if d' is near zero, choose another
preferred direction.

### F07. Local scan frame

d_hat = normalize(d'); s_hat = normalize(n0 × d_hat)

Simple meaning: Build a forward direction and a sideways direction.

Technical meaning: Construct orthonormal directions from a preferred
tangent and normal.

Confidence: Medium --- standard construction, direction selection is
inferred.. Evidence / basis: Standard geometric identity or proposed
engineering metric; not evidence that the OLP engineer used this exact
equation.

Implementation: Check orthogonality and handedness; use local frames for
different regions.

### F08. Signed distance to scan plane

δ(x) = s_hat · (x − p0) − k Δ

Simple meaning: Check which side of a lane plane a point lies on.

Technical meaning: Signed scalar distance from x to the kth plane in a
family of parallel planes.

Confidence: Medium --- proposed slicing method.. Evidence / basis:
Standard geometric identity or proposed engineering metric; not evidence
that the OLP engineer used this exact equation.

Implementation: All terms must share units; k is lane index and Δ is
pitch.

### F09. Edge/plane intersection parameter

t = δ(a) / (δ(a) − δ(b))

Simple meaning: Find where a triangle edge crosses a plane.

Technical meaning: Linear interpolation parameter for opposite signed
distances.

Confidence: High for nonparallel crossing; standard geometry.. Evidence
/ basis: Standard geometric identity or proposed engineering metric; not
evidence that the OLP engineer used this exact equation.

Implementation: Handle denominator near zero and points exactly on the
plane.

### F10. Intersection point

q = a + t (b − a)

Simple meaning: Locate the crossing point on an edge.

Technical meaning: Interpolate between edge endpoints with t in \[0,1\].

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Deduplicate near-identical points at shared triangle
edges.

### F11. Scan-plane equation

s_hat · (x − p0) = k Δ

Simple meaning: Define one lane's slicing plane.

Technical meaning: A family of planes indexed by integer k.

Confidence: Medium --- one candidate strategy, not required by
references.. Evidence / basis: Standard geometric identity or proposed
engineering metric; not evidence that the OLP engineer used this exact
equation.

Implementation: This plane family is suitable only if a local scan frame
is valid for the region.

### F12. Number of planes

N = floor((s_max − s_min)/Δ) + 1

Simple meaning: Estimate how many lanes fit across a region.

Technical meaning: Count evenly spaced lane coordinates over the chosen
interval.

Confidence: Medium --- depends on endpoint policy.. Evidence / basis:
Standard geometric identity or proposed engineering metric; not evidence
that the OLP engineer used this exact equation.

Implementation: Handle reversed bounds and decide whether to add a final
boundary lane.

### F13. Segment direction

d_i = normalize(q\_(i+1) − q_i)

Simple meaning: Direction of one small path segment.

Technical meaning: Finite-difference tangent estimate.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Skip duplicate points; use a wider neighborhood if
noisy.

### F14. Point-to-line perpendicular deviation

e_i = \|\|(q_i − a) − ((q_i − a) · u)u\|\|

Simple meaning: How far a point sits sideways from a straight line.

Technical meaning: Distance from q_i to the infinite line through a with
unit direction u.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Use only for assessing local straightness; do not force
curved regions flat.

### F15. Maximum line deviation

E_max = max_i e_i

Simple meaning: Largest sideways bend away from a reference line.

Technical meaning: Worst-case deviation metric for a candidate pass.

Confidence: High --- standard metric.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Set tolerance from process requirements, not arbitrary
convenience.

### F16. Mean squared line deviation

E_line = (1/n) Σ e_i²

Simple meaning: Average squared departure from a line.

Technical meaning: Least-squares straightness objective.

Confidence: High --- standard metric.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Combine with max error so one large local defect is not
hidden.

### F17. Second difference

Δ²q_i = q\_(i+1) − 2q_i + q\_(i−1)

Simple meaning: Shows how sharply the point sequence changes direction.

Technical meaning: Discrete second derivative proxy for equally spaced
samples.

Confidence: High as a discrete operator; Medium as curvature proxy..
Evidence / basis: Standard geometric identity or proposed engineering
metric; not evidence that the OLP engineer used this exact equation.

Implementation: Resample by arc length before interpreting magnitude.

### F18. Smoothness penalty

E_smooth = Σ \|\|Δ²q_i\|\|²

Simple meaning: Penalize abrupt wiggles.

Technical meaning: Squared second-difference regularizer.

Confidence: Medium --- useful objective, weight needs tuning.. Evidence
/ basis: Standard geometric identity or proposed engineering metric; not
evidence that the OLP engineer used this exact equation.

Implementation: Constrain surface deviation while smoothing.

### F19. Approximate curvature

κ_i ≈ \|\|Δ²q_i\|\| / δs²

Simple meaning: Estimate how sharply a path bends.

Technical meaning: Second-difference curvature proxy for approximately
equal arc-length spacing δs.

Confidence: Medium --- approximation.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: For nonuniform spacing, use a proper nonuniform
derivative or resample.

### F20. Reference-path deviation

E_ref = Σ \|\|q_i − q_i\*\|\|²

Simple meaning: Measure how far the generated path differs from an
engineer's path.

Technical meaning: Pointwise squared error after corresponding paths are
aligned and resampled.

Confidence: Medium --- requires correspondence.. Evidence / basis:
Standard geometric identity or proposed engineering metric; not evidence
that the OLP engineer used this exact equation.

Implementation: Do not compare raw point index i if sample counts
differ.

### F21. Combined objective

J = λr E_ref + λd E_dir + λc E_smooth

Simple meaning: Balance matching, direction and smoothness.

Technical meaning: Weighted objective with nonnegative terms and tunable
weights.

Confidence: Low until weights and correspondence are calibrated..
Evidence / basis: Standard geometric identity or proposed engineering
metric; not evidence that the OLP engineer used this exact equation.

Implementation: Normalize terms or use physically meaningful weights;
retain hard constraints separately.

Caution: Treat as a candidate equation; verify assumptions and calibrate
parameters against actual reference pairs.

### F22. Surface conformity error

E_surface = Σ dist(q_i, M)²

Simple meaning: Measure whether path points remain close to the mesh.

Technical meaning: Squared point-to-mesh distance for mesh M.

Confidence: High as a metric; Medium for chosen distance
implementation.. Evidence / basis: Standard geometric identity or
proposed engineering metric; not evidence that the OLP engineer used
this exact equation.

Implementation: Use closest point on triangles, not nearest vertex
alone.

### F23. Full candidate objective

J = λs E_surface + λd E_dir + λc E_smooth + λw E_waviness

Simple meaning: Balance staying on the surface, direction, smoothness
and mesh-noise resistance.

Technical meaning: Multi-objective path score; terms need compatible
scaling.

Confidence: Low until calibrated on reference pairs.. Evidence / basis:
Standard geometric identity or proposed engineering metric; not evidence
that the OLP engineer used this exact equation.

Implementation: Use bounded optimization and preserve protected geometry
features.

Caution: Treat as a candidate equation; verify assumptions and calibrate
parameters against actual reference pairs.

### F24. Triangle normal

n = normalize((b − a) × (c − a))

Simple meaning: Find the direction perpendicular to a triangle.

Technical meaning: Face normal from vertex winding.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Incorrect triangle winding reverses the normal; verify
orientation.

### F25. Interpolated vertex normal

n = normalize(α n1 + β n2 + γ n3)

Simple meaning: Blend nearby normals for a smoother direction.

Technical meaning: Barycentric interpolation inside a triangle with
α+β+γ=1.

Confidence: Medium --- common method, depends on mesh quality.. Evidence
/ basis: Standard geometric identity or proposed engineering metric; not
evidence that the OLP engineer used this exact equation.

Implementation: Do not blend across sharp edges or separate surfaces.

### F26. TCP offset from surface

p_TCP = q_surface + h n_hat

Simple meaning: Place the gun a chosen distance from the surface.

Technical meaning: Offset surface point by standoff h along a chosen
normal direction.

Confidence: Medium --- direction and tool convention must be confirmed..
Evidence / basis: Standard geometric identity or proposed engineering
metric; not evidence that the OLP engineer used this exact equation.

Implementation: Check whether n points toward or away from the target
and whether the controller TCP is calibrated.

### F27. Actual standoff

h_actual = \|\|p_TCP − q_surface\|\|

Simple meaning: Measure gun-to-surface distance.

Technical meaning: Euclidean distance between TCP reference point and
closest target point.

Confidence: High as a metric; Medium for surface correspondence..
Evidence / basis: Standard geometric identity or proposed engineering
metric; not evidence that the OLP engineer used this exact equation.

Implementation: Closest-point pairing can jump across nearby thin
surfaces.

### F28. Standoff error

E_h = max_i \|h_actual,i − h_target\|

Simple meaning: Largest deviation from the requested gap.

Technical meaning: Maximum absolute standoff error along a path.

Confidence: High as a metric.. Evidence / basis: Standard geometric
identity or proposed engineering metric; not evidence that the OLP
engineer used this exact equation.

Implementation: Tolerance depends on process, tool and part.

### F29. TCP position error

e_TCP = \|\|p_generated − p_reference\|\|

Simple meaning: Compare generated and reference robot positions.

Technical meaning: Euclidean position difference after frame and point
correspondence are aligned.

Confidence: High as a metric.. Evidence / basis: Standard geometric
identity or proposed engineering metric; not evidence that the OLP
engineer used this exact equation.

Implementation: Do not compare positions from different work-object
frames.

### F30. Gun pointing axis

a_hat = normalize(q_surface − p_TCP)

Simple meaning: Aim from the gun toward the target point.

Technical meaning: Direction vector from TCP to target; may need
sign/convention adaptation.

Confidence: Medium --- depends on actual gun tool-axis definition..
Evidence / basis: Standard geometric identity or proposed engineering
metric; not evidence that the OLP engineer used this exact equation.

Implementation: Confirm controller/tool frame convention; do not assume
local Z is the gun axis.

### F31. Central tangent estimate

t_hat_i = normalize(q\_(i+1) − q\_(i−1))

Simple meaning: Estimate the direction the path is moving.

Technical meaning: Central finite-difference tangent estimate.

Confidence: High as an estimate; Medium on noisy samples.. Evidence /
basis: Standard geometric identity or proposed engineering metric; not
evidence that the OLP engineer used this exact equation.

Implementation: Use one-sided differences at endpoints and skip
duplicate points.

### F32. Tangent projected into tool-normal plane

x_hat = normalize(t_hat − (t_hat · a_hat)a_hat)

Simple meaning: Keep one tool-frame axis aligned with travel as much as
possible.

Technical meaning: Project tangent onto plane perpendicular to pointing
axis.

Confidence: Medium --- tool-frame convention dependent.. Evidence /
basis: Standard geometric identity or proposed engineering metric; not
evidence that the OLP engineer used this exact equation.

Implementation: Handle near-parallel tangent/axis and maintain sign
continuity.

### F33. Complete orthogonal tool frame

y_hat = normalize(a_hat × x_hat)

Simple meaning: Create a second axis so the tool orientation is fully
defined.

Technical meaning: Construct right-handed frame from pointing axis and
projected tangent.

Confidence: Medium --- frame convention must match robot.. Evidence /
basis: Standard geometric identity or proposed engineering metric; not
evidence that the OLP engineer used this exact equation.

Implementation: Verify determinant/handedness and axis order before
quaternion conversion.

### F34. Rotation matrix

R = \[ x_hat y_hat a_hat \] (columns)

Simple meaning: Package the three tool axes into an orientation.

Technical meaning: Rotation matrix whose columns are the chosen local
axes.

Confidence: Medium --- only correct for the assumed tool-axis
convention.. Evidence / basis: Standard geometric identity or proposed
engineering metric; not evidence that the OLP engineer used this exact
equation.

Implementation: Convert to the robot's required representation and
validate with known poses.

### F35. Angular difference

θ = acos(clamp(a_i · a\_(i+1), −1, 1))

Simple meaning: Measure how much the pointing direction changes.

Technical meaning: Angle between unit pointing vectors.

Confidence: High as a metric.. Evidence / basis: Standard geometric
identity or proposed engineering metric; not evidence that the OLP
engineer used this exact equation.

Implementation: This measures pointing-axis change, not necessarily full
roll/pitch/yaw change.

### F36. Segment length

ℓ_i = \|\|q\_(i+1) − q_i\|\|

Simple meaning: Length of a small move.

Technical meaning: Euclidean distance between consecutive waypoints.

Confidence: High --- standard identity.. Evidence / basis: Standard
geometric identity or proposed engineering metric; not evidence that the
OLP engineer used this exact equation.

Implementation: Robot motion path may not be straight for joint moves.

### F37. Total polyline length

L = Σ ℓ_i

Simple meaning: Add all small move lengths to estimate total path
length.

Technical meaning: Discrete arc-length estimate of the waypoint
polyline.

Confidence: High as a metric.. Evidence / basis: Standard geometric
identity or proposed engineering metric; not evidence that the OLP
engineer used this exact equation.

Implementation: Separate painting length from travel length.

### F38. Arc-length resampling

s_j = j δs; choose q(s_j) along the polyline/curve

Simple meaning: Put points at consistent distances along a path.

Technical meaning: Resample by cumulative arc length with spacing δs.

Confidence: High as a method; δs is process-specific.. Evidence / basis:
Standard geometric identity or proposed engineering metric; not evidence
that the OLP engineer used this exact equation.

Implementation: Do not use equal point index spacing when the original
points are nonuniform.

### F39. Neighboring lane pitch

Δ_k ≈ \|(r − p) · s_hat\|

Simple meaning: Measure sideways separation between two passes.

Technical meaning: Projection of inter-pass displacement onto the local
spacing direction.

Confidence: Medium --- valid when local frames align.. Evidence / basis:
Standard geometric identity or proposed engineering metric; not evidence
that the OLP engineer used this exact equation.

Implementation: Use corresponding points/regions and local spacing
directions on curved surfaces.

### F40. Pitch from spray width and overlap

Δ = w_eff (1 − o)

Simple meaning: Choose lane spacing from effective width and desired
overlap.

Technical meaning: Pitch from effective process width w_eff and overlap
fraction o∈\[0,1).

Confidence: Low until width/overlap are known for the actual process..
Evidence / basis: Standard geometric identity or proposed engineering
metric; not evidence that the OLP engineer used this exact equation.

Implementation: Do not infer effective spray width from mesh geometry
alone.

Caution: Treat as a candidate equation; verify assumptions and calibrate
parameters against actual reference pairs.

### F41. Alternating raster direction

d_k = (−1)\^k d

Simple meaning: Reverse every second pass.

Technical meaning: Serpentine traversal direction for lanes in one
coherent raster region.

Confidence: High for specific observed regions; Medium globally..
Evidence / basis: Standard geometric identity or proposed engineering
metric; not evidence that the OLP engineer used this exact equation.

Implementation: Reset lane index per region and insert safe transitions
between regions.

### F42. RMS surface deviation

E_RMS = sqrt((1/n) Σ dist(q_i,M)²)

Simple meaning: Typical distance of the path from the target surface.

Technical meaning: Root-mean-square point-to-mesh distance.

Confidence: High as a metric.. Evidence / basis: Standard geometric
identity or proposed engineering metric; not evidence that the OLP
engineer used this exact equation.

Implementation: Pair with maximum deviation; RMS alone can hide local
outliers.

### F43. Maximum surface deviation

E_max,surf = max_i dist(q_i,M)

Simple meaning: Largest departure from the target surface.

Technical meaning: Worst-case point-to-mesh distance.

Confidence: High as a metric.. Evidence / basis: Standard geometric
identity or proposed engineering metric; not evidence that the OLP
engineer used this exact equation.

Implementation: Check both positive/negative offset and nearby-surface
ambiguity.

### F44. Orientation continuity

θ_i = acos(clamp(a_i · a\_(i+1), −1, 1))

Simple meaning: Check if the gun pointing direction changes abruptly.

Technical meaning: Adjacent pointing-axis angular change.

Confidence: High for axis continuity; incomplete for full 3D
orientation.. Evidence / basis: Standard geometric identity or proposed
engineering metric; not evidence that the OLP engineer used this exact
equation.

Implementation: Also compare full rotations with relative rotation
R_iᵀR\_(i+1).

### F45. Maximum orientation jump

θ_max = max_i θ_i

Simple meaning: Find the biggest adjacent orientation change.

Technical meaning: Maximum local axis-angle jump across the path.

Confidence: High as a metric.. Evidence / basis: Standard geometric
identity or proposed engineering metric; not evidence that the OLP
engineer used this exact equation.

Implementation: Threshold depends on robot capability and process
quality.

### F46. Coverage ratio

C = A_covered / A_target

Simple meaning: Estimate what fraction of the target area is reached by
the process footprint.

Technical meaning: Coverage ratio computed using a defined effective
footprint and target area.

Confidence: Low until footprint and target-region definition are
validated.. Evidence / basis: Standard geometric identity or proposed
engineering metric; not evidence that the OLP engineer used this exact
equation.

Implementation: A geometric path alone cannot determine true paint
coverage without process footprint assumptions.

Caution: Treat as a candidate equation; verify assumptions and calibrate
parameters against actual reference pairs.

### F47. Collision clearance

c_i = dist(T_i, O); c_min = min_i c_i

Simple meaning: Measure the gap between the robot/tool and obstacles.

Technical meaning: Minimum distance between robot/tool geometry T and
obstacle geometry O along sampled configurations.

Confidence: High as a metric concept; implementation requires robot
geometry.. Evidence / basis: Standard geometric identity or proposed
engineering metric; not evidence that the OLP engineer used this exact
equation.

Implementation: Checking only the TCP point is insufficient; swept robot
links and controller motion need validation.

# 6. Confidence matrix

| Rule / component \| Confidence \| Why \| What raises confidence \|

| --- \| --- \| --- \| --- \|

| Execution-order parsing \| High \| FANUC and DÜRR analysis explicitly
  show declarations and execution order differ. \| Automated parser
  tests against the original programs. \|

| Region-specific planning \| High for existence; Medium for automation
  \| ABB procedures and reference image show separate regions/behaviors.
  \| Compare automatically found regions with annotated references. \|

| Serpentine traversal \| High in observed raster regions \| ABB hood
  and DÜRR Aleta lanes alternate direction. \| Reconstruct the full
  ordered trajectories in their original frames. \|

| 150 mm lane pitch \| High for reported DÜRR region only \| Reported
  lane X coordinates differ by approximately 150 mm. \| Recompute from
  raw targets and confirm units/frame. \|

| Universal lane pitch \| Low \| No evidence that all parts/processes
  use the same width or overlap. \| Use process parameters and multiple
  independent OLP examples. \|

| Smooth curvature-following \| Medium \| Intent is established, but no
  quantified smoothing tolerance is provided. \| Reference-to-generated
  comparison over multiple mesh resolutions. \|

| Normals-based orientation \| Medium \| References contain orientation
  values; normal-based reconstruction is a proposed method. \| Compare
  generated tool frames to reference poses and validate tool convention.
  \|

| Spray coverage model \| Low until calibrated \| Path locations alone
  do not specify real spray footprint or deposition. \| Process data,
  validated effective width, and measured coating results. \|

| Collision/reachability \| Not established by path pattern alone \|
  Reference paths do not prove all robot configurations are safe in this
  implementation. \| Robot model, tool geometry, kinematics and
  offline/physical validation. \|

# 7. Recommended implementation architecture

1.  Reference parser: read the source program's execution instructions
    and resolve target positions, orientations, speeds, process states,
    frames, and external axes.

2.  Coordinate normalization: map mesh and reference data into a common
    declared frame and unit system; retain transform provenance.

3.  Region manager: represent target regions, boundaries, protected
    edges, local frames, and region-specific settings.

4.  Direction estimator: select a primary tangent direction per region
    using a reference path or a stable geometry-derived estimate.

5.  Lane planner: generate lane offsets using a configurable pitch and a
    clearly defined endpoint/boundary policy.

6.  Surface intersection / projection: generate 3D lane geometry on the
    target mesh without merging unrelated surfaces.

7.  Curve conditioner: resample and smooth trajectories while enforcing
    a maximum surface-deviation bound.

8.  Orientation builder: calculate stable surface normals and tool
    frames, with explicit controller/tool convention settings.

9.  Transition planner: generate separate approach, retract, lane
    connector, and region-to-region travel moves.

10. Process-state planner: apply gun/brush enable states, speeds, waits,
    and motion types to the correct segments.

11. Validator: calculate geometric, orientation, coverage, collision,
    and reachability checks; report errors rather than silently hiding
    them.

12. Exporter: preserve the existing export formats and semantics;
    include mode metadata only where compatibility is defined.

# 8. Pseudocode for the proposed Mesh Flow planner

INPUT: target mesh M, selected regions R, process settings P, optional
reference OLP programs 1. Validate mesh, units, transforms, region
selections, and process settings. 2. If reference programs exist: parse
executed instruction order and reconstruct typed motion segments.
estimate region-local directions, lane pitch, orientation conventions,
and process states. 3. For each region r in R: establish a stable local
scan frame. choose/estimate primary pass direction d and spacing
direction s. select lane pitch Δ from explicit process settings or
validated region reference. generate lane planes/curves over the region
bounds. intersect/project lanes onto M and split at boundaries or
disconnected surface components. order lanes in a serpentine pattern
where appropriate. resample curves by arc length. smooth subject to
surface-error and protected-feature constraints. calculate tool
orientation using validated surface/tool-frame conventions. 4. Insert
nonpainting approach/retract/transition moves between segments and
regions. 5. Apply process states, speeds, tool changes, and supported
external-axis behavior. 6. Validate standoff, surface deviation, lane
spacing, orientation jumps, coverage, collisions, reachability, and
export/controller constraints. 7. Export only if all hard checks pass;
include a diagnostic report. OUTPUT: typed, ordered path segments plus
validation report.

# 9. Validation and acceptance criteria

Thresholds below are placeholders until the process owner supplies
acceptable tolerances. Do not treat these example categories as
already-approved production limits.

| Test \| Metric / method \| Pass criterion \| Required evidence \|

| --- \| --- \| --- \| --- \|

| Execution order \| Compare parsed move sequence to source
  instructions. \| Exact order for all supported instructions; unknown
  instructions flagged. \| Golden parser fixtures for ABB, FANUC, and
  DÜRR examples. \|

| Coordinate integrity \| Round-trip transforms; unit and handedness
  checks. \| Within a declared numerical tolerance; no silent unit
  conversion. \| Known target points and transform tests. \|

| Lane pitch \| Project neighboring lane separation onto local spacing
  direction. \| Matches configured pitch within tolerance in stable
  regions. \| Raw reference coordinates and documented pitch setting. \|

| Surface conformity \| RMS and maximum point-to-mesh deviation. \|
  Below process-approved tolerances; protected features preserved. \|
  Multiple meshes, including coarse/noisy meshes. \|

| Smoothness \| Curvature proxy, second-difference score, and maximum
  local change. \| No unexpected spikes; meaningful curvature retained.
  \| Reference comparisons at multiple mesh resolutions. \|

| Tool orientation \| Pointing-axis and full-rotation angular
  differences. \| Within controller/process limits; no unexplained
  flips. \| Known reference poses and verified tool-axis convention. \|

| Standoff \| TCP-to-target distance along the trajectory. \| Within
  process-approved range. \| Calibrated TCP and target surface. \|

| Coverage \| Footprint-based covered area / target area. \| Meets
  required coverage with allowed overlap. \| Validated footprint model
  or measured coating evidence. \|

| Collision and reachability \| Robot/tool swept geometry and inverse
  kinematics. \| No hard collisions; all required poses reachable. \|
  Robot model, tool geometry, controller/offline simulation. \|

| Process states \| Gun/brush/speed state at each segment. \| No
  unintended paint-on transitions; supported state changes preserved. \|
  Source program and exported program comparison. \|

| Regression \| Run fixed reference and synthetic mesh suite. \| No
  regression in existing modes; Mode 1--3 unchanged. \| Automated test
  report and baseline artifacts. \|

| Export compatibility \| Load generated files in target downstream
  application. \| Syntax, frame, orientation, and motion semantics
  accepted. \| Importer/simulator checks; physical validation before
  production. \|

# 10. Implementation roadmap

| Phase \| Work \| Deliverable / exit gate \|

| --- \| --- \| --- \|

| 0. Baseline and rename audit \| Search Mode 4 display strings,
  identifiers, factory/dispatch, config persistence, exports, docs, and
  tests. Capture current behavior. \| Rename map and baseline regression
  report; no algorithm change yet. \|

| 1. Reference normalization \| Parse execution order for the selected
  ABB/FANUC/DÜRR programs; normalize frames, units, orientation
  representations and process states. \| Ordered, typed reference
  trajectories with parser tests. \|

| 2. Reference metrics \| Measure region directions, lane pitch,
  direction alternation, curvature, orientation change, transitions, and
  deviations. \| Machine-readable reference metrics and annotated
  comparison. \|

| 3. Minimal Mesh Flow core \| Implement region-local scan frame, lane
  generation, surface conformity, and serpentine ordering behind a
  feature flag. \| Synthetic meshes and reference-like geometry pass
  tests. \|

| 4. Curve and orientation conditioning \| Add arc-length resampling,
  constrained smoothing, normal handling, and orientation continuity. \|
  Quantified surface/orientation error report. \|

| 5. Transitions and process semantics \| Separate paint and transit
  segments; preserve gun/brush states, speeds, and relevant external
  axes. \| No unintended painting connectors; export semantics verified.
  \|

| 6. Safety and export validation \| Add standoff, collision,
  reachability, coverage, and controller checks where supported. \|
  Hard-failure behavior and simulator evidence. \|

| 7. Controlled comparison \| Compare generated paths with engineer
  references on multiple parts and mesh resolutions. \| Acceptance
  report with tolerances, residual gaps, and confidence updates. \|

| 8. Rename rollout \| Apply the approved user-facing name and canonical
  identifier with legacy compatibility where needed. \| Tests, docs,
  saved-project compatibility, and release notes updated. \|

# 11. Key risks and mitigations

| Risk \| Why it matters \| Mitigation \|

| --- \| --- \| --- \|

| Wrong frame or units \| A mathematically correct path can be
  physically misplaced. \| Explicit frame metadata, unit validation,
  known-point tests. \|

| Global raster across multiple regions \| Can create bad transitions or
  miss side/hood-specific behavior. \| Region-local directions and
  separate transition planning. \|

| Mesh noise causes path waviness \| TCP may oscillate and paint quality
  can degrade. \| Constrained smoothing and resolution-robustness tests.
  \|

| Over-smoothing erases details \| Edges, recesses, and boundary
  coverage may be lost. \| Protected features, maximum deviation bounds,
  region-aware smoothing. \|

| Normal flips or unstable tool frames \| Can cause sudden tool rotation
  or incorrect gun pointing. \| Normal orientation checks, continuity
  constraints, controller convention tests. \|

| Assuming 150 mm is universal \| Spacing may be unsuitable for another
  gun or surface. \| Treat it as a measured DÜRR-region observation
  only; use configurable pitch. \|

| Sorting target IDs as motion order \| Generated path may be scrambled.
  \| Parse executed instructions and test exact order. \|

| Renaming only the UI label \| Internal dispatch or persisted settings
  may still use the old identity. \| Audit UI, identifier, registry,
  config migration, exports, docs and tests together. \|

| Claiming coverage from centerlines alone \| Path lines do not prove
  real spray deposition. \| Use calibrated footprint/process data and
  report coverage assumptions. \|

# 12. Decisions required before production implementation

-   Confirm which source of lane pitch takes precedence: explicit user
    setting, process width/overlap, or measured reference pitch.

-   Define target-region selection and how boundaries, holes,
    disconnected surfaces, and protected features are represented.

-   Confirm the robot tool/TCP frame and gun pointing axis for each
    supported exporter.

-   Define painting process constraints: allowable standoff range, angle
    limits, speed, overlap, and whether painting while turning is
    allowed.

-   Select hard acceptance tolerances for surface deviation, orientation
    jumps, collision clearance, and coverage.

-   Decide the canonical internal identifier and backward-compatibility
    policy for the Mesh Flow rename.

-   Provide complete original OLP files and corresponding geometry for
    automated, reproducible reference comparisons.

# 13. Final recommendation

Proceed with Mesh Flow as a region-aware, surface-adaptive raster
planner---not as a promise to follow every triangle or to use a single
global raster. Implement reference parsing and metrics before tuning the
generator. Treat the observed serpentine pattern and the reported DÜRR
lane pitch as region-specific evidence. Keep candidate optimization
weights, smoothing thresholds, spray-width assumptions, and automatic
segmentation explicitly unvalidated until quantitative reference
comparisons are available.

The safest engineering sequence is: establish exact reference execution
order → normalize coordinate frames → quantify patterns → implement a
minimal planner → validate geometry and orientation → preserve process
states → validate exports and robot safety → apply the rename
consistently. No source code was modified to create this document.

# Appendix A --- Compact rule-to-formula map

| Rule \| Main formulas \| Purpose \|

| --- \| --- \| --- \|

| Direction/frame \| F01--F07, F13 \| Define local travel and lateral
  directions. \|

| Lane generation \| F08--F12, F39--F41 \| Place and order raster lanes.
  \|

| Straightness and smoothing \| F14--F23, F38 \| Measure straightness
  and smooth paths while respecting the surface. \|

| Normals, TCP and orientation \| F24--F35, F44--F45 \| Construct tool
  positions and orientation continuity. \|

| Distance and path length \| F02, F27--F29, F36--F38 \| Measure
  geometric error and motion length. \|

| Quality and safety \| F42--F47 \| Check surface error, coverage and
  collision clearance. \|

# Appendix B --- Confidence interpretation

High confidence does not mean production validated. It means the
supplied source analysis directly supports the observation or the
equation is a standard geometric identity. Medium confidence means the
method is technically plausible but depends on geometry, frame
conventions, or parameters. Low confidence means the design choice
requires calibration and comparative tests. Confidence should be
upgraded only when reproducible tests on actual reference programs and
corresponding meshes support it.

# 14. Additional Rules from Path-Quality Review

This section extends the original Mesh Flow specification with
additional rules prompted by the displayed path failures. These are
proposed engineering requirements, not implemented behavior. The
screenshots are from the older version and are treated as evidence of
failure modes, not evidence that the new design has already been
implemented.

Core design principle: simplify unreliable geometric detail without
blindly simplifying away the actual component. A broad, gentle body
curve should remain; tiny triangle-level fluctuations should not steer
the toolpath; genuine boundaries and safety-relevant features must be
preserved or explicitly flagged.

| ID / rule \| Simple explanation \| Technical requirement \| Confidence
  / status \| Implementation and validation \|

| --- \| --- \| --- \| --- \| --- \|

| R11 --- Mesh Quality Assessment \| Detect unreliable geometry before
  planning. \| Compute local indicators such as triangle aspect ratio,
  edge-length variation, normal variation, boundary proximity, connected
  components, and local sampling density. Treat these as indicators, not
  a single definitive quality score. \| Medium: proposed requirement
  inferred from the screenshot failures; thresholds are unvalidated. \|
  Create a per-region quality map. Validate against clean, noisy,
  incomplete, and mixed-quality meshes. Avoid rejecting a whole part
  because of one local defect. \|

| R12 --- Adaptive Geometry Simplification \| Ignore tiny triangle
  fluctuations but preserve the body's overall shape. \| Create a
  scale-aware representation for planning, while retaining the original
  mesh for final surface-distance checks and protected features. Bound
  simplification error relative to the original mesh. \| Medium:
  established engineering approach; project-specific tolerances are
  unvalidated. \| Test at multiple mesh resolutions and noise levels.
  Compare simplified-surface deviation and protected-feature retention.
  \|

| R13 --- Straightness Priority \| Prefer straight passes unless
  meaningful body curvature requires a bend. \| Optimize pass direction
  and path shape to minimize lateral deviation from a nominal straight
  line, subject to surface-conformity, standoff, boundary, and safety
  constraints. \| Medium: directly reflects the stated design goal;
  weights/tolerances are unvalidated. \| Measure maximum and RMS line
  deviation together with surface deviation. Do not flatten genuinely
  curved areas just to improve straightness. \|

| R14 --- Curvature-Limited Smoothing \| Remove spikes and sudden bends
  without making every pass flat. \| Apply curve conditioning with
  limits on curvature and curvature change, while enforcing maximum
  deviation from the target surface and protecting boundaries/features.
  \| Medium: candidate method; numeric thresholds need reference-based
  calibration. \| Test for both under-smoothing (wiggles remain) and
  over-smoothing (real shape is lost). Resample by arc length before
  comparing curvature. \|

| R15 --- Path Regularity \| Keep neighboring passes consistent where
  the surface allows. \| Evaluate pass pitch, direction consistency,
  pass length, endpoint behavior, and local curvature. Permit justified
  variation near boundaries and transitions. \| Medium: supported by
  observed raster patterns, but not universal across all regions. \|
  Report lane-spacing variation and abrupt endpoint/length changes.
  Compare only corresponding regions and coordinate frames. \|

| R16 --- Boundary-Aware Generation \| Do not connect unrelated surface
  patches or cross holes just to complete a line. \| Split candidate
  paths at target boundaries, holes, disconnected components,
  occlusions, and user-defined exclusion zones. Create explicit transit
  links only after collision and process-state checks. \| High for the
  need to separate regions/transitions; Medium for automated boundary
  classification. \| Use synthetic gaps, holes, disconnected shells, and
  real reference boundaries. Confirm no unintended paint-on connector
  crosses an excluded area. \|

| R17 --- Confidence-Based Fallback \| Approximate only when the surface
  can be estimated reliably; otherwise flag or skip the region. \|
  Assign each region a quality/confidence state based on geometry
  support and planner diagnostics. A fallback may produce a candidate
  path only if standoff, surface-deviation, boundary, and collision
  checks can still be satisfied. \| Medium: safety-oriented proposal;
  confidence thresholds are not yet defined. \| Distinguish `candidate`,
  `accepted`, and `rejected/needs review` states. Never silently treat
  missing geometry as known surface. \|

| R18 --- Path Quality Scoring \| Score the path before accepting it. \|
  Calculate separate metrics for straightness, smoothness, surface
  deviation, spacing consistency, orientation continuity, and process
  coverage. Use hard constraints for safety-critical limits; do not let
  a weighted score hide a hard failure. \| High for metric-based
  validation as a design principle; Low until scoring weights/thresholds
  are calibrated. \| Produce per-region reports and reasons for
  rejection. Test cases with one intentionally bad metric at a time. \|

| R19 --- Multi-Scale Geometry \| Use broad shape for the main path;
  consult finer details only when needed. \| Separate low-frequency
  shape from high-frequency mesh variation using a scale-aware
  representation. Preserve verified edges, recesses, and other protected
  features rather than smoothing them away. \| Medium: proposed
  approach; scale selection depends on mesh and process. \| Compare path
  results across mesh resolutions and noise perturbations. Verify that
  genuine features remain represented. \|

| R20 --- Preview and Diagnostics \| Show where the mesh or path is
  unreliable before export. \| Expose geometry-quality regions, skipped
  areas, curvature spikes, spacing deviations, orientation jumps, and
  validation failures in the GUI or report. \| High as a
  usability/traceability requirement; UI implementation remains pending.
  \| Provide diagnostic reason codes and region/waypoint identifiers;
  test that each warning maps to a reproducible condition. \|

# 15. Straight-First, Quality-Gated Planning Policy

The preferred order is: (1) estimate the broad surface shape; (2) choose
the simplest locally straight pass direction; (3) bend the pass only as
needed to follow meaningful surface curvature; (4) suppress small-scale
mesh noise; (5) preserve real boundaries and protected features; (6)
validate surface deviation, standoff, orientation and collisions; (7)
accept, flag for review, or reject each region. This is a planning
policy, not a claim that these steps already exist in the current code.

## 15.1 Region decision table

| Region condition \| Planner behavior \| Acceptance condition \|

| --- \| --- \| --- \|

| Clean, broad, nearly flat \| Generate straight, evenly spaced passes.
  \| Surface, standoff, orientation and coverage checks pass. \|

| Clean, broad, curved \| Generate smooth curved passes following the
  broad shape. \| Curvature remains continuous and surface deviation is
  within tolerance. \|

| Noisy but shape is recoverable \| Plan on a simplified representation;
  validate against original mesh. \| Original-mesh deviation and safety
  checks pass. \|

| Sharp edge or protected feature \| Split or constrain smoothing at the
  feature. \| Feature is retained and no unsafe orientation jump is
  introduced. \|

| Hole, disconnected patch, or exclusion \| Stop/split the paint path;
  plan a separate safe transit if appropriate. \| No paint-on connector
  crosses the excluded region. \|

| Missing/ambiguous geometry \| Flag for review or skip; approximate
  only with independent validated support. \| No region accepted solely
  on an unsupported guess. \|

## 15.2 Proposed quality gates

-   Geometry gate: identify unreliable areas and retain a traceable map
    of the evidence used.

-   Shape gate: ensure simplification has not removed protected geometry
    or exceeded a defined deviation.

-   Path gate: report straightness, smoothness, lane pitch, and
    endpoint/transition regularity.

-   Process gate: verify tool orientation, standoff, paint-on/off state,
    and required motion semantics.

-   Safety gate: check collision and reachability with the actual
    robot/tool model where available.

-   Export gate: do not silently export rejected or unreviewed regions
    as production-ready paths.

# 16. Formula additions for the new rules

These formulas are candidate metrics for implementation and diagnostics.
They do not prescribe universal thresholds; calibrate those thresholds
against actual reference paths, mesh quality, coating requirements, and
robot constraints.

### F48 --- Relative edge-length variation

CV_l = std(l_e) / mean(l_e)

Simple meaning: Shows how uneven the mesh edge lengths are locally.

Technical meaning: A local coefficient of variation for edge lengths;
high values can indicate mixed sampling density but are not proof of bad
geometry.

Confidence: Medium; proposed indicator.

Implementation: Calculate per region and compare with known clean/noisy
examples; avoid using it as the sole rejection criterion.

### F49 --- Neighbor-normal variation

θ_ij = acos(clamp(n_i · n_j, −1, 1))

Simple meaning: Measures the angle between neighboring face normals.

Technical meaning: Local normal-angle variation can identify sharp
features or noisy triangulation; topology and scale must be considered.

Confidence: High as a metric; Medium as a noise detector.

Implementation: Separate coherent sharp edges from irregular
high-frequency normal changes.

### F50 --- Simplification deviation

E_simp,max = max_i dist(v_i, M_s)

Simple meaning: Measures how far original mesh samples are from the
simplified mesh.

Technical meaning: Maximum distance from original vertices/samples to
simplified surface M_s; also consider reverse-direction distance to
avoid one-sided error.

Confidence: High as a metric; tolerance unvalidated.

Implementation: Check both directions and protect edges/features.

### F51 --- Straightness objective

E_straight = (1/n) Σ e_i²

Simple meaning: Rewards a pass that stays close to its nominal straight
line.

Technical meaning: Mean squared perpendicular deviation, subject to
surface and safety constraints.

Confidence: Medium; objective weights unvalidated.

Implementation: Never optimize this term alone; pair with surface
conformity and maximum-deviation constraints.

### F52 --- Curvature-change penalty

E_κ = Σ (κ\_(i+1) − κ_i)²

Simple meaning: Penalizes sudden changes in bending.

Technical meaning: Discrete curvature variation penalty for
arc-length-resampled points.

Confidence: Medium; numerical form needs careful discretization.

Implementation: Use consistent sampling and retain real boundaries; do
not smooth across region breaks.

### F53 --- Lane-spacing variation

σ_Δ = std(Δ_k)

Simple meaning: Measures how inconsistent neighboring pass spacing is.

Technical meaning: Standard deviation of measured local lane pitch
within a region.

Confidence: High as a metric; interpretation depends on curvature and
boundaries.

Implementation: Compare within coherent regions and allow deliberate
boundary adaptation.

### F54 --- Multi-objective path score

J = w_s E_straight + w_c E_κ + w_m E_surface + w_p E_pitch

Simple meaning: Balances straightness, smoothness, surface fit and lane
regularity.

Technical meaning: Weighted score for ranking candidate paths; terms
should be normalized and hard safety constraints remain separate.

Confidence: Low until calibrated.

Implementation: Store each component separately so a good aggregate
cannot hide a bad surface or safety metric.

### F55 --- Region confidence

Q_r = f(Q_mesh, E_surface, E_smooth, E_orientation, Q_safety)

Simple meaning: Combines evidence about whether a region is trustworthy
enough to accept.

Technical meaning: A policy-based quality state or calibrated score from
mesh and path metrics; no universal formula for f is assumed.

Confidence: Low until designed and tested.

Implementation: Prefer explicit pass/fail gates and reason codes before
introducing a single opaque score.

# 17. Revised implementation roadmap additions

1.  Add a mesh-quality analysis stage before lane generation; visualize
    quality warnings by region.

2.  Add a simplified planning surface while retaining the original mesh
    for deviation validation.

3.  Add straightness-first candidate generation with bounded surface
    deviation and curvature-change checks.

4.  Add boundary/protected-feature handling and a confidence-based
    region accept/reject state.

5.  Add quality report fields for straightness, smoothness, pitch
    variation, surface deviation, orientation continuity, and reasons
    for skipped regions.

6.  Test the same geometry at multiple tessellation densities and with
    controlled mesh noise; generated paths should remain broadly stable.

7.  Only promote thresholds to production defaults after comparison
    against multiple engineer-created OLP paths and matching meshes.

Status note: the original specification and these additions describe a
proposed design. The old-version screenshots do not prove the new rules
have been implemented. No source code has been changed; this update only
revises the Word document.

# 18. Outer-Surface-Only Path Generation and Mesh-Penetration Prevention

Requirement: Mesh Flow must generate painting paths only on the intended
outer/top target surface, must not generate paths beneath the target
layer, and must prevent path segments and robot/tool geometry from
penetrating or colliding with the mesh. These are proposed requirements,
not implemented or validated behavior.

## 18.1 Surface-membership constraint

Equation: d(pᵢ, S_target) ≤ εₛ

Simple meaning: every painting-path point must remain within an allowed
distance εₛ of the intended target surface.

Technical meaning: calculate closest-point-on-triangle distance to the
selected target surface. Use the original mesh for final checking even
if a simplified surface is used for planning. Confidence: High for the
constraint; εₛ must be calibrated.

## 18.2 Inside/outside classification

Equation: I(p) = 1 if p is inside M; otherwise I(p) = 0. Require I(pᵢ) =
0.

Simple meaning: reject path points inside the solid mesh. Technical
meaning: use a robust point-in-mesh or signed-distance test only when
the mesh supports reliable inside/outside classification. Open,
self-intersecting, non-manifold, or incomplete meshes may make this test
unreliable. Confidence: High for valid closed meshes; low for defective
meshes.

## 18.3 Segment-level non-penetration

Equation: x(t) = (1 − t)pᵢ + t pᵢ₊₁, 0 ≤ t ≤ 1; require x(t) ∉
Interior(M) along every segment intended to remain outside.

Simple meaning: checking only endpoints is not enough; the connection
between them must not pass through the mesh. Technical meaning: check
segment/triangle intersections and validate the actual interpolated
robot trajectory where it differs from a straight chord. Confidence:
High as a geometric requirement.

## 18.4 Outward TCP standoff

Equation: p_TCP,i = qᵢ + h n̂ᵢ, h \> 0.

Simple meaning: position the gun outside the target at the requested
distance. Technical meaning: qᵢ is the surface point, n̂ᵢ is a verified
outward unit normal, and h is standoff. Verify normal orientation, tool
convention, and TCP calibration. Confidence: Medium until conventions
are verified.

## 18.5 Optional facing-direction filter

Equation: n̂ᵢ · v̂ ≥ τ.

Simple meaning: where a meaningful local facing direction v̂ is known,
use it to help select the intended outer-facing surface. Technical
meaning: τ is a threshold. This is not a universal top-surface test for
curved bodies; prefer an explicitly selected target region when
available. Confidence: Medium for a defined local direction.

## 18.6 Tool and robot collision checks

The TCP alone is not enough to prove collision freedom. Check the
gun/tool body and robot links over the complete painting, approach,
retract, and transition motions, using the actual robot model and
controller motion assumptions where available. Block production export
if a hard collision check fails.

## 18.7 Acceptance and failure behavior

| Check \| Acceptance condition \| Failure behavior \|

| --- \| --- \| --- \|

| Target surface \| Points remain within εₛ of selected target surface.
  \| Reject/regenerate affected region. \|

| Inside/outside \| Mesh supports reliable classification; paint points
  are outside solid. \| Flag ambiguous mesh; do not claim safety. \|

| Whole segment \| No segment penetrates mesh interior. \| Reject
  segment and report waypoint IDs. \|

| Outward standoff \| TCP is offset outward and within standoff
  tolerance. \| Flag normal/frame uncertainty. \|

| Outer surface selection \| Path belongs to explicitly selected or
  reliably identified region. \| Do not silently select an
  underside/internal face. \|

| Robot/tool collision \| Tool and robot links maintain required
  clearance throughout motion. \| Block production export on hard
  failure. \|

If the mesh is too incomplete or defective to identify the outer surface
or reliably test penetration, flag or skip that region unless a separate
validated surface model is available.

# 19. Formula additions for surface-only and collision-safe paths

| Formula \| Equation \| Simple purpose \| Confidence / caveat \|

| --- \| --- \| --- \| --- \|

| F56 --- Surface distance \| d(pᵢ, S_target) ≤ εₛ \| Keep path points
  close to the intended surface. \| High as a constraint; tolerance
  unvalidated. \|

| F57 --- Inside/outside \| I(pᵢ) = 0 \| Reject points inside a valid
  solid mesh. \| Reliable only when mesh supports classification. \|

| F58 --- Segment non-penetration \| x(t)=(1−t)pᵢ+tpᵢ₊₁;
  x(t)∉Interior(M), ∀t∈\[0,1\] \| Check the full segment, not just
  endpoints. \| Also check actual controller interpolation. \|

| F59 --- TCP standoff \| p_TCP,i=qᵢ+h n̂ᵢ, h\>0 \| Offset tool outward
  from target. \| Verify normal and tool conventions. \|

| F60 --- Facing filter \| n̂ᵢ·v̂≥τ \| Filter candidate faces using a
  known local direction. \| Not universal for curved surfaces. \|

These formulas do not independently guarantee robot safety. Full
tool/link collision, reachability, and motion validation remain
necessary.

Status: this is a specification update only. No SmartGrid source code
has been modified.

# 20. Conflict Resolution and Formula Clarifications

Purpose: resolve interactions between the existing rules and clarify how
the equations should be applied. These are specification decisions for
the proposed Mesh Flow design; they do not imply that the current code
implements them.

## 20.1 Precedence rules

When two objectives disagree, mandatory geometry and safety constraints
take priority over path appearance. The planner should first satisfy
target-surface selection, surface-deviation tolerance, standoff,
boundary, collision, and reachability constraints. Only among feasible
candidate paths should it optimize straightness, smoothness, and
regularity.

| Rule interaction \| Potential conflict \| Resolution \| Priority \|

| --- \| --- \| --- \| --- \|

| Straightness vs. surface conformity \| A straight pass can depart from
  a curved body. \| Enforce a maximum surface-deviation/standoff error
  first; prefer straightness only among paths that meet it. \| Surface
  constraint first \|

| Smoothing vs. preserving details \| Smoothing may erase real edges,
  recesses, or boundaries. \| Suppress small-scale noise, but protect
  meaningful features and split smoothing at region boundaries. \|
  Feature/boundary preservation first \|

| Simplified planning surface vs. original mesh \| Simplification can
  hide obstacles or real shape changes. \| Use the simplified surface to
  plan; validate final paths against the original mesh and separate
  obstacle geometry. \| Original-geometry validation first \|

| Outer-surface selection vs. nearest-surface query \| The nearest
  triangle may be an underside or internal face. \| Restrict candidate
  faces to an explicitly selected or reliably identified target region
  before nearest-point projection. \| Target-region identity first \|

| Surface normal vs. TCP offset \| A reversed normal can offset the TCP
  inward. \| Verify outward normal direction and tool/TCP convention
  before applying standoff. \| Normal verification first \|

| Serpentine direction vs. region transitions \| Alternating lanes may
  create unsafe cross-region connectors. \| Alternate only within a
  coherent region; plan approach, retract, and inter-region transit
  separately. \| Region separation first \|

| Approximate fallback vs. uncertain geometry \| A smooth-looking
  fallback can be in the wrong location. \| Accept approximation only if
  surface deviation, standoff, boundary, and safety checks pass;
  otherwise flag or skip. \| Evidence and safety first \|

| Weighted quality score vs. hard safety checks \| A good aggregate
  score can hide a collision or penetration. \| Use safety and geometry
  limits as hard gates; use weighted scores only to rank candidates that
  pass. \| Hard gates first \|

| Straightness vs. orientation continuity \| A geometrically smooth path
  can still produce sudden tool rotation. \| Validate path geometry and
  orientation separately; reject unexplained orientation flips or
  excessive jumps. \| Both must pass \|

| Coverage vs. collision clearance \| A path aimed at full coverage may
  approach obstacles too closely. \| Never sacrifice required collision
  clearance to improve coverage; report uncovered areas separately. \|
  Safety first \|

## 20.2 Formula corrections and clarifications

### C1 --- Surface point and TCP are different

Let qᵢ be the target point on the selected surface and p_TCP,ᵢ be the
tool centre point. Apply d(qᵢ, S_target) ≤ εₛ to the surface point, not
to the offset TCP. Then define p_TCP,ᵢ = qᵢ + h n̂ᵢ when the outward
normal and tool convention are verified.

Simple: the painting target belongs on the surface; the gun itself stays
outside it.

### C2 --- Inside/outside test has mesh-validity preconditions

I(p)=0 is usable only when the mesh representation supports reliable
inside/outside classification. For open, self-intersecting,
non-manifold, or incomplete meshes, return an indeterminate status
rather than assuming outside.

Simple: if the mesh is broken, do not trust a test that says whether a
point is inside.

### C3 --- Check the actual motion, not only a waypoint chord

For a linear segment use x(t)=(1−t)pᵢ+t pᵢ₊₁, 0≤t≤1. For
joint-interpolated, circular, or otherwise controller-specific motion,
validate the actual interpolated tool and robot-link trajectory or a
conservative swept volume.

Simple: safe endpoints do not guarantee a safe movement between them.

### C4 --- Direction filter is conditional

The condition n̂ᵢ·v̂≥τ is only a candidate-face filter when v̂ is a
meaningful, defined region-local reference direction. It is not a
universal top-surface classifier for curved components. Prefer explicit
region selection or a validated surface-side definition.

Simple: a single global up direction cannot reliably identify the
outside of every curved surface.

### C5 --- Quality-score terms require compatible scaling

Before combining E_straight, E_curvature, E_surface, and E_pitch,
normalize the terms or define weights with documented units and
calibration. Preserve each component metric in diagnostics. Hard
constraints must not be converted into soft penalties.

Simple: do not add unlike measurements without scaling, and never let a
good score excuse a safety failure.

### C6 --- Standoff must be evaluated along the path

For TCP point p_TCP,ᵢ and corresponding target point qᵢ, evaluate
hᵢ=\|\|p_TCP,ᵢ−qᵢ\|\| and require hᵢ to remain within the approved
process range. If the process requires a directional standoff, also
check alignment with the verified normal.

Simple: check tool distance at every relevant point, not just the first
point.

### C7 --- Surface conformity and penetration are separate checks

A point can be close to the target surface yet lie on the wrong side or
belong to the wrong surface patch. Check target-region membership,
point-to-surface deviation, inside/outside where reliable, and
segment/tool collision independently.

Simple: being near the mesh does not prove the path is on the correct
side or safe.

### C8 --- Coverage estimates need a calibrated footprint

A coverage ratio based on a geometric footprint is an estimate unless
effective spray width, overlap, incidence angle, standoff, and process
behavior have been validated. Report assumptions and do not infer
deposited coating quality from centerlines alone.

Simple: drawing lines over a surface does not prove the paint will cover
it properly.

## 20.3 Final order of operations

1.  Validate mesh quality, coordinate frames, units, and target-region
    selection.

2.  Establish a trustworthy outer-surface representation; mark ambiguous
    regions as indeterminate.

3.  Generate candidate lanes using a region-local direction and
    configured/validated pitch.

4.  Smooth and simplify only within protected-feature and
    maximum-deviation constraints.

5.  Construct TCP positions and tool orientations using verified normals
    and conventions.

6.  Plan painting lanes and nonpainting transitions as separate motion
    segments.

7.  Apply hard gates: target-surface membership, surface deviation,
    standoff, mesh penetration, tool/robot collision, and reachability
    where supported.

8.  Calculate quality metrics and rank only candidates that pass all
    mandatory gates.

9.  Report rejected/skipped regions and the exact reasons; do not
    silently replace them with unverified paths.

10. Validate exported motion in the target simulator/controller workflow
    before physical use.

## 20.4 Final verification status

| Item \| Specification status \| Evidence still required \|

| --- \| --- \| --- \|

| Rule conflicts and precedence \| Clarified in this revision \| Review
  by the process/OLP engineer. \|

| Formula meanings and preconditions \| Clarified in this revision \|
  Unit tests and numerical edge-case tests. \|

| Outer-surface-only selection \| Defined as a requirement \| Matching
  mesh, selected regions, and reference paths. \|

| Mesh penetration and robot collision \| Defined as mandatory checks \|
  Valid mesh/robot/tool models and actual motion interpolation. \|

| Thresholds and objective weights \| Not finalized \| Calibration
  against reference OLP and process requirements. \|

| Reference-path matching \| Not proven by this document \| Automated
  quantitative comparison with original OLP programs and corresponding
  geometry. \|

| Production readiness \| Not verified \| Simulation,
  controller-specific validation, and appropriate physical validation.
  \|

Final note: this revision resolves specification-level ambiguities; it
does not certify the algorithm or current SmartGrid implementation. No
project source code was modified.
