# SmartGrid: final design for the new Mesh Surface mode

## 1. Purpose and priorities

The mode generates robot paths for a paint machine that must lay down **even paint thickness** over the whole outer surface of a part.

In order of priority:

1. **Straight paths,** both on screen and in the exported file.
2. **Exact pitch,** with no overlap and no crossing between passes.
3. **Constant standoff** within each pass.
4. **Full coverage and zero collision,** which are hard rules throughout.

## 2. Inputs and defaults

| Setting | Default | Notes |
|---|---|---|
| Pitch | user value | Center-to-center spacing of passes |
| Standoff | 250 mm | User picks from 150–300 mm; fixed for the whole run |
| Standoff tolerance | ±5 mm | User-editable; controls how long straight segments can be |
| Direction | H | H: passes run horizontally, stacked top to bottom. V: passes run vertically, stacked left to right |
| Hole gap threshold | 75 mm | User-editable, 50–100 mm |
| Lead-in / lead-out | 50 mm | User-editable |
| Corner angle | 20° | Pending mentor confirmation |

No face or region selection is used in this mode.

## 3. How a path is generated

**Step 1: Outer skin.** The whole mesh is loaded, and only the outer, visible surface is kept for painting. Each face is tested with rays from outside, and faces that can't be seen (the back of a bumper, inner ribs) are excluded. This removes junk fragment passes at the source.

**Step 2: First pass.** For H, the first pass is placed pitch/2 below the top edge of the part. It's the line where a horizontal plane meets the outer skin, giving the pass's line on the surface. For V, the first pass is placed the same way from the left edge.

**Step 3: Standoff line.** Each point S on the surface line is moved outward along the surface normal n by the standoff, giving `P = S + standoff × n`. This is the line the gun would follow at exactly the standoff distance.

**Step 4: Straightening.** That line is replaced by the fewest straight segments such that every point along every segment stays within ±5 mm of the standoff. The points where segments meet are the corner waypoints, and they fall where the part bends. On flat areas segments are long; on curved areas they're shorter.

**Step 5: Next pass.** The next pass is built from the previous one, not from a fixed grid:

- Measure distance across the surface (geodesic distance) from the previous pass's line on the surface.
- The next pass's surface line is where that distance equals the pitch.
- Steps 3 and 4 turn it into straight segments.
- At every waypoint the spacing is exactly pitch. Between waypoints, the straight segment may deviate slightly; this is measured and reported.

Each pass is defined as exactly one pitch from the previous one, so passes can't cross. Where a curve folds (in concave areas, for example), any loop that would bring a pass closer than pitch to the previous one is trimmed.

**Step 6: Holes.**

- Gap of 75 mm or less: the pass continues straight across with the gun on.
- Gap over 75 mm: the pass splits; the gun turns off and a connector crosses the gap.

**Step 7: Extra waypoints.**

- Any turn sharper than 20° gets an approach point and an exit point, pitch/2 before and after the corner, shortened if the segment is too short.
- Any sharp change in height or position along a pass gets points before and after it, by the same rule.
- Each pass is extended 50 mm past the part edge at both ends (lead-in and lead-out), so the gun is at steady speed when it reaches the part.
- Every added point is checked against the ±5 mm standoff rule.

**Step 8: Overhang pass.** When the next pass would fall off the part, one final full pass is added one pitch beyond the last pass, deliberately overhanging the edge. Its shape is taken from the last pass, shifted one pitch outward, so the leftover strip is covered without squeezing the spacing.

**Step 9: Gun direction (NX/NY/NZ).** Calculated entirely from the mesh, never random. The same mesh and settings always give the same values.

- n is taken from smoothed vertex normals at S, blended across the triangle containing S, then area-averaged over the surface within pitch/2 of S. This stops the gun twitching at triangle edges and small ripples.
- After straightening, the direction is recalculated from the actual positions: `N = (P − S) / |P − S|`, written as NX, NY, NZ. The gun aims along −N, perpendicular to the smoothed surface.
- Approach and exit points are calculated the same way, so the gun turns gradually across a corner.
- Lead-in, lead-out and the off-part portion of the overhang pass copy N from the nearest waypoint on the part.
- Connectors are left blank, as now.
- In the export, CSV, RoboDK and Visual Components write N; DELMIA APT writes `I,J,K = −N`.

**Step 10: One continuous path.** Passes alternate direction (left-right, then right-left) from the first to the overhang pass, joined by connectors with the gun off. The result has one start point and one end point. Connectors may travel over the back of the part.

**Step 11: Safety check.** The existing collision check runs on the final path and must report zero.

## 4. How the result is verified

| Check | Pass condition |
|---|---|
| Spacing at waypoints | Equals pitch |
| Spacing between waypoints | Reported per pass (max deviation) |
| Overlap | None beyond pitch tolerance |
| Gaps in coverage | None on the outer skin |
| Crossings between passes | Zero |
| Standoff along every segment | Within ±5 mm |
| Collisions | Zero |
| Waypoints per pass | Reported |
| Generation time | Reported; should be reasonable |

The final acceptance is the mentor's visual review, from views the mentor chooses.

## 5. What changes in the code

**Replaced within Mesh Surface:**

- Region classification and face selection
- The outward-normal threshold (−0.25)
- The 80 mm gap cluster filter
- `_filter_polylines`
- Pitch/5 waypoint resampling

**New:**

- Outer skin detection
- Pass-from-previous-pass generation with geodesic distance
- Straightening with ±5 mm standoff tolerance
- Hole, corner, lead-in/lead-out and overhang rules
- Smoothed-normal gun direction
- The measurement tool

**Reused unchanged:**

- `PaintPass`, `PaintRoute`
- The connector
- The collision check
- All exporters
- The geodesic distance code from `geodesic_generator.py`, as the basis for Step 5

## 6. Known limits

- **Spacing between waypoints:** spacing is exact at waypoints and may deviate slightly mid-segment. The measurement tool shows how much.
- **Narrow tips and openings:** a pass may split or end early there. It will still never cross another pass.
- **Generation time:** geodesic distance is computed once per pass, so very fine meshes may be slower. This will be measured.

---

# Things to verify with the mentor

Each item shows the current default, so the mentor only needs to confirm or correct it. **Most important: 2, 5, 12, 20 and 21.**

## Spray settings

1. **Standoff:** is 250 mm a good default, with the user choosing from 150–300 mm per part?
2. **Standoff tolerance:** is ±5 mm within a pass acceptable for even paint?
3. **Paint speed:** what speed does the robot move at during a pass, and must it stay constant through corners?
4. **Pitch:** what pitch do they use for this kind of part?

## Path shape

5. **Spacing between waypoints:** pitch is exact at waypoints but may drift slightly mid-segment. What's the maximum acceptable deviation?
6. **Corner rule:** is a 20° turn the right threshold for a corner waypoint?
7. **Approach and exit points:** is pitch/2 before and after each corner right?
8. **Lead-in and lead-out:** is 50 mm past the part edge correct?
9. **Hole gap:** is 75 mm the right threshold for switching the gun off over an opening?
10. **Overhang pass:** is one extra pass one pitch beyond the last pass the right way to cover the far edge?
11. **Pass direction:** H top to bottom and V left to right, with one start and one end. Is that their standard?

## Gun direction

12. **Aim:** always perpendicular to the smoothed surface, or is some tilt allowed? If so, how much?
13. **Turning at corners:** how fast can the gun change direction without affecting paint quality?
14. **Smoothing:** is averaging the surface normal over pitch/2 around the aim point acceptable?

## Surface to paint

15. **Outer skin only:** confirm the back and inside of the part are never painted.
16. **Connectors:** may connectors travel over the back of the part?

## Machine and export

17. Which robot and controller will run the paths?
18. Which of SmartGrid's export formats will they use (RoboDK, DELMIA APT, Visual Components, G-code, JSON/CSV)?
19. How should straight moves and gun direction be written in that format?

## Reference material to collect

20. The sample mesh of the part (STL or STEP).
21. The reference path exported from EcoScreen as a waypoint file, not only screenshots.
22. The reference code, and which part of it handles straight passes and corners.

## Acceptance

23. What exactly does the mentor check in the visual review, and from which views?
24. Is there a target generation time per part?
