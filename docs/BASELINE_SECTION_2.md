# Section 2 — Baseline measurement of current Mesh Surface

Record of current generator behaviour on the proxy bumper mesh. This is a
"before" snapshot; no findings here are pass/fail against the new design.

---

## Settings

| Parameter | Value |
|---|---|
| Mesh file | `models/test_bumper.stl` |
| Mesh description | Open cylindrical arc shell, outward normals |
| Radius | 900 mm |
| Height | 400 mm |
| Angle range | −0.6 to +0.6 rad (68.8°) |
| Chord width | 1016 mm |
| Vertices / Faces | 3,321 / 6,400 |
| Watertight | No |
| Angular steps | 80 (81 pts) |
| Height steps | 40 (41 pts) |
| Pitch | 100 mm |
| Standoff target | 250 ± 5 mm |
| Direction | horizontal |

---

## Summary table (measure.py output)

```
============================================================
  MESH SURFACE MEASUREMENT SUMMARY
============================================================
  Pitch              : 100.0 mm
  Standoff target    : 250.0 +/- 5.0 mm
  Direction          : horizontal
  Generation time    : 0.02 s
  Total passes       : 15
  Total waypoints    : 1325

  SPACING (surface, vs pitch)
    % OK             : 0.3 %
    % overlap        : 97.2 %
    % gap            : 2.5 %

  STANDOFF
    out-of-tolerance : 1325 samples

  CROSSINGS
    count            : 15
    passes (0,5)   dist=0.00 mm
    passes (0,10)  dist=0.50 mm
    passes (1,11)  dist=0.00 mm
    passes (1,6)   dist=0.50 mm
    passes (2,7)   dist=0.00 mm
    ... and 10 more

  THICKNESS (Gaussian, sigma=pitch/2)
    mean             : 40.104
    min              : 9.864
    max              : 55.053
    CV               : 0.267
    % uncovered      : 0.0 %

  COLLISIONS
    waypoints inside : 1295
============================================================
```

---

## Per-pass table

```
  Pass   Wpts  Spc_min  Spc_mean  Spc_max  Spd_OOT  Inside
  --------------------------------------------------------
     0    109      0.0      73.1    199.8      109     107
     1    109      0.0      76.5     85.0      109     107
     2    109     85.0      85.0     85.0      109     107
     3    109      0.1      83.1     85.0      109     107
     4    109     85.0      85.0     85.0      109     107
     5     78      0.1      73.6    199.2       78      76
     6     78      0.2      82.4     85.0       78      76
     7     78     85.0      85.0     85.0       78      76
     8     78      0.1      80.3     85.0       78      76
     9     78     85.0      85.0     85.0       78      76
    10     78      0.0      76.3    199.9       78      76
    11     78      0.0      74.4     85.0       78      76
    12     78     85.0      85.0     85.0       78      76
    13     78     85.0      85.0     85.0       78      76
    14     78     85.0      85.0     85.0       78      76
```

Regions found: `FRONT` (5,280 faces → 5 passes), `RIGHT` (560 faces → 5 passes),
`LEFT` (560 faces → 5 passes).

---

## What the numbers show about the current generator

**Standoff — 100% out of tolerance (1325/1325 samples, 250 mm off target).**
`generate_route` returns plane-intersection waypoints that lie directly on the
mesh surface; there is no standoff offset applied anywhere in the slicer or
stitcher. Every waypoint is at standoff ≈ 0 mm, which is 250 mm below the
250 mm target and therefore outside the ±5 mm tolerance for every single
sample.

**Spacing — 97.2% overlap, 0.3% OK.**
`classify_regions` splits the shell into three independent regions (FRONT,
RIGHT, LEFT) based on dominant normal axis. Each region is sliced by
`slice_region` at fixed axis-aligned z-intervals equal to `spray_width_mm`
(100 mm). Because all three regions span the same 0–400 mm z-range, each
produces passes at the same z-heights; passes from FRONT and RIGHT (and
FRONT and LEFT) are spatially adjacent at the region boundary and measure a
mean surface spacing of ≈ 85 mm — just below the 95 mm overlap threshold
(pitch × 0.95). Interior passes within a single region that only have
same-region neighbours show uniform 85 mm spacing, and the first and last
pass in each region have some neighbours from other regions at 0 mm distance
(exact overlap at the boundary). The net result is 97.2% of all spacing
measurements are flagged as overlap.

**Crossings — 15.**
Because FRONT, RIGHT, and LEFT each independently generate a full pass stack
over the same z-range, passes from different regions share the same z-height
and meet at the angular boundary between regions. `measure_route` detects a
crossing wherever two passes share a surface point within `0.05 × pitch`
(5 mm). With three region-boundary pairs (FRONT/RIGHT, FRONT/LEFT,
RIGHT/LEFT) and five z-levels each, up to 15 boundary intersections are
possible; all 15 are found.

**Collisions — 1295/1325 waypoints inside.**
Waypoints from the current generator lie on the mesh surface. After
`resample_arc` (linear interpolation between consecutive surface points), the
interpolated points fall on chords rather than arcs, landing slightly inside
the concave side of the curved shell. The normal-side collision test
(`dot(gun − nearest_surface, outward_normal) < 0`) flags these sub-surface
chord points as inside, producing a near-100% collision count even though the
gun has never been offset outward.

**Thickness — CV 0.267, 0% uncovered.**
The Gaussian kernel is evaluated against aim points (the surface projections
of the waypoints), not the waypoints themselves. Since the waypoints cover the
full surface in a dense grid (15 passes × 78–109 points each) the coverage is
complete. The CV of 0.267 is within the same range as the ideal-geometry test
cases in the Section 1 test suite, so the kernel itself is not the issue here;
the problem is that the waypoints this coverage is computed from are at 0 mm
standoff, not 250 mm.
