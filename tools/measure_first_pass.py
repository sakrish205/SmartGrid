"""Measure the new first pass against the Section 2 baseline.

Loads models/test_bumper.stl, runs get_outer_skin_faces + first_pass_line
(pitch=100, standoff=250 — same as the Section 2 baseline run), then calls
measure_route on the single resulting pass and prints standoff stats with an
explicit before/after comparison.
"""
from __future__ import annotations
import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.mesh.loader        import load_mesh
from app.path.outer_skin    import get_outer_skin_faces
from app.path.standoff_pass import first_pass_line
from app.path.metrics       import measure_route

PITCH    = 100.0
STANDOFF = 250.0
MESH     = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models', 'test_bumper.stl',
)

print(f"Loading {MESH}")
mesh = load_mesh(MESH)
print(f"  {len(mesh.faces):,} faces  watertight={mesh.is_watertight}")

print("Detecting outer skin...")
outer_ids = get_outer_skin_faces(mesh)
print(f"  {len(outer_ids):,} / {len(mesh.faces):,} faces are outer skin")

print("Generating first pass (new standoff method)...")
gun_pts = first_pass_line(
    mesh, outer_ids, STANDOFF, PITCH, direction='horizontal'
)
print(f"  {len(gun_pts)} waypoints")

print("Measuring...")
m     = measure_route(mesh, [gun_pts], pitch_mm=PITCH, standoff_mm=STANDOFF,
                       standoff_tol_mm=5.0)
sd    = m['passes'][0]['standoff']
co    = m['passes'][0]['collision']
n_wp  = m['passes'][0]['waypoint_count']

print()
print("=" * 60)
print("  STANDOFF COMPARISON  (Section 2 baseline vs new first pass)")
print("=" * 60)
print(f"  {'Metric':<28} {'Section 2':>10}  {'New pass':>10}")
print(f"  {'-'*52}")
print(f"  {'Standoff min (mm)':<28} {'~0':>10}  {sd['min_mm']:>10.1f}")
print(f"  {'Standoff mean (mm)':<28} {'~0':>10}  {sd['mean_mm']:>10.1f}")
print(f"  {'Standoff max (mm)':<28} {'~0':>10}  {sd['max_mm']:>10.1f}")
print(f"  {'Out-of-tolerance samples':<28} {'1325/1325':>10}  "
      f"{sd['count_out_of_tol']}/{n_wp:>4}")
print(f"  {'% out of tolerance':<28} {'100%':>10}  "
      f"{sd['count_out_of_tol'] / n_wp * 100:>9.1f}%")
print(f"  {'Waypoints inside mesh':<28} {'1295/1325':>10}  "
      f"{co['waypoints_inside']}/{n_wp:>4}")
print("=" * 60)
