"""Before/after comparison: first_pass_line vs straighten_pass.

Loads models/test_bumper.stl, runs the full new pipeline, prints a
two-column table comparing waypoint count and standoff stats.
"""
from __future__ import annotations
import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.mesh.loader        import load_mesh
from app.path.outer_skin    import get_outer_skin_faces
from app.path.standoff_pass import first_pass_line
from app.path.straighten    import straighten_pass
import trimesh
import numpy as np

PITCH    = 100.0
STANDOFF = 250.0
TOL      = 5.0
MESH     = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models', 'test_bumper.stl',
)

print(f"Loading {MESH}")
mesh = load_mesh(MESH)
print(f"  {len(mesh.faces):,} faces  watertight={mesh.is_watertight}")

print("Outer skin detection...")
outer_ids = get_outer_skin_faces(mesh)
print(f"  {len(outer_ids):,} / {len(mesh.faces):,} outer")

print("First pass (before straightening)...")
gun_pts = first_pass_line(mesh, outer_ids, STANDOFF, PITCH, 'horizontal')
_, d_before, _ = trimesh.proximity.closest_point(mesh, gun_pts)

print(f"Straightening (tol={TOL} mm)...")
corners = straighten_pass(mesh, gun_pts, STANDOFF, standoff_tol_mm=TOL)
_, d_after, _ = trimesh.proximity.closest_point(mesh, corners)

def _oot(dists):
    return int(np.sum((dists < STANDOFF - TOL) | (dists > STANDOFF + TOL)))

print()
print("=" * 68)
print("  STRAIGHTENING COMPARISON")
print("=" * 68)
print(f"  {'Metric':<32} {'Before':>14}  {'After':>14}")
print(f"  {'-'*62}")
print(f"  {'Waypoints':<32} {len(gun_pts):>14}  {len(corners):>14}")
print(f"  {'Standoff min (mm)':<32} {d_before.min():>14.1f}  {d_after.min():>14.1f}")
print(f"  {'Standoff mean (mm)':<32} {d_before.mean():>14.1f}  {d_after.mean():>14.1f}")
print(f"  {'Standoff max (mm)':<32} {d_before.max():>14.1f}  {d_after.max():>14.1f}")
print(f"  {'Out-of-tolerance (+-{:.0f}mm)'.format(TOL):<32} "
      f"{_oot(d_before):>12}/{len(gun_pts)}"
      f"  {_oot(d_after):>12}/{len(corners)}")
print(f"  {'Reduction':<32} {'—':>14}  "
      f"{(1 - len(corners)/len(gun_pts))*100:>12.0f}%")
print("=" * 68)
