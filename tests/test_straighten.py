"""Tests for app.path.straighten — straighten_pass.

Run with:  pytest tests/test_straighten.py -v -s
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.outer_skin    import get_outer_skin_faces
from app.path.standoff_pass import first_pass_line
from app.path.straighten    import straighten_pass

STANDOFF = 250.0
PITCH    = 100.0
TOL      = 5.0


def _make_arc_shell(radius: float = 400.0) -> trimesh.Trimesh:
    angles  = np.arange(0.0, np.radians(120.0) + 1e-9, np.radians(2.0))
    heights = np.arange(0.0, 600.0 + 1e-9, 10.0)
    na, nh  = len(angles), len(heights)
    AA, HH  = np.meshgrid(angles, heights, indexing='ij')
    verts   = np.column_stack([
        radius * np.cos(AA.ravel()),
        radius * np.sin(AA.ravel()),
        HH.ravel(),
    ])
    faces = []
    for i in range(na - 1):
        for j in range(nh - 1):
            v00 = i * nh + j;       v10 = (i + 1) * nh + j
            v01 = i * nh + (j + 1); v11 = (i + 1) * nh + (j + 1)
            faces.extend([[v00, v10, v11], [v00, v11, v01]])
    mesh = trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=False)
    trimesh.repair.fix_normals(mesh)
    return mesh


# ---------------------------------------------------------------------------
# Case 1 — flat line → 2 points
# ---------------------------------------------------------------------------

def test_flat_line_returns_two_points():
    """A perfectly flat line at constant standoff collapses to two endpoints."""
    # Build a flat plane mesh (z=0)
    verts = np.array([[0,0,0],[1000,0,0],[1000,1000,0],[0,1000,0]], dtype=float)
    faces = np.array([[0,1,2],[0,2,3]])
    plane = trimesh.Trimesh(vertices=verts, faces=faces, process=False)

    # Points at constant height = 250 mm above the plane
    n = 20
    pts = np.zeros((n, 3))
    pts[:, 0] = np.linspace(100.0, 900.0, n)
    pts[:, 1] = 500.0
    pts[:, 2] = 250.0   # exact standoff

    result = straighten_pass(plane, pts, standoff_mm=250.0, standoff_tol_mm=5.0)
    print(f"\n[straighten] flat line: {len(pts)} -> {len(result)} points")
    assert len(result) == 2, f"flat line should collapse to 2 pts, got {len(result)}"
    assert np.allclose(result[0], pts[0])
    assert np.allclose(result[-1], pts[-1])


# ---------------------------------------------------------------------------
# Case 2 — arc_shell first_pass: M < N/5, standoff within ±5 mm, endpoints OK
# ---------------------------------------------------------------------------

def test_arc_shell_reduces_points():
    """Straightened pass on arc shell: M < N/5, max standoff error within ±5 mm."""
    shell    = _make_arc_shell()
    outer    = get_outer_skin_faces(shell)
    gun_pts  = first_pass_line(shell, outer, STANDOFF, PITCH, 'horizontal')
    corners  = straighten_pass(shell, gun_pts, STANDOFF, TOL)

    n = len(gun_pts)
    m = len(corners)
    print(f"\n[straighten] arc_shell tol={TOL}: {n} -> {m} points  (threshold N/5={n/5:.1f})")

    assert m < n / 5, f"expected M < N/5={n/5:.1f}, got M={m}"

    # Standoff of straight-segment samples must stay within tolerance
    _, dists, _ = trimesh.proximity.closest_point(shell, corners)
    errs = np.abs(dists - STANDOFF)
    print(f"  standoff range [{dists.min():.1f}, {dists.max():.1f}]  max_err={errs.max():.2f}")
    assert errs.max() <= TOL, f"standoff error {errs.max():.2f} > tol {TOL}"

    # Endpoints preserved
    assert np.allclose(corners[0], gun_pts[0])
    assert np.allclose(corners[-1], gun_pts[-1])


# ---------------------------------------------------------------------------
# Case 3 — tighter tolerance gives more segments than case 2
# ---------------------------------------------------------------------------

def test_tighter_tolerance_more_segments():
    """±2 mm tolerance should produce more corners than ±5 mm."""
    shell   = _make_arc_shell()
    outer   = get_outer_skin_faces(shell)
    gun_pts = first_pass_line(shell, outer, STANDOFF, PITCH, 'horizontal')

    c5 = straighten_pass(shell, gun_pts, STANDOFF, standoff_tol_mm=5.0)
    c2 = straighten_pass(shell, gun_pts, STANDOFF, standoff_tol_mm=2.0)
    print(f"\n[straighten] tol=5mm -> {len(c5)} pts  tol=2mm -> {len(c2)} pts")
    assert len(c2) > len(c5), (
        f"expected tighter tol to produce more corners: "
        f"tol=2mm gave {len(c2)}, tol=5mm gave {len(c5)}"
    )


# ---------------------------------------------------------------------------
# Case 4 — degenerate inputs (<3 points) returned unchanged
# ---------------------------------------------------------------------------

def test_degenerate_returned_unchanged():
    """Inputs with <3 points must be returned as-is (no crash, no modification)."""
    shell = _make_arc_shell()
    for n in (0, 1, 2):
        pts = np.zeros((n, 3)) if n > 0 else np.empty((0, 3))
        if n == 2:
            pts[0] = [0, 0, 250]; pts[1] = [100, 0, 250]
        result = straighten_pass(shell, pts, STANDOFF, TOL)
        assert len(result) == n, f"n={n}: expected unchanged, got {len(result)}"
        if n > 0:
            assert np.allclose(result, pts)


# ---------------------------------------------------------------------------
# Case 5 — no collisions on arc shell result (all segments in free space)
# ---------------------------------------------------------------------------

def test_no_collisions_on_result():
    """All corners of the straightened pass must lie outside the mesh (dot > 0)."""
    shell   = _make_arc_shell()
    outer   = get_outer_skin_faces(shell)
    gun_pts = first_pass_line(shell, outer, STANDOFF, PITCH, 'horizontal')
    corners = straighten_pass(shell, gun_pts, STANDOFF, TOL)

    aim_pts, _, tri_ids = trimesh.proximity.closest_point(shell, corners)
    normals = shell.face_normals[tri_ids]
    diff    = corners - aim_pts
    dotprod = np.einsum('ij,ij->i', diff, normals)
    n_inside = int((dotprod < 0).sum())

    print(f"\n[straighten] collisions: {n_inside} / {len(corners)}")
    assert n_inside == 0, f"{n_inside} corner(s) landed inside mesh"
