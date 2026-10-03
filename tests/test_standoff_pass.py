"""Tests for app.path.standoff_pass — first_pass_line.

Run with:  pytest tests/test_standoff_pass.py -v -s
"""
from __future__ import annotations
import numpy as np
import pytest
import trimesh

from app.path.outer_skin    import get_outer_skin_faces
from app.path.standoff_pass import first_pass_line

STANDOFF = 250.0
PITCH    = 100.0


def _make_arc_shell(
    radius: float = 400.0,
    height: float = 600.0,
    arc_deg: float = 120.0,
    ang_step_deg: float = 2.0,
    h_step: float = 10.0,
) -> trimesh.Trimesh:
    angles  = np.arange(0.0, np.radians(arc_deg) + 1e-9, np.radians(ang_step_deg))
    heights = np.arange(0.0, height + 1e-9, h_step)
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


@pytest.fixture(scope='module')
def arc_shell() -> trimesh.Trimesh:
    return _make_arc_shell()


# ---------------------------------------------------------------------------
# Case 1 — Standoff must be within 1 mm of target for every returned point
# ---------------------------------------------------------------------------

def test_standoff_correct(arc_shell):
    """All gun points within 1 mm of the 250 mm target standoff.

    Section 2 baseline had 0 mm standoff (100% out of tolerance).
    This proves first_pass_line fixes that.
    """
    outer = get_outer_skin_faces(arc_shell)
    pts   = first_pass_line(arc_shell, outer, STANDOFF, PITCH, 'horizontal')

    _, dists, _ = trimesh.proximity.closest_point(arc_shell, pts)
    err = np.abs(dists - STANDOFF)
    print(f"\n[standoff_pass] n_pts={len(pts)}  "
          f"standoff min={dists.min():.2f}  mean={dists.mean():.2f}  "
          f"max={dists.max():.2f}  max_err={err.max():.4f}")

    assert err.max() < 1.0, (
        f"standoff error up to {err.max():.2f} mm (max allowed 1 mm)\n"
        f"  target={STANDOFF}  actual range [{dists.min():.2f}, {dists.max():.2f}]"
    )


# ---------------------------------------------------------------------------
# Case 2 — Returned line must be non-degenerate and finite
# ---------------------------------------------------------------------------

def test_line_valid(arc_shell):
    """Returned array has >2 points and no NaN / inf."""
    outer = get_outer_skin_faces(arc_shell)
    pts   = first_pass_line(arc_shell, outer, STANDOFF, PITCH, 'horizontal')

    assert len(pts) > 2,             f"expected >2 points, got {len(pts)}"
    assert np.all(np.isfinite(pts)), "NaN or inf found in returned gun points"
