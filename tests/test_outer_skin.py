"""Tests for app.path.outer_skin — get_outer_skin_faces.

Run with:  pytest tests/test_outer_skin.py -v -s
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path.outer_skin import get_outer_skin_faces


def _make_arc_shell(
    radius: float = 400.0,
    height: float = 600.0,
    arc_deg: float = 120.0,
    ang_step_deg: float = 2.0,
    h_step: float = 10.0,
) -> trimesh.Trimesh:
    """Open cylindrical arc shell with outward normals (same geometry as test_metrics)."""
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


# ---------------------------------------------------------------------------
# Case 1 — Single open shell: all faces are outer skin
# ---------------------------------------------------------------------------

def test_single_layer_all_outer():
    """Open arc shell with no occluding geometry — every face must be returned."""
    shell    = _make_arc_shell()
    face_ids = get_outer_skin_faces(shell)
    print(f"\n[outer_skin] single layer: {len(face_ids)} / {len(shell.faces)} outer")
    assert len(face_ids) == len(shell.faces), (
        f"expected {len(shell.faces)} outer faces, got {len(face_ids)}"
    )


# ---------------------------------------------------------------------------
# Case 2 — Two-layer shell: inner layer (r=350) is occluded by outer (r=400)
# ---------------------------------------------------------------------------

def test_inner_layer_excluded():
    """Inner layer faces are occluded by the outer layer and must be excluded.

    outer_shell faces occupy indices [0, n_outer).
    inner_shell faces occupy indices [n_outer, 2*n_outer).
    get_outer_skin_faces should return exactly [0, n_outer).
    """
    outer_shell = _make_arc_shell(radius=400.0)
    inner_shell = _make_arc_shell(radius=350.0)   # 50 mm inward
    n_outer     = len(outer_shell.faces)

    combined  = trimesh.util.concatenate([outer_shell, inner_shell])
    outer_ids = get_outer_skin_faces(combined)

    print(f"\n[outer_skin] two-layer: combined={len(combined.faces)} faces  "
          f"outer_ids={len(outer_ids)}  expected={n_outer}")

    assert len(outer_ids) == n_outer, (
        f"expected {n_outer} outer faces, got {len(outer_ids)}"
    )
    assert set(outer_ids.tolist()) == set(range(n_outer)), (
        "outer_ids should be exactly the first n_outer face indices"
    )
