"""Tests for app.path.metrics — 6 cases on a synthetic cylindrical arc shell.

Mesh: open cylindrical arc, radius=400 mm, height=600 mm, 120° arc,
      angular step=2°, height step=10 mm, outward normals.
Passes: horizontal arcs at the outer gun radius (400 + standoff) mm.
Spacing: 100 mm pitch, first pass at 50 mm (pitch/2).

Run with:  pytest tests/test_metrics.py -v -s
"""
from __future__ import annotations
import numpy as np
import pytest
import trimesh

from app.path.metrics import measure_route


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_arc_shell(
    radius: float = 400.0,
    height: float = 600.0,
    arc_deg: float = 120.0,
    ang_step_deg: float = 2.0,
    h_step: float = 10.0,
) -> trimesh.Trimesh:
    """Open cylindrical arc shell with outward normals.

    Angular and height grids are exactly matched to the default base_angles
    and pass heights, so closest_point lands on mesh vertices and standoff
    equals (R_GUN - radius) with negligible facet error.
    """
    angles  = np.arange(0.0, np.radians(arc_deg) + 1e-9, np.radians(ang_step_deg))
    heights = np.arange(0.0, height + 1e-9, h_step)
    na, nh  = len(angles), len(heights)

    AA, HH = np.meshgrid(angles, heights, indexing='ij')
    verts  = np.column_stack([
        radius * np.cos(AA.ravel()),
        radius * np.sin(AA.ravel()),
        HH.ravel(),
    ])

    faces = []
    for i in range(na - 1):
        for j in range(nh - 1):
            v00 = i * nh + j
            v10 = (i + 1) * nh + j
            v01 = i * nh + (j + 1)
            v11 = (i + 1) * nh + (j + 1)
            # winding order produces outward-facing normals
            faces.extend([[v00, v10, v11], [v00, v11, v01]])

    mesh = trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=False)
    trimesh.repair.fix_normals(mesh)
    return mesh


def _arc_passes(r_gun: float, heights, angles: np.ndarray) -> list:
    return [
        np.column_stack([
            r_gun * np.cos(angles),
            r_gun * np.sin(angles),
            np.full(len(angles), float(h)),
        ])
        for h in heights
    ]


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def arc_shell() -> trimesh.Trimesh:
    return _make_arc_shell()


@pytest.fixture(scope='module')
def base_angles() -> np.ndarray:
    return np.arange(0.0, np.radians(120.0) + 1e-9, np.radians(2.0))


PITCH    = 100.0          # mm
STANDOFF = 250.0          # mm
R_SURF   = 400.0          # mm  — cylinder radius
R_GUN    = R_SURF + STANDOFF   # 650 mm


# ---------------------------------------------------------------------------
# Case 1 — Correct: ideal spacing, correct standoff
# ---------------------------------------------------------------------------

def test_correct(arc_shell, base_angles):
    """Ideal passes at pitch=100 mm, standoff=250 mm.

    Expects:
      - pct_ok > 95 %
      - count_out_of_tol == 0
      - no crossings
      - no waypoints inside mesh
      - thickness CV < 0.30  (first run measured 0.183; capped conservatively)
      - pct_uncovered < 5 %  (all arc vertices within 3*sigma of a pass)
    """
    pass_heights = np.arange(PITCH / 2.0, 600.1, PITCH)   # [50,150,250,350,450,550]
    passes       = _arc_passes(R_GUN, pass_heights, base_angles)

    m = measure_route(arc_shell, passes, PITCH, STANDOFF, standoff_tol_mm=5.0)

    g   = m['global']
    thi = m['thickness']
    print(f"\n[correct] pct_ok={g['pct_ok']:.1f}%  "
          f"pct_overlap={g['pct_overlap']:.1f}%  "
          f"pct_gap={g['pct_gap']:.1f}%")
    print(f"[correct] count_out_of_tol={g['count_out_of_tol']}  "
          f"crossings={len(m['crossings'])}  "
          f"inside={g['total_waypoints_inside']}")
    print(f"[correct] thickness CV={thi['cv']:.3f}  "
          f"pct_uncovered={thi['pct_uncovered']:.1f}%")
    for i, r in enumerate(m['passes']):
        sd = r['standoff']
        print(f"  pass {i}: standoff {sd['min_mm']:.1f}–{sd['max_mm']:.1f} mm  "
              f"OOT={sd['count_out_of_tol']}")

    assert g['pct_ok']               >  95.0, f"pct_ok={g['pct_ok']:.1f}%"
    assert g['count_out_of_tol']     ==    0, f"out_of_tol={g['count_out_of_tol']}"
    assert len(m['crossings'])       ==    0, f"crossings={m['crossings']}"
    assert g['total_waypoints_inside'] == 0
    assert thi['cv']                 <   0.30, f"thickness CV={thi['cv']:.3f}"
    assert thi['pct_uncovered']      <   5.0,  f"pct_uncovered={thi['pct_uncovered']:.1f}%"
    for i, r in enumerate(m['passes']):
        assert r['collision']['method'] == 'normal_side', f"pass {i}: expected normal_side"
        assert r['collision']['waypoints_inside'] == 0,   f"pass {i}: should be outside"


# ---------------------------------------------------------------------------
# Case 2 — Overlap: one pass shifted 80 mm from its neighbour
# ---------------------------------------------------------------------------

def test_overlap(arc_shell, base_angles):
    """Pass[1] shifted to 80 mm from pass[0] — spacing < 95 mm → OVERLAP."""
    heights    = np.arange(PITCH / 2.0, 600.1, PITCH).tolist()
    heights[1] = heights[0] + 80.0          # was 150 mm, now 130 mm
    passes = _arc_passes(R_GUN, heights, base_angles)

    m = measure_route(arc_shell, passes, PITCH, STANDOFF, standoff_tol_mm=5.0)

    overlap_pcts = [
        r['spacing']['pct_overlap']
        for r in m['passes']
        if r['spacing']['pct_overlap'] is not None
    ]
    max_ov = max(overlap_pcts) if overlap_pcts else 0.0
    print(f"\n[overlap] per-pass pct_overlap: {[f'{p:.1f}' for p in overlap_pcts]}")

    assert any(p > 0 for p in overlap_pcts), \
        f"expected OVERLAP but pcts={overlap_pcts}"
    assert all(r['collision']['method'] == 'normal_side' for r in m['passes'])
    assert all(r['collision']['waypoints_inside'] == 0   for r in m['passes'])


# ---------------------------------------------------------------------------
# Case 3 — Gap: one pass shifted 120 mm from its neighbour
# ---------------------------------------------------------------------------

def test_gap(arc_shell, base_angles):
    """Pass[1] shifted to 120 mm from pass[0] — spacing > 105 mm → GAP."""
    heights    = np.arange(PITCH / 2.0, 600.1, PITCH).tolist()
    heights[1] = heights[0] + 120.0         # was 150 mm, now 170 mm
    passes = _arc_passes(R_GUN, heights, base_angles)

    m = measure_route(arc_shell, passes, PITCH, STANDOFF, standoff_tol_mm=5.0)

    gap_pcts = [
        r['spacing']['pct_gap']
        for r in m['passes']
        if r['spacing']['pct_gap'] is not None
    ]
    print(f"\n[gap] per-pass pct_gap: {[f'{p:.1f}' for p in gap_pcts]}")

    assert any(p > 0 for p in gap_pcts), \
        f"expected GAP but pcts={gap_pcts}"
    assert all(r['collision']['method'] == 'normal_side' for r in m['passes'])
    assert all(r['collision']['waypoints_inside'] == 0   for r in m['passes'])


# ---------------------------------------------------------------------------
# Case 4 — Crossing: two diagonal passes that swap z-coordinates
# ---------------------------------------------------------------------------

def test_crossing(arc_shell, base_angles):
    """Pass A: z decreases 150→50; Pass B: z increases 50→150.

    Both are at the same angle/radius; at the midpoint they share the same
    surface point (distance ≈ 0) → CROSSING flagged with correct pass IDs.
    """
    n   = len(base_angles)
    z_a = np.linspace(150.0, 50.0, n)
    z_b = np.linspace(50.0, 150.0, n)

    pass_a = np.column_stack([R_GUN * np.cos(base_angles),
                               R_GUN * np.sin(base_angles), z_a])
    pass_b = np.column_stack([R_GUN * np.cos(base_angles),
                               R_GUN * np.sin(base_angles), z_b])

    m = measure_route(arc_shell, [pass_a, pass_b], PITCH, STANDOFF,
                      standoff_tol_mm=5.0)

    print(f"\n[crossing] crossings found: {len(m['crossings'])}")
    for c in m['crossings'][:3]:
        print(f"  passes ({c['pass_a']},{c['pass_b']})  "
              f"dist={c['dist_mm']:.2f} mm  loc={[f'{x:.0f}' for x in c['location']]}")

    assert len(m['crossings']) > 0, "expected at least one CROSSING"
    ids_seen = {(c['pass_a'], c['pass_b']) for c in m['crossings']}
    ids_seen |= {(c['pass_b'], c['pass_a']) for c in m['crossings']}
    assert (0, 1) in ids_seen or (1, 0) in ids_seen, \
        f"crossing should involve passes 0 and 1, got {ids_seen}"


# ---------------------------------------------------------------------------
# Case 5 — Standoff violation: some waypoints pushed out of ±5 mm tolerance
# ---------------------------------------------------------------------------

def test_standoff_violation(arc_shell, base_angles):
    """10 waypoints on pass[2] scaled to 230 mm or 280 mm standoff.

    Expected: pass 2 count_out_of_tol > 0; all other passes remain 0.
    """
    heights = np.arange(PITCH / 2.0, 600.1, PITCH)
    passes  = _arc_passes(R_GUN, heights, base_angles)

    mid = len(base_angles) // 2
    pts = passes[2].copy()

    # 5 points at standoff 230 mm — 20 mm below nominal, outside ±5 mm tol
    scale_in = (R_SURF + 230.0) / R_GUN   # 630/650
    pts[mid:mid + 5, :2] *= scale_in

    # 5 more points at standoff 280 mm — 30 mm above nominal, outside ±5 mm tol
    scale_out = (R_SURF + 280.0) / R_GUN  # 680/650
    pts[mid + 5:mid + 10, :2] *= scale_out

    passes[2] = pts

    m = measure_route(arc_shell, passes, PITCH, STANDOFF, standoff_tol_mm=5.0)

    oot_p2 = m['passes'][2]['standoff']['count_out_of_tol']
    print(f"\n[standoff] pass 2 out_of_tol={oot_p2}")
    for i, r in enumerate(m['passes']):
        sd = r['standoff']
        print(f"  pass {i}: standoff {sd['min_mm']:.1f}–{sd['max_mm']:.1f} mm  "
              f"OOT={sd['count_out_of_tol']}")

    assert oot_p2 > 0, f"expected OUT_OF_TOLERANCE on pass 2, got {oot_p2}"
    for i, r in enumerate(m['passes']):
        if i != 2:
            oot = r['standoff']['count_out_of_tol']
            assert oot == 0, f"unexpected OUT_OF_TOLERANCE on pass {i}: {oot}"
    assert all(r['collision']['method'] == 'normal_side' for r in m['passes'])
    assert all(r['collision']['waypoints_inside'] == 0   for r in m['passes'])


# ---------------------------------------------------------------------------
# Case 6 — Collision: waypoints pushed inside a watertight box
# ---------------------------------------------------------------------------

def test_collision(base_angles):
    """Pass through interior of a 200 mm box — collision count > 0.

    Uses trimesh.creation.box (watertight) so mesh.contains is reliable.
    The arc shell used by other tests is open and not suitable for collision
    detection via ray casting.
    """
    box = trimesh.creation.box(extents=[200.0, 200.0, 200.0])
    assert box.is_watertight, "box fixture must be watertight"

    n      = len(base_angles)
    z_line = np.linspace(-80.0, 80.0, n)

    # pass_out: x=150 mm — outside the box (|x| > 100 mm half-extent)
    pass_out = np.column_stack([np.full(n, 150.0), np.zeros(n), z_line])
    # pass_in:  x=0 mm   — passes through centre of the box
    pass_in  = np.column_stack([np.zeros(n), np.zeros(n), z_line])

    m = measure_route(box, [pass_out, pass_in], pitch_mm=50.0, standoff_mm=50.0)

    n_out = m['passes'][0]['collision']['waypoints_inside']
    n_in  = m['passes'][1]['collision']['waypoints_inside']
    print(f"\n[collision] outer pass inside={n_out}  inner pass inside={n_in}")

    assert n_in  > 0, f"expected inner pass inside the box, got {n_in}"
    assert n_out == 0, f"outer pass should not be inside the box, got {n_out}"
    assert m['passes'][0]['collision']['method'] == 'contains'
    assert m['passes'][1]['collision']['method'] == 'contains'


# ---------------------------------------------------------------------------
# Case 7 — Open-shell collision via normal-side fallback
# ---------------------------------------------------------------------------

def test_collision_open_shell(arc_shell, base_angles):
    """Normal-side fallback detects gun inside an open (non-watertight) surface.

    pass_out at r=R_GUN (650 mm): dot(gun-surface, outward_normal) > 0 → outside.
    pass_in  at r=R_SURF-50 (350 mm): dot(gun-surface, outward_normal) < 0 → inside.
    """
    assert not arc_shell.is_watertight, "arc_shell must be open for this test"

    z = np.full(len(base_angles), 300.0)

    pass_out = np.column_stack([R_GUN * np.cos(base_angles),
                                 R_GUN * np.sin(base_angles), z])
    r_in = R_SURF - 50.0   # 350 mm — inside the shell
    pass_in  = np.column_stack([r_in * np.cos(base_angles),
                                 r_in * np.sin(base_angles), z])

    m = measure_route(arc_shell, [pass_out, pass_in],
                      pitch_mm=PITCH, standoff_mm=STANDOFF, standoff_tol_mm=5.0)

    c_out = m['passes'][0]['collision']
    c_in  = m['passes'][1]['collision']
    print(f"\n[open_collision] outside: method={c_out['method']} inside={c_out['waypoints_inside']}")
    print(f"[open_collision] inside:  method={c_in['method']}  inside={c_in['waypoints_inside']}")

    assert c_out['method'] == 'normal_side'
    assert c_in['method']  == 'normal_side'
    assert c_out['waypoints_inside'] == 0, f"outer pass should be outside, got {c_out['waypoints_inside']}"
    assert c_in['waypoints_inside']  >  0, f"inner pass should be inside, got {c_in['waypoints_inside']}"
