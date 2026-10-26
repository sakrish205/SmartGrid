"""Unit tests for SmartGrid OLP Native Robot Exporters.

Tests ABB RAPID (.mod), Fanuc Karel/TP (.LS), Dürr EcoScreen (.tid/.tip),
and CAD 3D wireframe curve (.dxf) generation.
"""
from __future__ import annotations
import re
import numpy as np
import pytest

from app.path.path_model import PaintRoute, PaintPass, Connection, GenerationParams
from app.export.olp_export import (
    export_abb_rapid,
    export_fanuc_ls,
    export_durr,
    export_cad_dxf,
)


@pytest.fixture
def sample_route() -> list[PaintRoute]:
    """Synthetic route with 2 passes and 1 connection."""
    p1_pts = np.array([
        [100.0, 200.0, 300.0],
        [150.0, 200.0, 300.0],
        [200.0, 200.0, 300.0],
    ])
    p1_normals = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    ])
    p1_tangents = np.array([
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    ])

    p2_pts = np.array([
        [200.0, 250.0, 300.0],
        [150.0, 250.0, 300.0],
        [100.0, 250.0, 300.0],
    ])
    p2_normals = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    ])
    p2_tangents = np.array([
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    ])

    conn_pts = np.array([
        [200.0, 200.0, 350.0],
        [200.0, 250.0, 350.0],
    ])

    route = PaintRoute(
        region_id='FRONT',
        passes=[
            PaintPass(id=0, region_id='FRONT', direction='horizontal', points=p1_pts,
                      is_forward=True, sub_index=0, slice_position=100.0,
                      normals=p1_normals, tangent=p1_tangents),
            PaintPass(id=1, region_id='FRONT', direction='horizontal', points=p2_pts,
                      is_forward=False, sub_index=0, slice_position=150.0,
                      normals=p2_normals, tangent=p2_tangents),
        ],
        connections=[
            Connection(id=0, from_pass_id=0, to_pass_id=1, points=conn_pts, is_air_move=True),
        ],
        unit='mm',
        spacing_mm=50.0,
        total_passes=2,
        total_length_mm=200.0,
        spray_normal=np.array([0.0, 0.0, 1.0]),
    )
    return [route]


def test_abb_rapid_export(tmp_path, sample_route):
    out = tmp_path / "test_prog.mod"
    export_abb_rapid(sample_route, str(out))

    assert out.exists()
    content = out.read_text(encoding="utf-8")

    assert "MODULE M1" in content
    assert "ENDMODULE" in content
    assert "PERS wobjdata wobjPart" in content
    assert "PROC mvPath()" in content
    assert "PaintL" in content
    assert "MoveL" in content

    # Verify unit quaternions from robtarget declarations
    q_pattern = re.compile(r'robtarget\s+\w+:=[^\]]+\],\[([-\d\.]+),([-\d\.]+),([-\d\.]+),([-\d\.]+)\]')
    matches = q_pattern.findall(content)
    assert len(matches) >= 8  # 3 (p1) + 3 (p2) + 2 (conn)
    for m in matches:
        q = [float(v) for v in m]
        norm = np.linalg.norm(q)
        assert np.isclose(norm, 1.0, atol=1e-3), f"Quaternion not normalized: {q}"


def test_fanuc_ls_export(tmp_path, sample_route):
    out = tmp_path / "test_prog.ls"
    export_fanuc_ls(sample_route, str(out))

    assert out.exists()
    content = out.read_text(encoding="utf-8")

    assert "/PROG" in content
    assert "/ATTR" in content
    assert "/APPL" in content
    assert "/MN" in content
    assert "/POS" in content

    assert "Gun=ON" in content
    assert "Gun=OFF" in content
    assert "Preset[1]" in content

    # Verify POS entries
    assert "UF : 1, UT : 1" in content
    assert "CONFIG : 'N U T, 0, 0, 0'" in content

    # Check Euler angle numbers
    wpr_pattern = re.compile(r'W =\s*([-\d\.]+)\s*deg,\s*P =\s*([-\d\.]+)\s*deg,\s*R =\s*([-\d\.]+)\s*deg')
    wprs = wpr_pattern.findall(content)
    assert len(wprs) >= 8
    for w, p, r in wprs:
        for val in (float(w), float(p), float(r)):
            assert -180.01 <= val <= 180.01, f"Euler angle out of range: {val}"


def test_durr_export(tmp_path, sample_route):
    out = tmp_path / "Aleta_Ptras.tid"
    export_durr(sample_route, str(out))

    assert out.exists()
    tip_file = tmp_path / "Aleta_Ptras.tip"
    assert tip_file.exists()

    tid_content = out.read_text(encoding="utf-8")
    tip_content = tip_file.read_text(encoding="utf-8")

    # .tid assertions
    assert "// POINT" in tid_content
    assert "// TRIGGERFRAME" in tid_content
    assert "P1 : POINT := (Object," in tid_content

    # .tip assertions
    assert 'RSELECT("R11")' in tip_content
    assert "SETTOOL(ECOBELL3E_200)" in tip_content
    assert "GUN(GUN1, GunOn" in tip_content
    assert "GUN(GUN1, GunOff" in tip_content
    assert "LIN(P1)" in tip_content

    # Every PT{n} referenced in the .tip must be declared as a POINT in the .tid
    # (the real controller fails to load a program with an undefined point ref).
    referenced_pts = set(re.findall(r'\bPT(\d+)\b', tip_content))
    declared_pts = set(re.findall(r'^PT(\d+) : POINT', tid_content, re.MULTILINE))
    assert referenced_pts, "expected at least one PT reference in the .tip"
    assert referenced_pts <= declared_pts, (
        f"PT refs missing a POINT declaration: {referenced_pts - declared_pts}"
    )

    # SETBRUSH and the following GUN(...GunOn...) must use distinct PT/TR pairs,
    # matching the reference file's one-index-per-directive convention.
    setbrush_idx = re.search(r'SETBRUSH\(GUN1, \d+, PT(\d+), TR(\d+)\)', tip_content)
    gunon_idx = re.search(r'GUN\(GUN1, GunOn, PT(\d+), TR(\d+)\)', tip_content)
    assert setbrush_idx and gunon_idx
    assert setbrush_idx.group(1) != gunon_idx.group(1)

    # GunOff must come after the LIN that reaches the final paint point of a pass,
    # not before it (gun must not shut off before finishing the last stroke).
    gunoff_pos = tip_content.index('GunOff')
    lin_p3_pos = tip_content.index('LIN(P3)')
    assert lin_p3_pos < gunoff_pos, "GunOff fired before reaching the last pass point"


def test_cad_dxf_export(tmp_path, sample_route):
    out = tmp_path / "toolpath.dxf"
    export_cad_dxf(sample_route, str(out))

    assert out.exists()
    content = out.read_text(encoding="utf-8")

    assert "SECTION" in content
    assert "HEADER" in content
    assert "TABLES" in content
    assert "PAINT_PASSES" in content
    assert "AIR_CONNECTORS" in content
    assert "POLYLINE" in content
    assert "VERTEX" in content
    assert "SEQEND" in content
    assert "EOF" in content
