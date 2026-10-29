"""Unit tests for the Export Mesh mode (app/export/mesh_export.py)."""
from __future__ import annotations
import numpy as np
import pytest
import trimesh

from app.path.path_model import PaintRoute, PaintPass, Connection
from app.export.mesh_export import export_route_mesh


@pytest.fixture
def sample_route() -> list[PaintRoute]:
    """Synthetic route with 2 passes and 1 connection."""
    p1_pts = np.array([
        [100.0, 200.0, 300.0],
        [150.0, 200.0, 300.0],
        [200.0, 200.0, 300.0],
    ])
    p2_pts = np.array([
        [200.0, 250.0, 300.0],
        [150.0, 250.0, 300.0],
        [100.0, 250.0, 300.0],
    ])
    conn_pts = np.array([
        [200.0, 200.0, 350.0],
        [200.0, 250.0, 350.0],
    ])

    route = PaintRoute(
        region_id='FRONT',
        passes=[
            PaintPass(id=0, region_id='FRONT', direction='horizontal', points=p1_pts,
                      is_forward=True, sub_index=0, slice_position=100.0),
            PaintPass(id=1, region_id='FRONT', direction='horizontal', points=p2_pts,
                      is_forward=False, sub_index=0, slice_position=150.0),
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


def test_export_route_mesh_writes_stl(tmp_path, sample_route):
    out = tmp_path / "toolpath.stl"
    export_route_mesh(sample_route, str(out))

    assert out.exists()
    mesh = trimesh.load(str(out))
    assert len(mesh.faces) > 0
    assert mesh.is_watertight


def test_export_route_mesh_raises_on_empty_routes():
    with pytest.raises(ValueError):
        export_route_mesh([], "unused.stl")
