"""End-to-end: region → route → verify points lie on mesh surface."""
import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import trimesh
import trimesh.proximity
from app.mesh.preprocessor import preprocess
from app.mesh.regions import classify_regions
from app.path.generator import generate_route
from app.path.face_grid_generator import generate_conform_route
from app.path.local_normals import interpolate_normals, build_tcp_frames


def test_generate_returns_paint_route(unit_box_mesh):
    data = preprocess(unit_box_mesh, "test")
    regions = classify_regions(data)
    rid = 'TOP'
    route = generate_route(data, rid, regions[rid], spray_width_mm=0.2)
    assert route.region_id == rid
    assert route.spacing_mm == 0.2


def test_conform_normals_and_tangents_stored():
    """Conform route must carry per-waypoint normals and tangents after Phase 2."""
    mesh = trimesh.creation.cylinder(radius=50.0, height=200.0, sections=64)
    from app.mesh.preprocessor import preprocess
    data = preprocess(mesh, "cyl")
    regions = classify_regions(data)
    face_ids = regions.get('FRONT', regions[next(iter(regions))])
    route = generate_conform_route(
        'FRONT', face_ids, mesh, up_axis=2,
        spray_width_mm=20.0, standoff_mm=10.0,
    )
    for p in route.passes:
        assert p.normals is not None, "normals must be populated"
        assert p.tangent is not None, "tangent must be populated"
        assert p.normals.shape == p.points.shape
        assert p.tangent.shape == p.points.shape


def test_tcp_frame_orthonormal():
    """N and T_corr must be unit vectors and orthogonal to each other."""
    # Use points on the +Z face of a box (flat surface → well-defined face normals)
    mesh = trimesh.creation.box(extents=[100.0, 100.0, 100.0])
    pts = np.array([
        [-30.0,  0.0, 50.0],
        [-10.0,  0.0, 50.0],
        [ 10.0,  0.0, 50.0],
        [ 30.0,  0.0, 50.0],
    ], dtype=float)
    normals = interpolate_normals(pts, mesh)
    tangents = build_tcp_frames(pts, normals)

    # unit length
    assert np.allclose(np.linalg.norm(normals, axis=1), 1.0, atol=1e-6)
    assert np.allclose(np.linalg.norm(tangents, axis=1), 1.0, atol=1e-6)
    # orthogonal
    dots = np.einsum('ij,ij->i', normals, tangents)
    assert np.allclose(dots, 0.0, atol=1e-6)


def test_standoff_within_tolerance_on_cylinder():
    """After standoff offset, all waypoints must sit within 5 mm of nominal standoff."""
    mesh = trimesh.creation.cylinder(radius=50.0, height=200.0, sections=64)
    from app.mesh.preprocessor import preprocess
    data = preprocess(mesh, "cyl")
    regions = classify_regions(data)
    face_ids = regions.get('FRONT', regions[next(iter(regions))])
    standoff = 10.0
    route = generate_conform_route(
        'FRONT', face_ids, mesh, up_axis=2,
        spray_width_mm=20.0, standoff_mm=standoff,
    )
    for p in route.passes:
        _, dists, _ = trimesh.proximity.closest_point(mesh, p.points)
        err = np.abs(dists - standoff)
        assert err.max() < 5.0, f"standoff deviation {err.max():.2f} mm exceeds 5 mm tolerance"
