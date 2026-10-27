"""Mesh Flow Phase 4 exit-gate tests: F25/F32-F35 orientation utilities."""
from __future__ import annotations
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.export.olp_export import _get_tcp_orientation
from app.path.orientation_frames import angular_difference, area_weighted_normal, build_rotation_matrix


class _FakePass:
    normals = np.array([[0.0, 0.0, 1.0]])
    tangent = np.array([[1.0, 0.0, 0.0]])
    points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])


class _FakeRoute:
    spray_normal = np.array([0.0, 0.0, 1.0])


def test_build_rotation_matrix_matches_olp_export_quaternion():
    """Cross-check: build_rotation_matrix must reproduce the exact quaternion
    olp_export.py::_get_tcp_orientation computes on a shared fixture pose,
    without modifying olp_export.py."""
    q_wxyz_expected, _wpr, _abc = _get_tcp_orientation(_FakeRoute(), _FakePass(), 0)

    n = np.array([0.0, 0.0, 1.0])
    tangent = np.array([1.0, 0.0, 0.0])
    a_hat = -n   # F30: gun pointing axis is -outward_normal

    R = build_rotation_matrix(tangent, a_hat)
    rot = Rotation.from_matrix(R)
    q_xyzw = rot.as_quat()
    q_wxyz = (
        round(float(q_xyzw[3]), 6), round(float(q_xyzw[0]), 6),
        round(float(q_xyzw[1]), 6), round(float(q_xyzw[2]), 6),
    )
    assert q_wxyz == q_wxyz_expected


def test_build_rotation_matrix_is_orthonormal_right_handed():
    R = build_rotation_matrix(tangent=[1.0, 0.3, 0.0], pointing_axis=[0.0, 0.0, -1.0])
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-9)


def test_build_rotation_matrix_handles_parallel_tangent_and_axis():
    # Tangent parallel to the pointing axis — F32's projection would be near-zero;
    # must fall back rather than produce a degenerate/NaN frame.
    R = build_rotation_matrix(tangent=[0.0, 0.0, 1.0], pointing_axis=[0.0, 0.0, 1.0])
    assert np.all(np.isfinite(R))
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)


def test_angular_difference_f35():
    assert angular_difference([1, 0, 0], [1, 0, 0]) == pytest.approx(0.0, abs=1e-9)
    assert angular_difference([1, 0, 0], [0, 1, 0]) == pytest.approx(np.pi / 2, abs=1e-9)
    assert angular_difference([1, 0, 0], [-1, 0, 0]) == pytest.approx(np.pi, abs=1e-9)


def test_area_weighted_normal_matches_face_normal_on_flat_region():
    import trimesh
    mesh = trimesh.creation.box(extents=[100, 100, 100])
    # Query a point on the (flat) +Z face — barycentric blend of that face's
    # vertex normals should closely match the face normal itself.
    pts = np.array([[0.0, 0.0, 50.0]])
    n = area_weighted_normal(pts, mesh)
    np.testing.assert_allclose(np.abs(n[0]), [0, 0, 1], atol=1e-6)
