"""Mesh Flow Phase 3 — surface intersection.

Reuses app.path.slicer.slice_region and app.path.stitcher.stitch_segments
unchanged (same trimesh.intersections.mesh_plane wrapping already proven in
app.path.generator.generate_route). region_id is passed as '' so slice_region
takes its strict face-membership branch (np.isin against region.face_indices) —
the same technique app.path.generator already uses for combined/arbitrary
selections, since Mesh Flow regions are not necessarily one of the 6 named
axis-aligned faces.
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.path import slicer as _slicer
from app.path import stitcher as _stitcher
from app.path.path_model import MeshFlowRegion


def intersect_region(
    mesh: trimesh.Trimesh,
    region: MeshFlowRegion,
    plane_normal: np.ndarray,
    plane_origin: np.ndarray,
) -> list[np.ndarray]:
    """Intersect the region's faces with one plane, return ordered polylines.

    Returns an empty list if the plane misses the region entirely.
    """
    segments = _slicer.slice_region(
        mesh, region.face_indices,
        plane_normal, plane_origin,
        region_id='',   # strict np.isin membership — region may not be axis-aligned
        up_axis=2,      # unused by slice_region when region_id=='' (no outward-sign lookup)
    )
    if segments is None:
        return []
    return _stitcher.stitch_segments(segments)
