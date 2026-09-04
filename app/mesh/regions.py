"""Optional: auto-classify triangles into logical regions (TOP/BOTTOM/etc.)."""
from __future__ import annotations
import numpy as np
from .preprocessor import MeshData


def _region_vectors(up_axis: int) -> dict[str, np.ndarray]:
    """Build direction vectors for all 6 regions given the user's up-axis."""
    up = np.zeros(3); up[up_axis] = 1.0
    fwd_axis = (up_axis + 1) % 3
    fwd = np.zeros(3); fwd[fwd_axis] = 1.0
    right_axis = (up_axis + 2) % 3
    right = np.zeros(3); right[right_axis] = 1.0
    return {
        'TOP':    up.copy(),
        'BOTTOM': -up.copy(),
        'FRONT':  fwd.copy(),
        'REAR':   -fwd.copy(),
        'RIGHT':  right.copy(),
        'LEFT':   -right.copy(),
    }


def classify_regions(
    mesh_data: MeshData,
    threshold: float = 0.55,
) -> dict[str, np.ndarray]:
    """Return mapping region_id -> int array of face indices.

    Uses a weighted combination of face-normal alignment (60%) and
    centroid position within the bounding box (40%).
    """
    region_vecs_dict = _region_vectors(mesh_data.up_axis)
    region_names = list(region_vecs_dict.keys())
    region_vecs = np.array([region_vecs_dict[k] for k in region_names])  # (6, 3)

    normals = mesh_data.face_normals        # (F, 3)
    centroids = mesh_data.face_centroids    # (F, 3)
    bbox_center = mesh_data.bbox_center     # (3,)
    bbox_extents = mesh_data.bbox_extents   # (3,)

    # Normal alignment: dot(normal, region_vec), clipped to [0, 1]
    normal_scores = np.clip(normals @ region_vecs.T, 0.0, 1.0)  # (F, 6)

    # Position score: how far along each region direction is the centroid?
    rel = centroids - bbox_center                                 # (F, 3)
    pos_raw = rel @ region_vecs.T                                 # (F, 6)
    # Scale each column by half-extent along that region's dominant axis
    dominant_axes = np.argmax(np.abs(region_vecs), axis=1)       # (6,)
    half_extents = bbox_extents[dominant_axes] / 2.0 + 1e-9      # (6,)
    position_scores = np.clip(pos_raw / half_extents, -1.0, 1.0) * 0.5 + 0.5  # (F, 6)

    combined = 0.6 * normal_scores + 0.4 * position_scores       # (F, 6)

    best_idx = np.argmax(combined, axis=1)                        # (F,)
    best_score = combined[np.arange(len(normals)), best_idx]      # (F,)
    classified = best_score >= threshold

    result: dict[str, np.ndarray] = {}
    for col, name in enumerate(region_names):
        mask = classified & (best_idx == col)
        result[name] = np.where(mask)[0].astype(np.int64)

    return result
