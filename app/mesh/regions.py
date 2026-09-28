"""Optional: auto-classify triangles into logical regions (TOP/BOTTOM/etc.)."""
from __future__ import annotations
import numpy as np
from .preprocessor import MeshData


def _region_vectors(up_axis: int, fwd_axis: int, right_axis: int) -> dict[str, np.ndarray]:
    """Build direction vectors for all 6 regions given explicit axis roles."""
    up = np.zeros(3); up[up_axis] = 1.0
    fwd = np.zeros(3); fwd[fwd_axis] = 1.0
    right = np.zeros(3); right[right_axis] = 1.0
    return {
        'TOP':    up.copy(),
        'BOTTOM': -up.copy(),
        'FRONT':  fwd.copy(),
        'REAR':   -fwd.copy(),
        'RIGHT':  right.copy(),
        'LEFT':   -right.copy(),
    }


def _classify_with_axes(
    mesh_data: MeshData,
    fwd_axis: int,
    right_axis: int,
    threshold: float,
) -> dict[str, np.ndarray]:
    """Classify faces given an explicit fwd/right axis assignment.

    Uses a weighted combination of face-normal alignment (25%) and
    centroid position within the bounding box (75%) — position-heavy so a
    region stays spatially confined to its side even when normals of a
    curved/wraparound part briefly point the "wrong" way.
    """
    region_vecs_dict = _region_vectors(mesh_data.up_axis, fwd_axis, right_axis)
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

    combined = 0.25 * normal_scores + 0.75 * position_scores     # (F, 6)

    best_idx = np.argmax(combined, axis=1)                        # (F,)
    best_score = combined[np.arange(len(normals)), best_idx]      # (F,)
    classified = best_score >= threshold

    result: dict[str, np.ndarray] = {}
    for col, name in enumerate(region_names):
        mask = classified & (best_idx == col)
        result[name] = np.where(mask)[0].astype(np.int64)

    return result


def _detect_orientation(
    mesh_data: MeshData,
    threshold: float,
) -> tuple[int, int, dict[str, np.ndarray]]:
    """Pick which of the two non-up axes is 'front' vs 'right'.

    The (up_axis+1)%3 / (up_axis+2)%3 convention is an arbitrary guess that
    only matches some meshes' native orientation. Test both assignments and
    keep whichever gives a more mirror-symmetric LEFT/RIGHT face count — a
    real left/right split of a part should be roughly 1:1, unlike the huge
    imbalance a wrong axis produces (LEFT accidentally becomes "the whole
    front fascia" instead of a corner).

    Falls back to the default convention when the swap isn't a clear win
    (e.g. a genuinely asymmetric part, or one with no faces classified as
    LEFT/RIGHT at all) — an uninformed swap on inconclusive data is worse
    than making no change.
    """
    up_axis = mesh_data.up_axis
    a, b = (up_axis + 1) % 3, (up_axis + 2) % 3
    counts_default = _classify_with_axes(mesh_data, a, b, threshold)
    counts_swapped = _classify_with_axes(mesh_data, b, a, threshold)

    def lr_ratio(counts: dict[str, np.ndarray]) -> float:
        l, r = len(counts['LEFT']), len(counts['RIGHT'])
        if l == 0 or r == 0:
            return float('inf')
        return max(l, r) / min(l, r)

    ratio_default = lr_ratio(counts_default)
    ratio_swapped = lr_ratio(counts_swapped)

    if ratio_swapped < ratio_default * 0.8:
        return b, a, counts_swapped
    return a, b, counts_default


def classify_regions(
    mesh_data: MeshData,
    threshold: float = 0.42,
) -> dict[str, np.ndarray]:
    """Return mapping region_id -> int array of face indices.

    Auto-detects which horizontal axis is "front" vs "right" per mesh (see
    `_detect_orientation`) so this works across differently-oriented STL
    exports, not just one specific mesh's coordinate convention.
    """
    fwd_axis, right_axis, result = _detect_orientation(mesh_data, threshold)
    mesh_data.fwd_axis = fwd_axis
    mesh_data.right_axis = right_axis
    return result
