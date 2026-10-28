"""Mesh Flow Phase 3 — region manager (R1).

Wraps app.mesh.regions.classify_regions_with_confidence (reused, not reimplemented)
and surfaces its per-face confidence score into MeshFlowRegion.confidence/
confidence_reasons instead of discarding it.
"""
from __future__ import annotations
import numpy as np
import trimesh

from app.mesh.preprocessor import MeshData
from app.mesh.regions import classify_regions_with_confidence
from app.meshflow.open_decisions import OpenDecisionBlocked, RegionBoundaryPolicy
from app.path.path_model import MeshFlowRegion, RegionConfidence

# A region whose mean per-face confidence score falls below this is FLAGGED rather
# than ACCEPTED outright — still usable, but surfaced for review (spec §15.1/§15.2).
_ACCEPT_MEAN_SCORE = 0.6


# A connected sub-patch must have at least this fraction of the region's LARGEST
# patch's area to count as a real panel rather than classification noise (a decoy
# bracket/liner/trim sliver). Measured directly on Part1.STL: real panels within a
# region bottom out around 5-8% of that region's largest patch (e.g. TOP's 11
# genuine panels range 100%..7.6% of its max); junk components are always below
# 0.5% (TOP's next-smallest jumps straight to 0.3%, then under 0.1%) — a >10x gap
# either side of 2%, so this cutoff is not finely tuned to one file.
_MIN_COMPONENT_AREA_FRACTION = 0.02


def _significant_connected_components(mesh: trimesh.Trimesh, face_indices: np.ndarray, cover_all: bool = False) -> np.ndarray:
    """Restrict face_indices to every connected patch substantial enough to be a
    real panel, dropping only classification noise.

    app.mesh.regions.classify_regions scores each face independently — normal
    direction + bbox-side position, with no connectivity term. On a real
    multi-part mesh (diagnosed directly against Part1.STL: every one of its 6
    named regions split into 16-96 disconnected sub-patches), that merges
    physically unrelated surfaces into one region whenever their normals and
    bbox-side happen to coincide — which is what let waypoints land inside the
    mesh (lane_planner/surface_intersector/core_generator all consume
    region.face_indices as if it were one physical surface; a standoff-offset
    point is only ever wrong because the POINT itself came from the wrong
    surface, not because the offset formula was wrong).

    Keeping only the single largest component (the first version of this fix)
    over-corrected: on Part1.STL it discarded 65-80% of each region's real
    surface area, because a region's true surface is itself legitimately split
    across several separate panel shells (e.g. TOP's hood/roof/trunk, or a
    left/right mirrored pair) — confirmed by a sharp area cliff between real
    panels and junk (see _MIN_COMPONENT_AREA_FRACTION). Keeping every patch
    above that cliff restores full real coverage while still excluding the
    unrelated small parts that caused the original inside-the-mesh defect.
    """
    if cover_all:
        return face_indices
    if len(face_indices) <= 1:
        return face_indices
    adjacency = mesh.face_adjacency
    in_region = np.isin(adjacency, face_indices).all(axis=1)
    components = trimesh.graph.connected_components(
        adjacency[in_region], nodes=face_indices, min_len=1,
    )
    if len(components) <= 1:
        return face_indices
    areas = mesh.area_faces
    comp_areas = np.array([areas[c].sum() for c in components])
    cutoff = comp_areas.max() * _MIN_COMPONENT_AREA_FRACTION
    kept = [comp for comp, area in zip(components, comp_areas) if area >= cutoff]
    return np.sort(np.concatenate(kept))


def build_regions(
    mesh_data: MeshData,
    selected_region_ids: list[str],
    boundary_policy: RegionBoundaryPolicy = RegionBoundaryPolicy.EXPLICIT_FACE_SELECTION,
    classification_threshold: float = 0.42,
    cover_all: bool = False,
) -> list[MeshFlowRegion]:
    """Build MeshFlowRegion records for the selected named regions (R1).

    Only RegionBoundaryPolicy.EXPLICIT_FACE_SELECTION is implemented — it reuses the
    existing named-face classification exactly as Boundary Box/Mesh Surface do today,
    then restricts each region to its significant connected surface patches (see
    _significant_connected_components) — a correctness fix for this one policy, not
    a new policy choice. A true hole-aware/protected-feature-aware boundary policy
    remains decision #2 (spec §12) and is not guessed at here.
    """
    if boundary_policy is not RegionBoundaryPolicy.EXPLICIT_FACE_SELECTION:
        raise OpenDecisionBlocked(2, f"region boundary policy {boundary_policy.value!r} not implemented")

    region_faces, best_score = classify_regions_with_confidence(mesh_data, classification_threshold)
    mesh = mesh_data.trimesh_mesh

    regions: list[MeshFlowRegion] = []
    for region_id in selected_region_ids:
        face_indices = region_faces.get(region_id, np.array([], dtype=np.int64))
        if len(face_indices) == 0:
            regions.append(MeshFlowRegion(
                region_id=region_id,
                face_indices=face_indices,
                confidence=RegionConfidence.REJECTED,
                confidence_reasons=[f"no faces classified into region {region_id!r}"],
            ))
            continue

        raw_count = len(face_indices)
        face_indices = _significant_connected_components(mesh, face_indices, cover_all=cover_all)
        dropped_note = (
            f"; restricted to significant connected surface patches "
            f"({len(face_indices)}/{raw_count} faces kept)"
            if len(face_indices) != raw_count else ""
        )

        mean_score = float(best_score[face_indices].mean())
        min_score = float(best_score[face_indices].min())
        if mean_score >= _ACCEPT_MEAN_SCORE:
            confidence = RegionConfidence.ACCEPTED
            reasons = [f"mean face-classification score {mean_score:.2f} >= {_ACCEPT_MEAN_SCORE}{dropped_note}"]
        else:
            confidence = RegionConfidence.FLAGGED
            reasons = [
                f"mean face-classification score {mean_score:.2f} < {_ACCEPT_MEAN_SCORE} — "
                f"region boundary may be ambiguous (min face score {min_score:.2f}){dropped_note}",
            ]

        regions.append(MeshFlowRegion(
            region_id=region_id,
            face_indices=face_indices,
            confidence=confidence,
            confidence_reasons=reasons,
        ))

    return regions
