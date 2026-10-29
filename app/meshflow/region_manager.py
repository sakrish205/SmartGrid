"""Mesh Flow Phase 3 ΓÇö region manager (R1).

Wraps app.mesh.regions.classify_regions_with_confidence (reused, not reimplemented)
and surfaces its per-face confidence score into MeshFlowRegion.confidence/
confidence_reasons instead of discarding it.
"""
from __future__ import annotations
import numpy as np

from app.mesh.preprocessor import MeshData
from app.mesh.regions import classify_regions_with_confidence
from app.meshflow.open_decisions import OpenDecisionBlocked, RegionBoundaryPolicy
from app.path.path_model import MeshFlowRegion, RegionConfidence

# A region whose mean per-face confidence score falls below this is FLAGGED rather
# than ACCEPTED outright ΓÇö still usable, but surfaced for review (spec ┬º15.1/┬º15.2).
_ACCEPT_MEAN_SCORE = 0.6


def build_regions(
    mesh_data: MeshData,
    selected_region_ids: list[str],
    boundary_policy: RegionBoundaryPolicy = RegionBoundaryPolicy.EXPLICIT_FACE_SELECTION,
    classification_threshold: float = 0.42,
) -> list[MeshFlowRegion]:
    """Build MeshFlowRegion records for the selected named regions (R1).

    Only RegionBoundaryPolicy.EXPLICIT_FACE_SELECTION is implemented ΓÇö it reuses the
    existing named-face classification exactly as Boundary Box/Mesh Surface do today.
    Hole-aware, disconnected-component-aware, and protected-feature-aware boundary
    policies are decision #2 (spec ┬º12) and are not guessed at here.
    """
    if boundary_policy is not RegionBoundaryPolicy.EXPLICIT_FACE_SELECTION:
        raise OpenDecisionBlocked(2, f"region boundary policy {boundary_policy.value!r} not implemented")

    region_faces, best_score = classify_regions_with_confidence(mesh_data, classification_threshold)

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

        mean_score = float(best_score[face_indices].mean())
        min_score = float(best_score[face_indices].min())
        if mean_score >= _ACCEPT_MEAN_SCORE:
            confidence = RegionConfidence.ACCEPTED
            reasons = [f"mean face-classification score {mean_score:.2f} >= {_ACCEPT_MEAN_SCORE}"]
        else:
            confidence = RegionConfidence.FLAGGED
            reasons = [
                f"mean face-classification score {mean_score:.2f} < {_ACCEPT_MEAN_SCORE} ΓÇö "
                f"region boundary may be ambiguous (min face score {min_score:.2f})",
            ]

        regions.append(MeshFlowRegion(
            region_id=region_id,
            face_indices=face_indices,
            confidence=confidence,
            confidence_reasons=reasons,
        ))

    return regions
