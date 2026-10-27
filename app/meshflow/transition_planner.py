"""Mesh Flow Phase 5 — transition planner (R8).

Reuses app.path.connector.connect_passes for the geometric nearest-endpoint
search, then relabels the resulting Connection.kind — the endpoint-pairing
logic itself is not reimplemented.
"""
from __future__ import annotations
import trimesh

from app.path import connector as _connector
from app.path.path_model import Connection, MeshFlowRegion, MotionSegmentKind, PaintPass


def plan_transitions(
    passes: list[PaintPass],
    mesh: trimesh.Trimesh,
    region: MeshFlowRegion,
    spray_width_mm: float = 100.0,
    waypoint_spacing_mm: float = 0.0,
) -> list[Connection]:
    """R8 — builds lane-connector transitions between consecutive passes within
    one region. All connections produced here are LANE_CONNECTOR kind — this
    function only ever receives passes from a single region (one call per
    region), so region-to-region REGION_TRANSIT connectors are a separate
    concern (built by the caller joining multiple regions' routes, not here);
    see §15.1 "no paint-on connector crosses the excluded region" — connectors
    built per-region can never accidentally cross into another region's space.
    """
    connections = _connector.connect_passes(
        passes, spray_width_mm=spray_width_mm, waypoint_spacing_mm=waypoint_spacing_mm,
    )
    for c in connections:
        c.kind = MotionSegmentKind.LANE_CONNECTOR
    return connections
