"""Export PaintRoute list to neutral JSON for OLP software.

Structure:
  root
  ├── version
  ├── olp_convention        — tool frame mapping guide for any OLP
  ├── generation_params     — software, timestamp, source file, all path settings
  ├── summary               — totals across all routes
  └── routes[]
      ├── region_id, spacing_mm, spray_normal
      ├── execution_sequence[]  — ordered [{type, id}] for robot program
      ├── passes[]
      │   ├── id, region_id, is_forward, length_mm
      │   ├── surface_normal   [NX, NY, NZ]  — same as route spray_normal
      │   ├── start / end      [X, Y, Z]
      │   └── tcp_waypoints[]  [[X,Y,Z], …]  — all path points (raw or resampled)
      └── connections[]
          ├── id, from_pass_id, to_pass_id, length_mm
          ├── start / end
          └── tcp_waypoints[]  [[X,Y,Z], …]

OLP mapping:
  NX/NY/NZ = outward surface normal; gun approach = -N; OLP computes W/P/R.
  passes     → Trigger ON  (spray gun firing)
  connections → Trigger OFF (air travel)
"""
from __future__ import annotations
import json

import numpy as np
from app.path.path_model import PaintRoute, GenerationParams


_OLP_CONVENTION = {
    "unit":              "mm",
    "normal_convention": "outward surface normal — gun approach direction = -NX/-NY/-NZ",
    "tool_frame":        "Z = -spray_normal  |  X = pass travel direction  |  Y = cross(Z,X)",
    "Trigger_ON":        "spray gun firing (pass moves)",
    "Trigger_OFF":       "air travel (connection moves)",
    "mapping": {
        "RoboDK_drag_drop": "X Y Z NX NY NZ  (6-col curve import, Utilities > Import Curve)",
        "VisualComponents": "seq_id + X Y Z NX NY NZ + Trigger  (import script)",
        "DELMIA":           "seq_id + X Y Z NX NY NZ + Trigger  (APT macro)",
    },
}


class _NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        return super().default(obj)


def export_route_json(
    routes: list[PaintRoute],
    filepath: str,
    params: GenerationParams | None = None,
) -> None:
    """Export routes to neutral JSON. Always includes all tcp_waypoints."""
    total_passes      = sum(r.total_passes for r in routes)
    total_connections = sum(len(r.connections) for r in routes)
    total_length      = sum(r.total_length_mm for r in routes)
    regions           = sorted({r.region_id for r in routes})

    gen_block: dict = {}
    if params is not None:
        gen_block = {
            "software":             params.software,
            "source_file":          params.source_file,
            "generated_at":         params.generated_at,
            "path_mode":            params.path_mode,
            "regions":              params.regions,
            "up_axis":              params.up_axis,
            "spray_width_mm":       params.spray_width_mm,
            "standoff_mm":          params.standoff_mm if params.standoff_mm else "off",
            "waypoint_interval_mm": (
                params.waypoint_spacing_mm
                if params.waypoint_spacing_mm
                else "off (raw mesh-slicer points)"
            ),
            "direction":            params.direction,
            "sweep":                params.sweep,
        }

    data = {
        "version":           "2.0",
        "olp_convention":    _OLP_CONVENTION,
        "generation_params": gen_block,
        "summary": {
            "total_routes":      len(routes),
            "total_passes":      total_passes,
            "total_connections": total_connections,
            "total_length_mm":   round(total_length, 3),
            "regions":           regions,
        },
        "routes": [_route_to_dict(r, idx) for idx, r in enumerate(routes)],
    }

    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, cls=_NumpyEncoder, indent=2)


def _route_to_dict(route: PaintRoute, route_index: int) -> dict:
    conn_by_from = {c.from_pass_id: c for c in route.connections}

    execution_sequence = []
    for p in route.passes:
        execution_sequence.append({"type": "pass", "id": p.id, "Trigger": "ON"})
        conn = conn_by_from.get(p.id)
        if conn is not None:
            execution_sequence.append({"type": "connection", "id": conn.id, "Trigger": "OFF"})

    sn = route.spray_normal
    spray_normal = [round(float(sn[0]), 6), round(float(sn[1]), 6), round(float(sn[2]), 6)]

    return {
        "route_index":        route_index,
        "region_id":          route.region_id,
        "unit":               route.unit,
        "spacing_mm":         route.spacing_mm,
        "total_passes":       route.total_passes,
        "total_length_mm":    round(route.total_length_mm, 3),
        "spray_normal":       spray_normal,
        "execution_sequence": execution_sequence,
        "passes":             [_pass_to_dict(p, spray_normal) for p in route.passes],
        "connections":        [_conn_to_dict(c) for c in route.connections],
    }


def _pass_to_dict(p, spray_normal: list) -> dict:
    length = float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1))) if len(p.points) >= 2 else 0.0
    pts = [[round(v, 4) for v in row] for row in p.points.tolist()]
    return {
        "id":             p.id,
        "Trigger":        "ON",
        "region_id":      p.region_id,
        "is_forward":     p.is_forward,
        "length_mm":      round(length, 3),
        "surface_normal": spray_normal,
        "start":          pts[0] if pts else [],
        "end":            pts[-1] if pts else [],
        "tcp_waypoints":  pts,
    }


def _conn_to_dict(c) -> dict:
    length = float(np.sum(np.linalg.norm(np.diff(c.points, axis=0), axis=1))) if len(c.points) >= 2 else 0.0
    pts = [[round(v, 4) for v in row] for row in c.points.tolist()]
    return {
        "id":           c.id,
        "Trigger":      "OFF",
        "from_pass_id": c.from_pass_id,
        "to_pass_id":   c.to_pass_id,
        "length_mm":    round(length, 3),
        "start":        pts[0],
        "end":          pts[-1],
        "tcp_waypoints": pts,
    }
