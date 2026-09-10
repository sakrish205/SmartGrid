"""Export PaintRoute list to JSON for OLP software.

Structure:
  root
  ├── version
  ├── generation_params   — software, timestamp, source file, all path settings
  ├── summary             — totals across all routes
  └── routes[]
      ├── route metadata (region, spacing, spray_normal)
      │     spray_normal is the OUTWARD surface unit normal.
      │     OLP tool approach direction = -spray_normal (gun points into surface).
      │     Full 6-DOF tool frame: Z = -spray_normal, X = pass_direction,
      │                            Y = cross(Z, X).
      ├── execution_sequence[]  — ordered list of {type, id} for robot program
      ├── passes[]     — full pass data (points are the TCP waypoints in mm)
      └── connections[] — air moves between passes (is_spray = False)
"""
from __future__ import annotations
import json
from datetime import datetime

import numpy as np
from app.path.path_model import PaintRoute, GenerationParams


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
    show_waypoints: bool = True,
    params: GenerationParams | None = None,
) -> None:
    """Export routes to JSON.

    show_waypoints=True  → full tcp_waypoints array per pass.
    show_waypoints=False → only start/end in each pass (no tcp_waypoints key).
    params               → if provided, written as generation_params for repeatability.
    """
    total_passes      = sum(r.total_passes for r in routes)
    total_connections = sum(len(r.connections) for r in routes)
    total_length      = sum(r.total_length_mm for r in routes)
    regions           = sorted({r.region_id for r in routes})
    directions        = sorted({p.direction for r in routes for p in r.passes})

    gen_block: dict = {}
    if params is not None:
        gen_block = {
            "software":            params.software,
            "source_file":         params.source_file,
            "generated_at":        params.generated_at,
            "path_mode":           params.path_mode,
            "regions":             params.regions,
            "up_axis":             params.up_axis,
            "spray_width_mm":      params.spray_width_mm,
            "standoff_mm":         params.standoff_mm if params.standoff_mm else "off",
            "waypoint_interval_mm": (
                params.waypoint_spacing_mm
                if params.waypoint_spacing_mm
                else "off"
            ),
            "direction":           params.direction,
            "sweep":               params.sweep,
        }

    data = {
        "version": "1.3",
        "generation_params": gen_block,
        "summary": {
            "total_routes":       len(routes),
            "total_passes":       total_passes,
            "total_connections":  total_connections,
            "total_length_mm":    round(total_length, 3),
            "regions":            regions,
            "directions":         directions,
            "waypoints_included": show_waypoints,
        },
        "routes": [_route_to_dict(r, idx, show_waypoints) for idx, r in enumerate(routes)],
    }

    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, cls=_NumpyEncoder, indent=2)


def _route_to_dict(route: PaintRoute, route_index: int, show_waypoints: bool = True) -> dict:
    dirs = {p.direction for p in route.passes}
    route_direction = dirs.pop() if len(dirs) == 1 else 'mixed'

    conn_by_from = {c.from_pass_id: c for c in route.connections}

    # Build execution sequence in robot program order
    execution_sequence = []
    for p in route.passes:
        execution_sequence.append({"type": "pass", "id": p.id})
        conn = conn_by_from.get(p.id)
        if conn is not None:
            execution_sequence.append({"type": "connection", "id": conn.id})

    sn = route.spray_normal
    return {
        "route_index":        route_index,
        "region_id":          route.region_id,
        "direction":          route_direction,
        "unit":               route.unit,
        "spacing_mm":         route.spacing_mm,
        "total_passes":       route.total_passes,
        "total_length_mm":    round(route.total_length_mm, 3),
        "spray_normal":       [round(float(sn[0]), 6), round(float(sn[1]), 6), round(float(sn[2]), 6)],
        "execution_sequence": execution_sequence,
        "passes":      [_pass_to_dict(p, show_waypoints) for p in route.passes],
        "connections": [_conn_to_dict(c) for c in route.connections],
    }


def _pass_to_dict(p, show_waypoints: bool = True) -> dict:
    length = float(np.sum(np.linalg.norm(np.diff(p.points, axis=0), axis=1))) if len(p.points) >= 2 else 0.0
    pts = [[round(v, 4) for v in row] for row in p.points.tolist()]
    if len(p.points) >= 2:
        d_vec = p.points[1] - p.points[0]
        dn = np.linalg.norm(d_vec)
        d_vec = d_vec / dn if dn > 1e-9 else d_vec
        pass_dir = [round(float(d_vec[0]), 6), round(float(d_vec[1]), 6), round(float(d_vec[2]), 6)]
    else:
        pass_dir = [0.0, 0.0, 0.0]
    d = {
        "id":             p.id,
        "region_id":      p.region_id,
        "direction":      p.direction,
        "is_forward":     p.is_forward,
        "sub_index":      p.sub_index,
        "slice_position": round(float(p.slice_position), 4),
        "length_mm":      round(length, 3),
        "pass_direction": pass_dir,
        "start":          pts[0],
        "end":            pts[-1],
    }
    if show_waypoints:
        d["tcp_waypoints"] = pts
    return d


def _conn_to_dict(c) -> dict:
    length = float(np.sum(np.linalg.norm(np.diff(c.points, axis=0), axis=1))) if len(c.points) >= 2 else 0.0
    pts = [[round(v, 4) for v in row] for row in c.points.tolist()]
    return {
        "id":           c.id,
        "from_pass_id": c.from_pass_id,
        "to_pass_id":   c.to_pass_id,
        "is_air_move":  c.is_air_move,
        "length_mm":    round(length, 3),
        "start":        pts[0],
        "end":          pts[-1],
        "tcp_waypoints": pts,
    }
