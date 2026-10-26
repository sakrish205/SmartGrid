"""Phase 0 golden-baseline guard for Mesh Flow (see convertmdfiles/MeshFlow_Phase0_Rename_Audit.md).

Captures generate_route()'s current output on real meshes before the Mesh Flow
rename/rewrite touches anything. Phase 8 of the plan must reproduce these exact
values after the rename to prove it introduced no behavior change.
"""
from __future__ import annotations
import json
import os

import numpy as np
import trimesh

from app.mesh.preprocessor import preprocess
from app.mesh.regions import classify_regions
from app.path.generator import generate_route

_FIXTURE_PATH = os.path.join(os.path.dirname(__file__), 'fixtures', 'mesh_surface_baseline.json')


def _capture(mesh, name, spray_width_mm, region):
    data = preprocess(mesh, name)
    regions = classify_regions(data)
    route = generate_route(data, region, regions[region], spray_width_mm=spray_width_mm)
    return {
        'mesh': name,
        'region_id': route.region_id,
        'spacing_mm': route.spacing_mm,
        'total_passes': route.total_passes,
        'total_length_mm': round(route.total_length_mm, 6),
        'n_connections': len(route.connections),
        'pass_point_counts': [len(p.points) for p in route.passes],
        'first_pass_first_point': route.passes[0].points[0].round(6).tolist(),
        'first_pass_last_point': route.passes[0].points[-1].round(6).tolist(),
        'spray_normal': route.spray_normal.round(6).tolist(),
        'normals_is_none': route.passes[0].normals is None,
        'tangent_is_none': route.passes[0].tangent is None,
    }


def test_mesh_surface_matches_baseline():
    with open(_FIXTURE_PATH, encoding='utf-8') as f:
        baseline = json.load(f)

    box = trimesh.creation.box()
    cyl = trimesh.creation.cylinder(radius=50.0, height=200.0, sections=64)

    current = {
        'box_top': _capture(box, 'unit_box', 0.2, 'TOP'),
        'cylinder_front': _capture(cyl, 'cylinder', 20.0, 'FRONT'),
    }

    for key in ('box_top', 'cylinder_front'):
        assert current[key] == baseline[key], (
            f"Mesh Surface baseline drift in '{key}' — generate_route()'s output "
            f"changed. If this is an intentional Mesh Flow rename/rewrite step, "
            f"update tests/fixtures/mesh_surface_baseline.json deliberately; "
            f"otherwise this is a regression."
        )
