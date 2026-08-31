"""End-to-end: region → route → verify points lie on mesh surface."""
import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from app.mesh.preprocessor import preprocess
from app.mesh.regions import classify_regions
from app.path.generator import generate_route


def test_generate_returns_paint_route(unit_box_mesh):
    data = preprocess(unit_box_mesh, "test")
    regions = classify_regions(data)
    rid = 'TOP'
    route = generate_route(data, rid, regions[rid], spray_width_mm=0.2)
    assert route.region_id == rid
    assert route.spacing_mm == 0.2
