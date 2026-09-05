from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

UNIT_TO_MM: dict[str, float] = {
    'mm': 1.0,
    'cm': 10.0,
    'm':  1000.0,
    'in': 25.4,
    'ft': 304.8,
}


@dataclass
class PaintPass:
    id: int
    region_id: str
    direction: str        # "horizontal" — extensible to "vertical" later
    points: np.ndarray    # shape (N, 3), world coordinates in mm
    is_forward: bool      # True = first stitched direction, False = reversed
    sub_index: int        # 0 = primary chain, >0 = additional chain at same level (holes)
    slice_position: float # coordinate along slice axis, for debugging


@dataclass
class Connection:
    id: int
    from_pass_id: int
    to_pass_id: int
    points: np.ndarray    # shape (M, 3), ordered points along the actual mesh boundary
    is_air_move: bool     # False for valid MVP boundary connectors


@dataclass
class PaintRoute:
    region_id: str
    passes: list          # list[PaintPass]
    connections: list     # list[Connection]
    unit: str             # always 'mm' internally
    spacing_mm: float
    total_passes: int
    total_length_mm: float
    spray_normal: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Unit vector: spray gun approach direction perpendicular to the surface plane.
