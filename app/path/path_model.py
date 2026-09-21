from __future__ import annotations
from dataclasses import dataclass, field
from datetime import datetime
import numpy as np

UNIT_TO_MM: dict[str, float] = {
    'mm': 1.0,
    'cm': 10.0,
    'm':  1000.0,
    'in': 25.4,
    'ft': 304.8,
}


_SOFTWARE = 'SmartGrid 1.3'


@dataclass
class GenerationParams:
    """All settings used for one path generation run — included in every export for repeatability."""
    source_file:         str        # mesh filename (basename only)
    path_mode:           str        # 'Boundary Box' | 'Face Grid / Adaptive' | 'Face Grid / Conform' | 'Mesh Surface'
    regions:             list       # e.g. ['TOP', 'FRONT']
    up_axis:             str        # 'X' | 'Y' | 'Z'
    spray_width_mm:      float
    standoff_mm:         float      # 0.0 = none applied
    waypoint_spacing_mm: float      # 0.0 = disabled (uniform mesh vertices used as-is)
    direction:           str        # 'horizontal' | 'vertical' | 'both'
    sweep:               str        # 'CW' | 'CCW'
    paint_speed_mmpm:    float      = 1000.0   # robot TCP speed during spray passes (mm/min)
    generated_at:        str = field(default_factory=lambda: datetime.now().isoformat(timespec='seconds'))
    software:            str = field(default=_SOFTWARE)


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
