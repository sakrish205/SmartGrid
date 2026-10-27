"""Reference-program metrics — Mesh Flow Phase 2.

Diagnostic only: consumes a `ReferenceProgram` (Phase 1) and reports metrics
using the spec's exact formulas (cited by ID below). Does not touch
app/path/ or the live Mesh Surface generator except to reuse resampler.py's
arc-length utility, per F17's own implementation note ("resample by arc
length before interpreting magnitude").
"""
from __future__ import annotations
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from app.path.curvature_metrics import (
    approximate_curvature,
    segment_directions,
    second_differences,
    smoothness_penalty,
)
from app.path.segment_runs import (
    split_contiguous_gun_on_runs,
    split_contiguous_gun_on_runs_including_unknown,
)
from app.reference.reference_model import ReferenceProgram, ReferenceWaypoint


def segment_lengths(points: np.ndarray) -> np.ndarray:
    """F36 — l_i = ||q_(i+1) - q_i||."""
    return np.linalg.norm(np.diff(points, axis=0), axis=1)


def total_length(points: np.ndarray) -> float:
    """F37 — L = sum(l_i)."""
    return float(segment_lengths(points).sum())


def pointing_axis(orientation: np.ndarray, controller: str) -> np.ndarray | None:
    """Tool pointing axis (approach direction) from raw reference orientation.

    Reuses the convention already established in app/export/olp_export.py's
    _get_tcp_orientation: ABB quaternion is [w,x,y,z] (scipy wants [x,y,z,w]);
    Fanuc/Durr orientation is extrinsic-XYZ Euler in degrees. The pointing
    axis is the tool Z column of the resulting rotation matrix.
    """
    if orientation is None:
        return None
    if controller == 'ABB':
        w, x, y, z = orientation
        rot = Rotation.from_quat([x, y, z, w])
    else:  # FANUC or DURR: W/P/R or A/B/C as extrinsic XYZ degrees
        rot = Rotation.from_euler('xyz', orientation, degrees=True)
    return rot.as_matrix()[:, 2]


def orientation_continuity(axes: np.ndarray) -> np.ndarray:
    """F44 — theta_i = acos(clamp(a_i . a_(i+1), -1, 1))."""
    if len(axes) < 2:
        return np.zeros(0)
    dots = np.clip(np.sum(axes[:-1] * axes[1:], axis=1), -1.0, 1.0)
    return np.arccos(dots)


def max_orientation_jump(axes: np.ndarray) -> float:
    """F45 — theta_max = max_i theta_i."""
    theta = orientation_continuity(axes)
    return float(theta.max()) if len(theta) else 0.0


def lane_pitch(centroid_a: np.ndarray, centroid_b: np.ndarray, spacing_dir: np.ndarray) -> float:
    """F39 — Delta_k ~= |(r - p) . s_hat|, s_hat a unit spacing direction."""
    s_hat = spacing_dir / np.linalg.norm(spacing_dir)
    return float(abs(np.dot(centroid_b - centroid_a, s_hat)))


def alternation_signs(lane_directions: list[np.ndarray]) -> list[float]:
    """F41 check — compare each lane's direction sign against (-1)^k * d_0."""
    if not lane_directions:
        return []
    d0 = lane_directions[0]
    return [float(np.dot(d, d0) * ((-1) ** k)) for k, d in enumerate(lane_directions)]


@dataclass
class LaneMetrics:
    waypoint_count: int
    total_length_mm: float
    smoothness_penalty: float
    max_curvature: float
    max_orientation_jump_rad: float
    mean_direction: np.ndarray | None


@dataclass
class ReferenceMetricsReport:
    source_file: str
    controller: str
    lanes: list[LaneMetrics] = field(default_factory=list)
    lane_pitches_mm: list[float] = field(default_factory=list)
    alternation_signs: list[float] = field(default_factory=list)
    skipped_transit_runs: int = 0


def compute_reference_metrics(program: ReferenceProgram, arc_spacing_mm: float = 10.0) -> ReferenceMetricsReport:
    motion = program.motion_waypoints()
    lane_runs = split_contiguous_gun_on_runs(motion)
    all_run_count = len(split_contiguous_gun_on_runs_including_unknown(motion))

    report = ReferenceMetricsReport(
        source_file=program.source_file,
        controller=program.controller,
        skipped_transit_runs=max(0, all_run_count - len(lane_runs)),
    )

    centroids: list[np.ndarray] = []
    directions: list[np.ndarray] = []

    for run in lane_runs:
        if len(run) < 2:
            continue
        points = np.array([w.position for w in run])
        axes = np.array([
            a for w in run if (a := pointing_axis(w.orientation, program.controller)) is not None
        ])
        report.lanes.append(LaneMetrics(
            waypoint_count=len(run),
            total_length_mm=total_length(points),
            smoothness_penalty=smoothness_penalty(points),
            max_curvature=float(approximate_curvature(points, arc_spacing_mm).max())
                if len(points) >= 3 else 0.0,
            max_orientation_jump_rad=max_orientation_jump(axes) if len(axes) >= 2 else 0.0,
            mean_direction=segment_directions(points).mean(axis=0) if len(points) >= 2 else None,
        ))
        centroids.append(points.mean(axis=0))
        if report.lanes[-1].mean_direction is not None:
            directions.append(report.lanes[-1].mean_direction)

    if len(centroids) >= 2:
        spacing_dir = centroids[1] - centroids[0]
        if np.linalg.norm(spacing_dir) > 1e-9:
            report.lane_pitches_mm = [
                lane_pitch(centroids[i], centroids[i + 1], spacing_dir)
                for i in range(len(centroids) - 1)
            ]
    report.alternation_signs = alternation_signs(directions)
    return report
