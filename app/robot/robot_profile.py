"""Robot profile data model and JSON persistence."""
from __future__ import annotations
import json
from dataclasses import dataclass, asdict, field
from pathlib import Path


@dataclass
class RobotProfile:
    name: str
    # Speed
    tcp_spray_speed_mmps: float = 500.0
    approach_speed_mmps: float = 1000.0
    max_joint_speed_degps: float = 60.0
    # Angle limits
    max_orientation_change_deg: float = 15.0
    min_approach_angle_deg: float = -15.0
    max_approach_angle_deg: float = 15.0
    max_wrist_tilt_deg: float = 90.0
    # Standoff
    standoff_optimal_mm: float = 200.0
    standoff_min_mm: float = 150.0
    standoff_max_mm: float = 250.0
    # Gun geometry (sphere model)
    nozzle_radius_mm: float = 10.0
    gun_body_length_mm: float = 300.0
    gun_body_diameter_mm: float = 60.0
    # Spray
    spray_cone_deg: float = 30.0
    overlap_factor_pct: float = 15.0
    # Export
    default_speed_mmpm: float = 30000.0
    speed_units: str = 'mm/min'


PROFILES_PATH = Path.home() / 'AppData' / 'Roaming' / 'SmartGrid' / 'robots.json'

_DEFAULT_PROFILE = RobotProfile(name='Default Robot')


def load_profiles() -> tuple[list[RobotProfile], str | None]:
    """Return (profiles, active_name). Empty list + None if file missing."""
    try:
        data = json.loads(PROFILES_PATH.read_text(encoding='utf-8'))
        profiles = [RobotProfile(**p) for p in data.get('profiles', [])]
        return profiles, data.get('active')
    except Exception:
        return [], None


def save_profiles(profiles: list[RobotProfile], active_name: str | None = None) -> None:
    PROFILES_PATH.parent.mkdir(parents=True, exist_ok=True)
    existing_active = None
    try:
        existing_active = json.loads(PROFILES_PATH.read_text(encoding='utf-8')).get('active')
    except Exception:
        pass
    PROFILES_PATH.write_text(
        json.dumps({
            'profiles': [asdict(p) for p in profiles],
            'active': active_name if active_name is not None else existing_active,
        }, indent=2),
        encoding='utf-8',
    )


def get_active_profile() -> RobotProfile | None:
    profiles, active_name = load_profiles()
    if not profiles or not active_name:
        return None
    return next((p for p in profiles if p.name == active_name), None)


def set_active_profile(name: str) -> None:
    profiles, _ = load_profiles()
    save_profiles(profiles, active_name=name)


if __name__ == '__main__':
    # Self-check: round-trip a profile through JSON
    p = RobotProfile(name='Test', nozzle_radius_mm=12.5)
    save_profiles([p], active_name='Test')
    profiles, active = load_profiles()
    assert len(profiles) == 1
    assert profiles[0].nozzle_radius_mm == 12.5
    assert active == 'Test'
    assert get_active_profile().name == 'Test'
    print('robot_profile: OK')
