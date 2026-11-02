"""Validate external simulator scenario inputs without adding WSL dependencies."""

import json
import math
from pathlib import Path


def _number(value, label, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    if not minimum <= value <= maximum:
        raise ValueError(f"{label} must be in [{minimum}, {maximum}]")
    return float(value)


def load_profile(path: Path) -> dict:
    """Read a complete scenario. Wind is ENU m/s; noise fields are multipliers or SI."""
    profile = json.loads(path.read_text(encoding="utf-8"))
    if set(profile) != {"name", "seed", "sitl", "gazebo"}:
        raise ValueError("scenario must contain name, seed, sitl, gazebo only")
    if not isinstance(profile["name"], str) or not profile["name"].isascii() or not profile["name"].replace("_", "").isalnum():
        raise ValueError("scenario name must use English letters, digits, and underscores")
    seed = profile["seed"]
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**31:
        raise ValueError("seed must be a nonnegative 31-bit integer")
    sitl = profile["sitl"]
    if set(sitl) != {"wind_speed_mps", "wind_from_deg", "wind_turbulence", "accel_noise_mps2", "momentum_drag"}:
        raise ValueError("sitl scenario fields are incomplete or unknown")
    _number(sitl["wind_speed_mps"], "wind_speed_mps", 0, 10)
    _number(sitl["wind_from_deg"], "wind_from_deg", 0, 360)
    _number(sitl["wind_turbulence"], "wind_turbulence", 0, 2)
    _number(sitl["accel_noise_mps2"], "accel_noise_mps2", 0, 1)
    _number(sitl["momentum_drag"], "momentum_drag", 0, 1)
    gazebo = profile["gazebo"]
    if set(gazebo) != {"wind_velocity_enu_mps", "wind_force_scale", "wind_gust_fraction",
                       "wind_gust_period_s", "imu_gyro_noise_scale", "imu_accel_noise_scale"}:
        raise ValueError("gazebo scenario fields are incomplete or unknown")
    velocity = gazebo["wind_velocity_enu_mps"]
    if not isinstance(velocity, list) or len(velocity) != 3:
        raise ValueError("wind_velocity_enu_mps must have three components")
    for component in velocity:
        _number(component, "wind_velocity_enu_mps", -10, 10)
    _number(gazebo["wind_force_scale"], "wind_force_scale", 0, 2)
    _number(gazebo["wind_gust_fraction"], "wind_gust_fraction", 0, 0.5)
    _number(gazebo["wind_gust_period_s"], "wind_gust_period_s", 1, 120)
    _number(gazebo["imu_gyro_noise_scale"], "imu_gyro_noise_scale", 0, 10)
    _number(gazebo["imu_accel_noise_scale"], "imu_accel_noise_scale", 0, 10)
    return profile
