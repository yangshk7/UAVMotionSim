import importlib.util
import json
from pathlib import Path

import pytest

PROFILE = Path(__file__).resolve().parents[1] / "deployment/scenarios/crosswind_sensor.json"
SPEC = importlib.util.spec_from_file_location("scenario_profile", PROFILE.parent.parent / "scenario_profile.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
load_profile = MODULE.load_profile


def test_profile_has_explicit_bounded_parameters():
    profile = load_profile(PROFILE)
    assert profile["sitl"]["wind_speed_mps"] == 2
    assert profile["gazebo"]["wind_velocity_enu_mps"] == [-2, 0, 0]
    assert profile["seed"] == 20261101


@pytest.mark.parametrize("change", [
    {"name": "含中文"},
    {"seed": -1},
    {"sitl": {"wind_speed_mps": float("nan")}},
    {"gazebo": {"wind_velocity_enu_mps": [1, 2]}},
])
def test_invalid_profile_is_rejected(tmp_path, change):
    profile = load_profile(PROFILE)
    for key, value in change.items():
        if isinstance(value, dict):
            profile[key].update(value)
        else:
            profile[key] = value
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(profile), encoding="utf-8")
    with pytest.raises(ValueError):
        load_profile(path)
