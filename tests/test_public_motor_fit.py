import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


def load_module():
    deployment = Path(__file__).resolve().parents[1] / "deployment"
    sys.path.insert(0, str(deployment))
    spec = importlib.util.spec_from_file_location("public_motor_fit", deployment / "fit_public_uav_fd_motors.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_recovers_known_pwm_rpm_map_on_held_out_time():
    pwm_time = np.arange(0, 80, 0.01)
    command = 1500 + 60 * np.sin(0.57 * pwm_time) + 35 * np.sin(1.31 * pwm_time)
    pwm = np.column_stack([np.zeros(len(pwm_time)), pwm_time * 1e6, command])
    esc_time = np.arange(0.5, 79.5, 0.1)
    rpm = 8 * np.interp(esc_time - 0.12, pwm_time, command) - 6500
    esc = np.column_stack([np.zeros(len(esc_time)), esc_time * 1e6,
                           np.zeros(len(esc_time)), rpm])
    result = load_module().fit_motor(pwm, esc)
    assert result["fit_time_shift_s"] == pytest.approx(0.12)
    assert result["slope_rpm_per_pwm_us"] == pytest.approx(8, rel=1e-4)
    assert result["offset_rpm"] == pytest.approx(-6500, abs=1)
    assert result["test_rmse_rpm"] < 1
    assert result["test_r2"] > 0.999
