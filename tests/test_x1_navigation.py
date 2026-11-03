import numpy as np
from numpy.testing import assert_allclose

from uavsim.config.loader import load_config


def test_x1_manual_rms_budget_in_active_configurations():
    for path in ("configs/navigation_x1_5_rtk.yaml", "main/config.yaml", "main/motion_config.yaml", "configs/three_axis_force_6n.yaml"):
        d = load_config(path).disturbances
        position_bias = np.array(d.navigation_position_error.bias_std_sar_m)
        position_noise = np.array(d.navigation_position_error.noise_std_sar_m)
        velocity_bias = np.array(d.navigation_velocity_error.bias_std_sar_mps)
        velocity_noise = np.array(d.navigation_velocity_error.noise_std_sar_mps)
        attitude_bias = np.array(d.navigation_attitude_error.bias_std_body_deg)
        attitude_noise = np.array(d.navigation_attitude_error.noise_std_body_deg)

        assert_allclose(np.linalg.norm(np.hypot(position_bias, position_noise)[:2]), 0.015, rtol=1e-4)
        assert_allclose(np.hypot(position_bias, position_noise)[2], 0.020, rtol=1e-4)
        assert_allclose(np.linalg.norm(np.hypot(velocity_bias, velocity_noise)[:2]), 0.019, rtol=1e-4)
        assert_allclose(np.hypot(velocity_bias, velocity_noise)[2], 0.014, rtol=1e-4)
        assert_allclose(np.hypot(attitude_bias, attitude_noise), [0.018, 0.018, 0.084], rtol=1e-4)
        for bias, noise in (
            (position_bias, position_noise),
            (velocity_bias, velocity_noise),
            (attitude_bias, attitude_noise),
        ):
            assert_allclose(noise**2 / (bias**2 + noise**2), 0.20, rtol=2e-4)
