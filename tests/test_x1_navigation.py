import numpy as np
from numpy.testing import assert_allclose

from uavsim.config.loader import load_config


def test_x1_manual_rms_budget_in_active_configurations():
    for path in ("configs/navigation_x1_5_rtk.yaml", "main/config.yaml", "main/motion_config.yaml", "configs/three_axis_force_6n.yaml"):
        d = load_config(path).disturbances
        position = np.array(d.navigation_position_error.bias_std_sar_m)
        velocity = np.array(d.navigation_velocity_error.bias_std_sar_mps)
        assert_allclose(np.linalg.norm(position[:2]), 0.015, rtol=1e-14)
        assert_allclose(position[2], 0.020)
        assert_allclose(np.linalg.norm(velocity[:2]), 0.019, rtol=1e-14)
        assert_allclose(velocity[2], 0.014)
        assert_allclose(d.navigation_attitude_error.bias_std_body_deg, [0.018, 0.018, 0.084])
        assert_allclose(d.navigation_position_error.noise_std_sar_m, 0)
        assert_allclose(d.navigation_velocity_error.noise_std_sar_mps, 0)
        assert_allclose(d.navigation_attitude_error.noise_std_body_deg, 0)
