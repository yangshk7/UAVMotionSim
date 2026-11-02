import numpy as np
import pytest

from uavsim.analysis.controller_validation import check_controller_scenario
from uavsim.config.validator import Config


@pytest.mark.parametrize(
    "name",
    [
        "hover",
        "cruise",
        "velocity_step",
        "height_recovery",
        "acceleration_deceleration",
        "altitude_change",
        "lateral_motion",
        "roll",
        "pitch",
        "yaw",
    ],
)
def test_controlled_flight_acceptance(name):
    report = check_controller_scenario(name)
    assert report["passed"], report


def test_small_angle_rate_motor_poles():
    config = Config()
    c = config.controller
    tau = config.vehicle.motor_time_constant_s
    for attitude, rate, integral in zip(c.kp_attitude_per_s, c.kp_rate_per_s, c.ki_rate_per_s2):
        # 状态 [angle,body_rate,torque/J,rate_integral]；小角度、未饱和假设。
        matrix = np.array(
            [
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [-rate * attitude / tau, -rate / tau, -1 / tau, integral / tau],
                [-attitude, -1, 0, 0],
            ]
        )
        assert np.max(np.linalg.eigvals(matrix).real) < 0
