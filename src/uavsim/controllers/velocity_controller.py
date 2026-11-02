import numpy as np


class VelocityController:
    """离散 PI，饱和回算采用条件积分；积分量单位 m。"""

    def __init__(self, config, gravity_mps2):
        self.config = config
        self.integral = np.zeros(3)
        self.gravity = gravity_mps2

    def update(self, command, velocity, acceleration_feedforward, dt_s, actuator_saturated=False):
        c = self.config
        error = command - velocity
        candidate = np.clip(
            self.integral + dt_s * error, -np.array(c.velocity_integral_limit_m), c.velocity_integral_limit_m
        )
        raw = (
            acceleration_feedforward
            + np.array(c.kp_velocity_per_s) * error
            + np.array(c.ki_velocity_per_s2) * candidate
        )
        limited = raw.copy()
        limited[2] = np.clip(raw[2], -c.max_vertical_acceleration_mps2, c.max_vertical_acceleration_mps2)
        max_horizontal = min(
            c.max_horizontal_acceleration_mps2,
            (self.gravity - limited[2]) * np.tan(np.deg2rad(c.max_tilt_deg)),
        )
        limited[:2] *= min(1.0, max_horizontal / max(np.linalg.norm(limited[:2]), 1e-12))
        if not actuator_saturated:
            accept = (np.abs(raw - limited) < 1e-12) | ((raw - limited) * error < 0)
            self.integral[accept] = candidate[accept]
        return limited


def desired_attitude(acceleration_n, yaw_rad, mass_kg, gravity_mps2):
    """NED 下 F=m(a-ge3)，令机体 +Z 指向 -F。输出 R_NB、推力向量。"""
    force = mass_kg * (acceleration_n - np.array([0.0, 0.0, gravity_mps2]))
    b3 = -force / np.linalg.norm(force)
    heading = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0])
    b2 = np.cross(b3, heading)
    b2 /= np.linalg.norm(b2)
    b1 = np.cross(b2, b3)
    return np.column_stack((b1, b2, b3)), force
