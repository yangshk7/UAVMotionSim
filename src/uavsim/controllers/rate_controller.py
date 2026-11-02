import numpy as np


class RateController:
    def __init__(self, config, inertia):
        self.config = config
        self.inertia = inertia
        self.integral = np.zeros(3)

    def update(self, command, omega_b, dt_s, actuator_saturated=False):
        c = self.config
        error = command - omega_b
        candidate = np.clip(
            self.integral + error * dt_s, -np.array(c.rate_integral_limit_rad), c.rate_integral_limit_rad
        )
        torque = self.inertia @ (np.array(c.kp_rate_per_s) * error + np.array(c.ki_rate_per_s2) * candidate)
        torque += np.cross(omega_b, self.inertia @ omega_b)
        limited = np.clip(torque, -np.array(c.max_torque_Nm), c.max_torque_Nm)
        if not actuator_saturated and np.allclose(torque, limited, rtol=0, atol=1e-12):
            self.integral = candidate
        return limited
