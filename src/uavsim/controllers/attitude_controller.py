import numpy as np
from scipy.spatial.transform import Rotation


class AttitudeController:
    def __init__(self, config):
        self.gain = np.asarray(config.kp_attitude_per_s)
        self.maximum = config.max_body_rate_radps

    def update(self, rotation_nb, desired_rotation_nb, desired_rate_b):
        # SO(3) 对数误差位于当前机体系；不把欧拉角逐轴相减。
        error = Rotation.from_matrix(rotation_nb.T @ desired_rotation_nb).as_rotvec()
        command = self.gain * error + rotation_nb.T @ desired_rotation_nb @ desired_rate_b
        return np.clip(command, -self.maximum, self.maximum)
