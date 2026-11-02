import numpy as np


class PositionController:
    def __init__(self, config):
        self.gain = np.asarray(config.kp_position_per_s)
        self.maximum = config.max_speed_mps

    def update(self, reference_position, reference_velocity, position):
        velocity = reference_velocity + self.gain * (reference_position - position)
        return velocity * min(1.0, self.maximum / max(np.linalg.norm(velocity), 1e-12))
