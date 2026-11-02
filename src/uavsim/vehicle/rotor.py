import numpy as np


def rotor_thrust(speed_radps: np.ndarray, kf: float) -> np.ndarray:
    """速度 rad/s；返回单旋翼推力 N；不允许负转速。"""
    if np.any(np.asarray(speed_radps) < 0):
        raise ValueError("Negative rotor speed")
    return kf * np.square(speed_radps)
