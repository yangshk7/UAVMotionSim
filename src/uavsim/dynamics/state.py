"""19 状态：[p_N m, v_N m/s, q_NB wxyz, omega_B rad/s, rotor rad/s]。"""

import numpy as np


def initial_state(
    position_n: np.ndarray, velocity_n: np.ndarray, quaternion_nb: np.ndarray, rotor_speed_radps: float
) -> np.ndarray:
    return np.concatenate((position_n, velocity_n, quaternion_nb, np.zeros(3), np.full(6, rotor_speed_radps)))
