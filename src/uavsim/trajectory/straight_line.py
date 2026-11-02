import numpy as np


def straight_line(time_s: np.ndarray, speed_mps: float, height_m: float, center_time_s: float = 0.0) -> tuple:
    t = np.asarray(time_s)
    position = np.column_stack((speed_mps * (t - center_time_s), np.zeros_like(t), np.full_like(t, height_m)))
    velocity = np.zeros_like(position)
    velocity[:, 0] = speed_mps
    return position, velocity, np.zeros_like(position)
