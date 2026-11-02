import numpy as np


def attitude_command(splines: list, time_s: np.ndarray) -> tuple:
    """返回 Euler 安装式偏置 rad 及其对应体轴角速度 rad/s，不对欧拉导数直接改名。"""
    angles = np.zeros((len(time_s), 3))
    rates = np.zeros_like(angles)
    for axis, spline in enumerate(splines):
        if spline is not None:
            angles[:, axis] = spline(time_s)
            rates[:, axis] = spline(time_s, 1)
    roll, pitch = angles[:, 0], angles[:, 1]
    rd, pd, yd = rates.T
    body_rates = np.column_stack(
        (
            rd - yd * np.sin(pitch),
            pd * np.cos(roll) + yd * np.sin(roll) * np.cos(pitch),
            -pd * np.sin(roll) + yd * np.cos(roll) * np.cos(pitch),
        )
    )
    return angles, body_rates
