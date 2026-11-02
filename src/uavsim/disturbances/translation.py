import numpy as np


def translation_command(splines: list, time_s: np.ndarray) -> tuple:
    """X 仅扰动速度指令，保留名义位置目标供闭环校正；Y/Z 扰动位置目标。单位为 SAR m、m/s、m/s²。"""
    t = np.asarray(time_s)
    p = np.zeros((len(t), 3))
    v = np.zeros_like(p)
    a = np.zeros_like(p)
    for axis, spline in enumerate(splines):
        if spline is None:
            continue
        if axis == 0:
            v[:, axis] = spline(t)
            a[:, axis] = spline(t, 1)
        else:
            p[:, axis] = spline(t)
            v[:, axis] = spline(t, 1)
            a[:, axis] = spline(t, 2)
    return p, v, a
