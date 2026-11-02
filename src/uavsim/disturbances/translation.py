import numpy as np


def translation_command(splines: list, time_s: np.ndarray) -> tuple:
    """三轴统一扰动 SAR 位置目标；连续样条的一、二阶导数提供速度和加速度前馈。"""
    t = np.asarray(time_s)
    p = np.zeros((len(t), 3))
    v = np.zeros_like(p)
    a = np.zeros_like(p)
    for axis, spline in enumerate(splines):
        if spline is None:
            continue
        p[:, axis] = spline(t)
        v[:, axis] = spline(t, 1)
        a[:, axis] = spline(t, 2)
    return p, v, a
