import numpy as np


def translation_command(splines: list, time_s: np.ndarray) -> tuple:
    """X 速度扰动积分为位置；Y/Z 位置扰动微分为速度/加速度。全部外部 SAR 坐标。"""
    t = np.asarray(time_s)
    p = np.zeros((len(t), 3))
    v = np.zeros_like(p)
    a = np.zeros_like(p)
    for axis, spline in enumerate(splines):
        if spline is None:
            continue
        if axis == 0:
            integral = spline.antiderivative()
            p[:, axis] = integral(t) - integral(spline.x[0])
            v[:, axis] = spline(t)
            a[:, axis] = spline(t, 1)
        else:
            p[:, axis] = spline(t)
            v[:, axis] = spline(t, 1)
            a[:, axis] = spline(t, 2)
    return p, v, a
