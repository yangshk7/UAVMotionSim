import numpy as np
from scipy.interpolate import CubicHermiteSpline, CubicSpline


def evaluate_table(
    time_s: np.ndarray, times_s: np.ndarray, positions_m: np.ndarray, velocities_mps: np.ndarray | None = None
) -> tuple:
    """C2 自然三次样条或 C1 Hermite；区间外按端点速度线性延拓，避免预稳定外推发散。"""
    spline = (
        CubicSpline(times_s, positions_m, axis=0, bc_type="natural")
        if velocities_mps is None
        else CubicHermiteSpline(times_s, positions_m, velocities_mps, axis=0)
    )
    t = np.clip(time_s, times_s[0], times_s[-1])
    p, v, a = spline(t), spline(t, 1), spline(t, 2)
    for boundary, mask in [(times_s[0], time_s < times_s[0]), (times_s[-1], time_s > times_s[-1])]:
        p[mask] = spline(boundary) + (time_s[mask] - boundary)[:, None] * spline(boundary, 1)
        v[mask] = spline(boundary, 1)
        a[mask] = 0.0
    return p, v, a
