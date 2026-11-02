"""规范候选初值，未经实机校准。动力学力/力矩幅度不从这些值推算。"""

TRANSLATION = {"small": (0.2, 0.05, 0.03), "medium": (0.5, 0.15, 0.10), "large": (1.0, 0.30, 0.20)}
ATTITUDE_DEG = {"small": (0.5, 0.5, 1.0), "medium": (1.5, 1.5, 3.0), "large": (3.0, 3.0, 5.0)}
NAVIGATION_M = {"small": (0.01, 0.01, 0.02), "medium": (0.05, 0.05, 0.10), "large": (0.20, 0.20, 0.40)}


def factor_scale(factor, group: str, axis: int) -> tuple[float, str]:
    if factor.injection_mode == "dynamics":
        return factor.amplitude, factor.unit
    values = TRANSLATION if group == "translation" else ATTITUDE_DEG
    units = ("m/s", "m", "m") if group == "translation" else ("deg",) * 3
    return (values[factor.level][axis] if factor.amplitude is None else factor.amplitude), units[axis]
