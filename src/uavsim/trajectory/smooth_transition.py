import numpy as np


def quintic_blend(u: np.ndarray) -> np.ndarray:
    """0→1 五次过渡，端点一二阶导数为零。"""
    u = np.clip(u, 0, 1)
    return u * u * u * (10 + u * (-15 + 6 * u))
