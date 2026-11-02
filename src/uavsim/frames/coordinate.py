"""NED/FRD 与外部 SAR 的 Z 反射；极向量和轴向量须区别对待。"""

import numpy as np

C = np.diag([1.0, 1.0, -1.0])


def polar_to_sar(vector: np.ndarray) -> np.ndarray:
    """位置/速度/加速度 (...,3)，返回 X 前、Y 右、Z 上的分量。"""
    return np.asarray(vector) * [1.0, 1.0, -1.0]


def axial_to_sar(vector: np.ndarray) -> np.ndarray:
    """角速度等轴向量在反射坐标中使用 det(C) C。"""
    return np.asarray(vector) * [-1.0, -1.0, 1.0]


def rotation_to_sar(rotation_nb: np.ndarray) -> np.ndarray:
    """两端都反射：C R_NB C；保持 proper rotation，禁止翻单个四元数分量。"""
    return C @ np.asarray(rotation_nb) @ C
