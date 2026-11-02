"""标量在前 wxyz；主动旋转 B→N。"""

import numpy as np
from scipy.spatial.transform import Rotation


def normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(norm < 1e-14) or not np.all(np.isfinite(q)):
        raise ValueError("Quaternion must be finite and nonzero")
    return q / norm


def to_matrix(q: np.ndarray) -> np.ndarray:
    """输入已归一化 wxyz，输出 (...,3,3)。RK4 子步允许轻微模误差。"""
    q = np.asarray(q)
    w, x, y, z = np.moveaxis(q, -1, 0)
    values = [
        1 - 2 * (y * y + z * z),
        2 * (x * y - w * z),
        2 * (x * z + w * y),
        2 * (x * y + w * z),
        1 - 2 * (x * x + z * z),
        2 * (y * z - w * x),
        2 * (x * z - w * y),
        2 * (y * z + w * x),
        1 - 2 * (x * x + y * y),
    ]
    if q.ndim == 1:
        return np.array(values).reshape(3, 3)
    return np.stack(values, axis=-1).reshape(q.shape[:-1] + (3, 3))


def from_matrix(matrix: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(matrix).as_quat(scalar_first=True)


def derivative(q: np.ndarray, omega_b: np.ndarray) -> np.ndarray:
    """qdot=0.5 q⊗[0,omega_B]，omega 单位 rad/s。"""
    w, x, y, z = q
    p, r, s = omega_b
    return 0.5 * np.array(
        [-x * p - y * r - z * s, w * p + y * s - z * r, w * r + z * p - x * s, w * s + x * r - y * p]
    )
