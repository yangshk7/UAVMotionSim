import numpy as np
from scipy.spatial.transform import Rotation


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def euler_to_matrix(rpy_rad: np.ndarray) -> np.ndarray:
    """R=Rz(yaw) Ry(pitch) Rx(roll)，角度 rad；仅用于接口。"""
    return Rotation.from_euler("xyz", rpy_rad).as_matrix()


def matrix_to_euler(matrix: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(matrix).as_euler("xyz")
