import numpy as np
from scipy.optimize import lsq_linear

from .hex_x12 import HexX12


class ControlAllocator:
    """4×6 受限加权最小二乘。可行时走预计算伪逆；饱和时解有界问题。"""

    def __init__(self, vehicle: HexX12, weights=(1.0, 2.0, 2.0, 4.0)):
        self.matrix = vehicle.allocation_matrix
        self.maximum = vehicle.config.max_rotor_thrust_N
        self.weights = np.asarray(weights)
        self.weighted = self.weights[:, None] * self.matrix
        self.inverse = np.linalg.pinv(self.weighted)

    def allocate(self, wrench: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """输入 [总推力 N, roll/pitch/yaw 力矩 N m]；输出各旋翼推力和分配残差。"""
        desired = np.asarray(wrench, dtype=float)
        if desired.shape != (4,) or not np.all(np.isfinite(desired)):
            raise ValueError("Finite 4-component wrench required")
        thrust = self.inverse @ (desired * self.weights)
        if np.min(thrust) < 0 or np.max(thrust) > self.maximum:
            thrust = lsq_linear(
                self.weighted, desired * self.weights, bounds=(0, self.maximum), tol=1e-11, method="bvls"
            ).x
        thrust = np.clip(thrust, 0, self.maximum)
        return thrust, self.matrix @ thrust - desired
