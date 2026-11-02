import numpy as np

from uavsim.frames.quaternion import derivative as quaternion_derivative
from uavsim.frames.quaternion import to_matrix
from uavsim.vehicle.hex_x12 import HexX12


class RigidBodyDynamics:
    """无地面接触/风场的空中刚体；外力 FRD N，外力矩 FRD N m。"""

    def __init__(self, vehicle: HexX12):
        self.vehicle = vehicle
        self.config = vehicle.config
        self.gravity = np.array([0.0, 0.0, self.config.gravity_mps2])
        self.drag = np.array(self.config.drag_cd_area_m2) * (0.5 * self.config.air_density_kg_m3)

    def derivative(
        self, state: np.ndarray, motor_command: np.ndarray, force_b: np.ndarray, torque_b: np.ndarray
    ) -> np.ndarray:
        """纯确定性 RHS；调用时不更新控制器、不抽取噪声、不修改输入。"""
        cfg = self.config
        rotation = to_matrix(state[6:10])
        thrust = cfg.kf * state[13:] ** 2
        wrench = self.vehicle.allocation_matrix @ thrust
        body_force = np.array([force_b[0], force_b[1], force_b[2] - wrench[0]])
        if cfg.drag_enabled:
            velocity_b = rotation.T @ state[3:6]
            body_force -= self.drag * np.abs(velocity_b) * velocity_b
        omega = state[10:13]
        angular_momentum = self.vehicle.inertia @ omega
        # 手写小向量叉乘，避免每个 RK4 子步反复调度通用 cross。
        gyro = np.array(
            [
                omega[1] * angular_momentum[2] - omega[2] * angular_momentum[1],
                omega[2] * angular_momentum[0] - omega[0] * angular_momentum[2],
                omega[0] * angular_momentum[1] - omega[1] * angular_momentum[0],
            ]
        )
        output = np.empty(19)
        output[:3] = state[3:6]
        output[3:6] = self.gravity + (rotation @ body_force) / cfg.mass_kg
        output[6:10] = quaternion_derivative(state[6:10], omega)
        output[10:13] = self.vehicle.inv_inertia @ (wrench[1:] + torque_b - gyro)
        output[13:] = (motor_command - state[13:]) / cfg.motor_time_constant_s
        return output
