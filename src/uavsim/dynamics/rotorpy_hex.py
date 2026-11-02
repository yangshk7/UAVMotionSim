"""将 RotorPy 3.0 的多旋翼动力学用于本项目的 NED/FRD Hex-X 状态。"""

import numpy as np
from rotorpy.vehicles.multirotor import Multirotor

from uavsim.frames.quaternion import to_matrix
from uavsim.vehicle.hex_x12 import HexX12

_FLIP = np.array([1.0, -1.0, -1.0])  # NED/FRD -> RotorPy 的 Z-up 坐标约定


class RotorPyHexDynamics:
    """RotorPy 负责刚体、旋翼与电机 RHS；本层只做坐标和外扰适配。

    RotorPy 3.0 构造器对 4×N 分配矩阵求方阵逆。因此先用合法四旋翼
    初始化，再配置六旋翼几何；直接电机转速指令不会调用其分配逆矩阵。
    本项目的有界 4×6 分配器仍负责生成六个电机指令。
    """

    def __init__(self, vehicle: HexX12):
        self.vehicle = vehicle
        cfg = vehicle.config
        inertia = (_FLIP[:, None] * vehicle.inertia) * _FLIP[None, :]
        radius = cfg.arm_length_m / np.sqrt(2)
        quad = {
            f"r{i + 1}": np.array([x * radius, y * radius, 0.0])
            for i, (x, y) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1)))
        }
        params = {
            "mass": cfg.mass_kg,
            "Ixx": inertia[0, 0],
            "Iyy": inertia[1, 1],
            "Izz": inertia[2, 2],
            "Ixy": inertia[0, 1],
            "Ixz": inertia[0, 2],
            "Iyz": inertia[1, 2],
            "num_rotors": 4,
            "rotor_pos": quad,
            "rotor_directions": np.array([1, -1, 1, -1]),
            "rotor_speed_min": 0.0,
            "rotor_speed_max": vehicle.max_speed_radps,
            "k_eta": cfg.kf,
            "k_m": cfg.km,
            "tau_m": cfg.motor_time_constant_s,
            "motor_noise_std": 0.0,
        }
        self.model = Multirotor(params, control_abstraction="cmd_motor_speeds", aero=False)
        positions = vehicle.rotor_positions_b_m * _FLIP
        self.model.rotor_pos = {f"r{i + 1}": p for i, p in enumerate(positions)}
        self.model.num_rotors = 6
        self.model.rotor_dir = -vehicle.reaction_signs
        self.model.extract_geometry()
        self.model.f_to_TM = np.vstack(
            (
                np.ones(6),
                positions[:, 1],
                -positions[:, 0],
                self.model.rotor_dir * cfg.km / cfg.kf,
            )
        )
        self.model.TM_to_f = np.linalg.pinv(self.model.f_to_TM)
        self.drag = np.asarray(cfg.drag_cd_area_m2) * (0.5 * cfg.air_density_kg_m3)

    def derivative(
        self, state: np.ndarray, motor_command: np.ndarray, force_b: np.ndarray, torque_b: np.ndarray
    ) -> np.ndarray:
        """输入/输出为项目 19 维 NED/FRD 状态及 SI 单位；无随机抽样。"""
        cfg = self.vehicle.config
        rotor_state = np.empty(22)
        rotor_state[:3] = state[:3] * _FLIP
        rotor_state[3:6] = state[3:6] * _FLIP
        rotor_state[6:10] = [state[7], -state[8], -state[9], state[6]]  # wxyz -> xyzw
        rotor_state[10:13] = state[10:13] * _FLIP
        rotor_state[13:16] = 0.0  # 无风
        rotor_state[16:] = state[13:]
        command = np.clip(motor_command, 0, self.vehicle.max_speed_radps)
        derivative = self.model._s_dot_fn(0.0, rotor_state, command)
        output = np.empty(19)
        output[:3] = derivative[:3] * _FLIP
        output[3:6] = derivative[3:6] * _FLIP
        output[6:10] = [derivative[9], derivative[6], -derivative[7], -derivative[8]]
        output[10:13] = derivative[10:13] * _FLIP
        output[13:] = derivative[16:]
        rotation = to_matrix(state[6:10])
        output[3:6] += (rotation @ force_b) / cfg.mass_kg
        output[3:6] += np.array([0.0, 0.0, cfg.gravity_mps2 - self.model.g])
        output[10:13] += self.vehicle.inv_inertia @ torque_b
        if cfg.drag_enabled:
            velocity_b = rotation.T @ state[3:6]
            output[3:6] -= rotation @ (self.drag * np.abs(velocity_b) * velocity_b) / cfg.mass_kg
        return output
