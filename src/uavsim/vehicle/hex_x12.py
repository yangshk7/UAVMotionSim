import numpy as np

from uavsim.config.validator import VehicleConfig


class HexX12:
    """30° 起、间隔 60°；FRD 推力 -Z；反扭矩符号表示机体 +Z。"""

    def __init__(self, config: VehicleConfig):
        self.config = config
        angles = np.deg2rad(30 + 60 * np.arange(6))
        self.rotor_positions_b_m = config.arm_length_m * np.column_stack(
            (np.cos(angles), np.sin(angles), np.zeros(6))
        )
        self.reaction_signs = np.array([1, -1, 1, -1, 1, -1])
        # r×[0,0,-T] = [-y*T, x*T, 0]；不可沿用 FLU 推力正号。
        self.allocation_matrix = np.vstack(
            (
                np.ones(6),
                -self.rotor_positions_b_m[:, 1],
                self.rotor_positions_b_m[:, 0],
                self.reaction_signs * config.km / config.kf,
            )
        )
        self.inertia = np.asarray(config.inertia_kg_m2)
        self.inv_inertia = np.linalg.inv(self.inertia)
        self.max_speed_radps = np.sqrt(config.max_rotor_thrust_N / config.kf)
        self.hover_speed_radps = np.sqrt(config.mass_kg * config.gravity_mps2 / (6 * config.kf))
