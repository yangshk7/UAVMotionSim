"""刚性安装天线：位置 m、速度 m/s、加速度 m/s²，内部 NED/FRD。"""

import numpy as np


def antenna_geometry(
    position_n: np.ndarray,
    velocity_n: np.ndarray,
    rotation_nb: np.ndarray,
    omega_b: np.ndarray,
    lever_b: np.ndarray,
    depression_deg: float,
    acceleration_n: np.ndarray | None = None,
    angular_acceleration_b: np.ndarray | None = None,
    squint_deg: float = 0.0,
) -> dict:
    """支持单状态或批量；速度包括 omega×lever，不能只复制质心速度。"""
    lever = np.asarray(lever_b)
    dep, squint = np.deg2rad([depression_deg, squint_deg])
    beam_b = np.array([np.cos(dep) * np.sin(squint), np.cos(dep) * np.cos(squint), np.sin(dep)])

    def rotate(v: np.ndarray) -> np.ndarray:
        return np.einsum("...ij,...j->...i", rotation_nb, v)

    output = {
        "apc_position_N_m": position_n + rotate(lever),
        "apc_velocity_N_mps": velocity_n + rotate(np.cross(omega_b, lever)),
        "beam_N_unit": rotate(beam_b),
    }
    if acceleration_n is not None and angular_acceleration_b is not None:
        relative = np.cross(angular_acceleration_b, lever) + np.cross(omega_b, np.cross(omega_b, lever))
        output["apc_acceleration_N_mps2"] = acceleration_n + rotate(relative)
    return output
