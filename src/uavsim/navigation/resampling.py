import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def resample_positions(
    source_time_s: np.ndarray, source_position_m: np.ndarray, target_time_s: np.ndarray
) -> np.ndarray:
    """100→800 Hz 分量线性插值；不重抽噪声，禁止超出导航覆盖区间。"""
    if (
        np.any(np.diff(source_time_s) <= 0)
        or target_time_s[0] < source_time_s[0] - 1e-12
        or target_time_s[-1] > source_time_s[-1] + 1e-12
    ):
        raise ValueError("Navigation time grid must increase and cover SAR interval")
    return np.column_stack(
        [np.interp(target_time_s, source_time_s, source_position_m[:, axis]) for axis in range(3)]
    )


def resample_navigation(packet: dict, target_time_s: np.ndarray) -> dict:
    """位置/速度线性，姿态 SLERP；均只使用原始 100 Hz 导航解，无新增随机抽样。"""
    source = packet["time_s"]
    position = resample_positions(source, packet["position_sar_m"], target_time_s)
    velocity = resample_positions(source, packet["velocity_sar_mps"], target_time_s)
    rotation = Slerp(source, Rotation.from_quat(packet["q_NB_wxyz"], scalar_first=True))(target_time_s)
    return {
        "time_s": target_time_s,
        "position_sar_m": position,
        "velocity_sar_mps": velocity,
        "rotation_NB": rotation.as_matrix(),
        "q_NB_wxyz": rotation.as_quat(scalar_first=True),
        "euler_NB_rpy_rad": rotation.as_euler("xyz"),
        "resampling": "100 Hz positions/velocities linear; attitude SLERP; no additional random noise",
    }
