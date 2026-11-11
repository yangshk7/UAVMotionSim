"""SAR 采集段数据契约：高频机体真值 + 独立 100 Hz 导航估计。

位置和速度为局部 SAR X 前、Y 右、Z 上，单位 m 与 m/s；姿态
q_NB_wxyz 为机体 FRD 到局部 NED 的主动四元数。两路时间均从采集开始
计时，外部适配器只有在确认公共时间轴后才能构造 MotionSegment。
"""

from dataclasses import dataclass

import numpy as np

from uavsim.frames.coordinate import polar_to_sar
from uavsim.frames.quaternion import to_matrix
from uavsim.sar_adapter.pulse_sync import sample_indices


@dataclass(frozen=True)
class MotionSeries:
    """同一参考点的位姿序列；time_s 为采集相对秒，数组首维为样本数。"""

    time_s: np.ndarray
    position_sar_m: np.ndarray
    velocity_sar_mps: np.ndarray
    q_NB_wxyz: np.ndarray
    apc_position_sar_m: np.ndarray

    def __post_init__(self) -> None:
        time = np.asarray(self.time_s)
        count = len(time)
        if time.ndim != 1 or count < 2 or not np.all(np.isfinite(time)) or np.any(np.diff(time) <= 0):
            raise ValueError("Motion time must be finite and strictly increasing")
        if abs(time[0]) > 1e-9:
            raise ValueError("Motion time must start at acquisition t=0")
        for name, width in (("position_sar_m", 3), ("velocity_sar_mps", 3),
                            ("q_NB_wxyz", 4), ("apc_position_sar_m", 3)):
            values = np.asarray(getattr(self, name))
            if values.shape != (count, width) or not np.all(np.isfinite(values)):
                raise ValueError(f"Invalid {name} shape or nonfinite values")
        if np.max(np.abs(np.linalg.norm(self.q_NB_wxyz, axis=1) - 1)) > 1e-6:
            raise ValueError("Motion quaternions must have unit norm")


@dataclass(frozen=True)
class MotionSegment:
    """共享采集时轴原点的真值与导航序列；导航不得充当物理真值。"""

    truth: MotionSeries
    navigation: MotionSeries
    source: str
    truth_rate_hz: float
    navigation_rate_hz: float
    clock_alignment: str
    navigation_model: str

    def __post_init__(self) -> None:
        if abs(self.truth.time_s[-1] - self.navigation.time_s[-1]) > 1e-7:
            raise ValueError("Truth and navigation must cover the same acquisition interval")
        if self.truth_rate_hz <= 0 or self.navigation_rate_hz <= 0:
            raise ValueError("Sample rates must be positive")
        for series, rate in ((self.truth, self.truth_rate_hz),
                             (self.navigation, self.navigation_rate_hz)):
            intervals = np.diff(series.time_s)
            if not np.allclose(intervals, 1 / rate, rtol=0, atol=1e-7):
                raise ValueError("Declared rate does not match the time grid")
        if self.clock_alignment != "exact_common_grid":
            raise ValueError("SAR-ready segments require a verified common time grid")


def from_simulated_pass(pass_result: dict, config) -> MotionSegment:
    """取单航次采集段，不含预稳定；真值原生积分率，导航为配置频率。"""
    indices = sample_indices(config)["acquisition"]
    full = pass_result["truth_full"]
    nav = pass_result["navigation"]
    truth_position = np.asarray(full["position_sar_m"])[indices]
    truth_q = np.asarray(full["q_NB_wxyz"])[indices]
    rotation = to_matrix(truth_q)
    lever_n = np.einsum("nij,j->ni", rotation, config.antenna.phase_center_frd_m)
    truth = MotionSeries(
        time_s=np.arange(len(indices), dtype=np.float64) / config.simulation.dynamics_rate_hz,
        position_sar_m=truth_position,
        velocity_sar_mps=polar_to_sar(full["velocity_N_mps"])[indices],
        q_NB_wxyz=truth_q,
        apc_position_sar_m=truth_position + polar_to_sar(lever_n),
    )
    navigation = MotionSeries(
        time_s=np.asarray(nav["time_s"]),
        position_sar_m=np.asarray(nav["position_sar_m"]),
        velocity_sar_mps=np.asarray(nav["velocity_sar_mps"]),
        q_NB_wxyz=np.asarray(nav["q_NB_wxyz"]),
        apc_position_sar_m=np.asarray(nav["apc_position_sar_m"]),
    )
    return MotionSegment(
        truth=truth,
        navigation=navigation,
        source={"rotorpy": "rotorpy", "matlab": "matlab"}.get(config.runtime.backend, "internal_cpu"),
        truth_rate_hz=config.simulation.dynamics_rate_hz,
        navigation_rate_hz=config.simulation.navigation_rate_hz,
        clock_alignment="exact_common_grid",
        navigation_model=("linearized_15_state_gnss_ins" if config.disturbances.imu_gnss.enabled
                          else "independent_configured_position_velocity_attitude_errors"),
    )


def crop_motion_segment(segment: MotionSegment, start_time_s: float, end_time_s: float) -> MotionSegment:
    """按两路共有的样本时刻裁剪航迹，重置时间原点；起止单位 s，端点均包含。"""
    if not np.isfinite(start_time_s) or not np.isfinite(end_time_s) or end_time_s <= start_time_s:
        raise ValueError("Crop times must be finite and increasing")

    def crop(series: MotionSeries) -> MotionSeries:
        start = int(np.searchsorted(series.time_s, start_time_s))
        end = int(np.searchsorted(series.time_s, end_time_s))
        if (start >= len(series.time_s) or end >= len(series.time_s)
                or abs(series.time_s[start] - start_time_s) > 1e-7
                or abs(series.time_s[end] - end_time_s) > 1e-7
                or end <= start):
            raise ValueError("Crop endpoints must lie on both truth and navigation sample grids")
        window = slice(start, end + 1)
        return MotionSeries(
            time_s=series.time_s[window] - series.time_s[start],
            position_sar_m=series.position_sar_m[window],
            velocity_sar_mps=series.velocity_sar_mps[window],
            q_NB_wxyz=series.q_NB_wxyz[window],
            apc_position_sar_m=series.apc_position_sar_m[window],
        )

    return MotionSegment(
        truth=crop(segment.truth),
        navigation=crop(segment.navigation),
        source=segment.source,
        truth_rate_hz=segment.truth_rate_hz,
        navigation_rate_hz=segment.navigation_rate_hz,
        clock_alignment=segment.clock_alignment,
        navigation_model=segment.navigation_model,
    )
