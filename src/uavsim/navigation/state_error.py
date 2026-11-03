"""导航速度与姿态估计误差；独立于真实运动与位置误差。"""

import numpy as np
from scipy.spatial.transform import Rotation

from uavsim.disturbances.ou_process import ou_process
from uavsim.frames.coordinate import polar_to_sar
from uavsim.frames.quaternion import from_matrix, to_matrix
from uavsim.frames.rotation import matrix_to_euler


def _vector_error(
    config, block, count: int, pass_seed: int, category: int, bias_std: np.ndarray, noise_std: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    bias = np.zeros((count, 3))
    noise = np.zeros_like(bias)
    if block.enabled or config.disturbances.all_enabled:
        for axis in range(3):
            stream = np.random.SeedSequence(
                [config.base_seed, pass_seed, category, axis, block.random_seed or 0]
            )
            slow, white = stream.spawn(2)
            bias[:, axis] = bias_std[axis] * ou_process(
                count,
                1 / config.simulation.navigation_rate_hz,
                block.correlation_time_s,
                np.random.default_rng(slow),
            )
            noise[:, axis] = noise_std[axis] * np.random.default_rng(white).normal(size=count)
    return bias + noise, bias, noise


def navigation_pose(
    config, position_estimate: dict, sampled_truth: dict, pass_seed: int
) -> tuple[dict, dict]:
    """100 Hz 位姿记录；返回带误差导航和独立真值。

    R_est=R_true Exp(delta_theta_B)，delta 是 FRD 切空间小旋转向量，非欧拉角误差。
    位置/速度误差使用 SAR 局部分量。角速度和加速度尚未建立传感器误差，明确保留 truth 前缀。
    """
    count = len(position_estimate["time_s"])
    d = config.disturbances
    velocity_error, velocity_bias, velocity_noise = _vector_error(
        config,
        d.navigation_velocity_error,
        count,
        pass_seed,
        4,
        np.asarray(d.navigation_velocity_error.bias_std_sar_mps),
        np.asarray(d.navigation_velocity_error.noise_std_sar_mps),
    )
    attitude_error, attitude_bias, attitude_noise = _vector_error(
        config,
        d.navigation_attitude_error,
        count,
        pass_seed,
        5,
        np.deg2rad(d.navigation_attitude_error.bias_std_body_deg),
        np.deg2rad(d.navigation_attitude_error.noise_std_body_deg),
    )
    rotation = to_matrix(sampled_truth["q_NB_wxyz"])
    estimated_rotation = rotation @ Rotation.from_rotvec(attitude_error).as_matrix()
    estimated_q = from_matrix(estimated_rotation)
    # 保持序列四元数符号连续，避免等价 q/-q 在导航日志中跳变。
    signs = np.ones(count)
    signs[1:] = np.cumprod(np.where(np.sum(estimated_q[1:] * estimated_q[:-1], axis=1) < 0, -1.0, 1.0))
    estimated_q *= signs[:, None]
    truth = {
        "time_s": position_estimate["time_s"],
        "sample_index": np.arange(count, dtype=np.int64),
        "position_sar_m": polar_to_sar(sampled_truth["position_N_m"]),
        "position_N_m": sampled_truth["position_N_m"],
        "velocity_sar_mps": polar_to_sar(sampled_truth["velocity_N_mps"]),
        "velocity_N_mps": sampled_truth["velocity_N_mps"],
        "q_NB_wxyz": sampled_truth["q_NB_wxyz"],
        "rotation_NB": rotation,
        "euler_NB_rpy_rad": matrix_to_euler(rotation),
        "truth_acceleration_sar_mps2": polar_to_sar(sampled_truth["acceleration_N_mps2"]),
        "truth_omega_B_radps": sampled_truth["omega_B_radps"],
        "position_error_sar_m": np.zeros((count, 3)),
        "velocity_error_sar_mps": np.zeros((count, 3)),
        "attitude_error_rotvec_B_rad": np.zeros((count, 3)),
        "label": "navigation_rate_truth",
        "reference_point": "CG",
        "position_frame": "local SAR X-forward Y-right Z-up",
        "attitude_frame": "R_NB body FRD to local NED; quaternion wxyz",
    }
    truth["euler_NB_rpy_deg"] = np.rad2deg(truth["euler_NB_rpy_rad"])
    nav = {
        **truth,
        **position_estimate,
        "position_error_sar_m": position_estimate["error_sar_m"],
        "position_N_m": polar_to_sar(position_estimate["position_sar_m"]),
        "velocity_sar_mps": truth["velocity_sar_mps"] + velocity_error,
        "velocity_error_sar_mps": velocity_error,
        "velocity_bias_sar_mps": velocity_bias,
        "velocity_noise_sar_mps": velocity_noise,
        "q_NB_wxyz": estimated_q,
        "rotation_NB": estimated_rotation,
        "euler_NB_rpy_rad": matrix_to_euler(estimated_rotation),
        "attitude_error_rotvec_B_rad": attitude_error,
        "attitude_bias_rotvec_B_rad": attitude_bias,
        "attitude_noise_rotvec_B_rad": attitude_noise,
        "label": "simulated_navigation_solution",
        "channel_sources": {
            "position": "truth plus independent configured position error",
            "velocity": "truth plus independent configured velocity error",
            "attitude": "truth right-multiplied by configured SO(3) error",
            "truth_omega_B_radps": "ideal truth; gyro error not modeled",
            "truth_acceleration_sar_mps2": "ideal truth; accelerometer error not modeled",
        },
        "estimator_model": "Independent statistical errors; not a GNSS/IMU fusion filter",
    }
    if d.imu_gnss.enabled:
        from uavsim.navigation.imu_gnss import simulate_errors

        errors, angles, estimated_rotation, sigmas, updates = simulate_errors(
            config, sampled_truth, rotation, pass_seed
        )
        nav.update(
            position_sar_m=truth['position_sar_m'] + polar_to_sar(errors[:, :3]),
            position_N_m=truth['position_N_m'] + errors[:, :3],
            position_error_sar_m=polar_to_sar(errors[:, :3]),
            error_sar_m=polar_to_sar(errors[:, :3]),
            velocity_sar_mps=truth['velocity_sar_mps'] + polar_to_sar(errors[:, 3:6]),
            velocity_error_sar_mps=polar_to_sar(errors[:, 3:6]),
            attitude_error_rotvec_B_rad=angles,
            rotation_NB=estimated_rotation,
            q_NB_wxyz=from_matrix(estimated_rotation),
            euler_NB_rpy_rad=matrix_to_euler(estimated_rotation),
            ins_error_state=errors,
            ins_error_std=sigmas,
            gnss_update=updates,
            estimator_model='Local NED 15-state linearized GNSS/INS error-state Kalman simulation',
        )
        # 旧独立输出噪声分量不再具有物理意义，避免报告误读。
        for key in ('velocity_bias_sar_mps', 'velocity_noise_sar_mps',
                    'attitude_bias_rotvec_B_rad', 'attitude_noise_rotvec_B_rad',
                    'bias_sar_m', 'noise_sar_m'):
            nav.pop(key, None)
        nav['channel_sources'].update(
            position='IMU error propagation with GNSS correction',
            velocity='IMU error propagation with GNSS correction',
            attitude='gyro/accelerometer error coupling with GNSS and optional heading correction',
        )
    nav["velocity_N_mps"] = polar_to_sar(nav["velocity_sar_mps"])
    nav["euler_NB_rpy_deg"] = np.rad2deg(nav["euler_NB_rpy_rad"])
    lever = np.asarray(config.antenna.phase_center_frd_m)
    for packet in [truth, nav]:
        packet["apc_position_sar_m"] = packet["position_sar_m"] + polar_to_sar(
            np.einsum("nij,j->ni", packet["rotation_NB"], lever)
        )
    return nav, truth
