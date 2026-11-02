import numpy as np
from scipy.interpolate import CubicHermiteSpline, CubicSpline

from uavsim.config.validator import Config
from uavsim.frames.coordinate import polar_to_sar
from uavsim.frames.rotation import euler_to_matrix

from .straight_line import straight_line
from .waypoint import evaluate_table


def build_reference(config: Config, time_s: np.ndarray, baseline_sar_m=(0.0, 0.0, 0.0)) -> dict:
    """名义 APC 轨迹及对应 CG 参考；时间 s、位置 m、速度 m/s、加速度 m/s²。"""
    tr = config.trajectory
    t = np.asarray(time_s)
    if tr.type == "straight_constant_velocity":
        p, v, a = straight_line(
            t,
            tr.speed_mps,
            tr.apc_height_m,
            config.simulation.acquisition_duration_s / 2 if tr.center_aperture else 0.0,
        )
    elif tr.type == "segmented_velocity":
        knots = np.asarray(tr.times_s)
        velocities = np.asarray(tr.velocities_sar_mps)
        vs = CubicHermiteSpline(knots, velocities, np.zeros_like(velocities), axis=0)
        ps = vs.antiderivative()
        clip = np.clip(t, knots[0], knots[-1])
        p = ps(clip) - ps(knots[0]) + [0.0, 0.0, tr.apc_height_m]
        v = vs(clip)
        a = vs(clip, 1)
        for boundary, mask in [(knots[0], t < knots[0]), (knots[-1], t > knots[-1])]:
            p[mask] += (t[mask] - boundary)[:, None] * vs(boundary)
            a[mask] = 0
    else:
        p, v, a = evaluate_table(
            t,
            np.asarray(tr.times_s),
            np.asarray(tr.positions_sar_m),
            np.asarray(tr.velocities_sar_mps) if tr.velocities_sar_mps else None,
        )
    p += np.asarray(baseline_sar_m)
    limits = config.controller
    if np.max(np.linalg.norm(v, axis=1)) > limits.max_speed_mps + 1e-9:
        raise ValueError("Reference table exceeds configured speed limit")
    if np.max(np.linalg.norm(a[:, :2], axis=1)) > limits.max_horizontal_acceleration_mps2 + 1e-9:
        raise ValueError("Reference table exceeds configured horizontal acceleration limit")
    if np.max(np.abs(a[:, 2])) > limits.max_vertical_acceleration_mps2 + 1e-9:
        raise ValueError("Reference table exceeds configured vertical acceleration limit")
    yaw = np.full(len(t), np.deg2rad(tr.yaw_deg))
    yaw_rate = np.zeros(len(t))
    yaw_acc = np.zeros(len(t))
    if tr.yaw_series_deg:
        ys = CubicSpline(tr.times_s, np.unwrap(np.deg2rad(tr.yaw_series_deg)), bc_type=((1, 0.0), (1, 0.0)))
        clip = np.clip(t, tr.times_s[0], tr.times_s[-1])
        yaw = ys(clip)
        valid = (t >= tr.times_s[0]) & (t <= tr.times_s[-1])
        yaw_rate[valid] = ys(t[valid], 1)
        yaw_acc[valid] = ys(t[valid], 2)
    rotation = euler_to_matrix(np.column_stack((np.zeros(len(t)), np.zeros(len(t)), yaw)))
    lever = np.asarray(config.antenna.phase_center_frd_m)

    def rotate(x: np.ndarray) -> np.ndarray:
        return np.einsum("nij,nj->ni", rotation, x)

    lever_world = np.einsum("nij,j->ni", rotation, lever)
    omega = np.column_stack((np.zeros(len(t)), np.zeros(len(t)), yaw_rate))
    alpha = np.column_stack((np.zeros(len(t)), np.zeros(len(t)), yaw_acc))
    return {
        "time_s": t,
        "apc_position_sar_m": p,
        "apc_velocity_sar_mps": v,
        "apc_acceleration_sar_mps2": a,
        "position_N_m": polar_to_sar(p) - lever_world,
        "velocity_N_mps": polar_to_sar(v) - rotate(np.cross(omega, lever)),
        "acceleration_N_mps2": polar_to_sar(a)
        - rotate(np.cross(alpha, lever) + np.cross(omega, np.cross(omega, lever))),
        "rotation_NB": rotation,
        "omega_B_radps": omega,
        "angular_acceleration_B_radps2": alpha,
        "yaw_rad": yaw,
        "yaw_rate_radps": yaw_rate,
    }
