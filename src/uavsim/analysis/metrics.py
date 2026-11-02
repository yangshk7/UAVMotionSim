import numpy as np

from uavsim.analysis.route_quality import evaluate_route_quality
from uavsim.frames.quaternion import to_matrix


def rms(array: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(array))))


def state_metrics(data: dict, config) -> dict:
    truth = data["truth_full"]
    dt = 1 / config.simulation.dynamics_rate_hz
    sl = slice(2, -2)
    dp = np.gradient(truth["position_N_m"], dt, axis=0, edge_order=2)
    dv = np.gradient(truth["velocity_N_mps"], dt, axis=0, edge_order=2)
    rotation = to_matrix(truth["q_NB_wxyz"])
    dr = np.gradient(rotation, dt, axis=0, edge_order=2)
    omega = truth["omega_B_radps"]
    skew = np.zeros_like(rotation)
    skew[:, 0, 1] = -omega[:, 2]
    skew[:, 0, 2] = omega[:, 1]
    skew[:, 1, 0] = omega[:, 2]
    skew[:, 1, 2] = -omega[:, 0]
    skew[:, 2, 0] = -omega[:, 1]
    skew[:, 2, 1] = omega[:, 0]
    sar = data["truth_sar"]
    expected = np.arange(len(sar["time_s"])) / config.simulation.sar_rate_hz
    residuals = {
        "position_derivative_rms_mps": rms((dp - truth["velocity_N_mps"])[sl]),
        "velocity_derivative_rms_mps2": rms((dv - truth["acceleration_N_mps2"])[sl]),
        "rotation_derivative_rms_per_s": rms((dr - rotation @ skew)[sl]),
        "quaternion_norm_max_error": float(np.max(np.abs(np.linalg.norm(truth["q_NB_wxyz"], axis=1) - 1))),
        "rotation_determinant_max_error": float(np.max(np.abs(np.linalg.det(rotation) - 1))),
        "rotation_orthogonality_max_error": float(
            np.max(np.abs(np.swapaxes(rotation, -1, -2) @ rotation - np.eye(3)))
        ),
        "finite": bool(
            all(np.all(np.isfinite(value)) for value in truth.values() if isinstance(value, np.ndarray))
        ),
        "sar_samples": len(sar["time_s"]),
        "navigation_samples": len(data["navigation"]["time_s"]),
        "truth_full_samples": len(truth["time_s"]),
        "sar_time_exact": bool(np.array_equal(sar["time_s"], expected)),
        "rotor_min_radps": float(truth["rotor_speed_radps"].min()),
        "rotor_max_radps": float(truth["rotor_speed_radps"].max()),
        "allocation_residual_max": float(np.max(np.abs(truth["allocation_residual"]))),
        "apc_position_nominal_difference_rms_m": rms(
            sar["apc_position_sar_m"] - data["reference"]["apc_position_sar_m"]
        ),
    }
    residuals["kinematics_passed"] = bool(
        residuals["position_derivative_rms_mps"] < 1e-3
        and residuals["velocity_derivative_rms_mps2"] < 1e-3
        and residuals["rotation_derivative_rms_per_s"] < 1e-3
    )
    nav = data["navigation"]
    residuals["navigation_pose_samples"] = len(nav["time_s"])
    residuals["navigation_position_error_rms_m"] = rms(nav["position_error_sar_m"])
    residuals["navigation_velocity_error_rms_mps"] = rms(nav["velocity_error_sar_mps"])
    residuals["navigation_attitude_rotvec_error_rms_rad"] = rms(nav["attitude_error_rotvec_B_rad"])
    residuals["navigation_quaternion_norm_max_error"] = float(
        np.max(np.abs(np.linalg.norm(nav["q_NB_wxyz"], axis=1) - 1))
    )
    residuals["passed"] = bool(
        residuals["finite"]
        and residuals["sar_time_exact"]
        and residuals["kinematics_passed"]
        and residuals["quaternion_norm_max_error"] < 1e-12
        and residuals["navigation_quaternion_norm_max_error"] < 1e-12
    )
    if config.simulation.duration_mode == "route_length":
        residuals["route_quality"] = evaluate_route_quality(data, config)
        residuals["passed"] = residuals["passed"] and residuals["route_quality"]["passed"]
    return residuals


def experiment_metrics(result: dict, config) -> dict:
    report = {key: state_metrics(value, config) for key, value in result.items() if key.startswith("pass")}
    if "pass2" in result:
        first = result["pass1"]["truth_sar"]["apc_position_sar_m"]
        second = result["pass2"]["truth_sar"]["apc_position_sar_m"]
        baseline = np.asarray(config.nominal_baseline_sar_m)
        report["two_pass"] = {
            "nominal_baseline_sar_m": baseline.tolist(),
            "actual_difference_rms_m": rms(second - first),
            "difference_after_nominal_baseline_rms_m": rms(second - first - baseline),
        }
    report["passed"] = all(value["passed"] for key, value in report.items() if key.startswith("pass"))
    return report
