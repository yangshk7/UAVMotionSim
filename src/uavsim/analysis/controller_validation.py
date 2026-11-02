"""控制器整定用可控仿真，独立于随机扰动试验。"""

import numpy as np

from uavsim.config.validator import Config
from uavsim.dynamics.simulator import simulate_pass


def controller_scenario(name: str) -> Config:
    data = {
        "simulation": {"dynamics_rate_hz": 1600, "pre_stabilization_s": 0.5, "acquisition_duration_s": 10.0}
    }
    data["trajectory"] = {}
    if name == "hover":
        data["trajectory"]["speed_mps"] = 0.0
    elif name == "velocity_step":
        data["simulation"]["initial_velocity_error_sar_mps"] = [-1.0, 0.0, 0.0]
    elif name == "height_recovery":
        data["simulation"]["initial_position_error_sar_m"] = [0.0, 0.0, 0.2]
    elif name in ("roll", "pitch", "yaw"):
        angles = [0.0, 0.0, 0.0]
        angles[("roll", "pitch", "yaw").index(name)] = 5.0 if name != "yaw" else 20.0
        data["simulation"]["initial_attitude_error_rpy_deg"] = angles
    elif name == "acceleration_deceleration":
        data["trajectory"].update(
            type="segmented_velocity",
            times_s=[0.0, 1.0, 3.0, 5.0, 8.0, 10.0],
            velocities_sar_mps=[[9, 0, 0], [9, 0, 0], [10, 0, 0], [9, 0, 0], [9, 0, 0], [9, 0, 0]],
        )
    elif name in ("altitude_change", "lateral_motion"):
        positions = [[0.0, 0.0, 300.0], [20.0, 0.0, 300.0], [60.0, 0.0, 300.0], [100.0, 0.0, 300.0]]
        axis = 2 if name == "altitude_change" else 1
        positions[2][axis] += 1.0
        positions[3][axis] += 1.0
        data["trajectory"].update(
            type="time_series",
            times_s=[0.0, 2.0, 6.0, 10.0],
            positions_sar_m=positions,
            velocities_sar_mps=[[10.0, 0.0, 0.0]] * 4,
        )
    elif name != "cruise":
        raise ValueError(f"Unknown validation scenario: {name}")
    return Config.model_validate(data)


def check_controller_scenario(name: str) -> dict:
    config = controller_scenario(name)
    result = simulate_pass(config, 9301)
    truth = result["truth_sar"]
    reference = result["reference"]
    tail = truth["time_s"] >= 9.0
    velocity_error = np.linalg.norm(truth["velocity_N_mps"][tail] - reference["velocity_N_mps"][tail], axis=1)
    height_error = np.abs(truth["position_N_m"][tail, 2] - reference["position_N_m"][tail, 2])
    report = {
        "scenario": name,
        "steady_velocity_max_error_mps": float(velocity_error.max()),
        "steady_height_max_error_m": float(height_error.max()),
        "tail_velocity_std_mps": np.std(truth["velocity_N_mps"][tail], axis=0).tolist(),
        "rotor_min_radps": float(truth["rotor_speed_radps"].min()),
        "rotor_max_radps": float(truth["rotor_speed_radps"].max()),
        "finite": bool(np.all(np.isfinite(truth["position_N_m"]))),
        "terminal_attitude_error_rad": float(
            np.linalg.norm(truth["euler_NB_rpy_rad"][-1] - [0, 0, np.deg2rad(config.trajectory.yaw_deg)])
        ),
    }
    report["passed"] = bool(
        report["steady_velocity_max_error_mps"] < 0.1
        and report["steady_height_max_error_m"] < 0.05
        and report["finite"]
        and 0 <= report["rotor_min_radps"] <= report["rotor_max_radps"] <= np.sqrt(42 / 8e-5) + 1e-9
        and max(report["tail_velocity_std_mps"]) < 0.1
    )
    return report
