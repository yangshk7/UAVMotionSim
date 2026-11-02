import numpy as np
import pytest
from numpy.testing import assert_allclose, assert_array_equal

from uavsim.config.validator import Config
from uavsim.controllers.attitude_controller import AttitudeController
from uavsim.dynamics.simulator import simulate_pass
from uavsim.experiments.two_pass import run_experiment
from uavsim.frames.quaternion import to_matrix
from uavsim.frames.rotation import euler_to_matrix
from uavsim.trajectory.reference_manager import build_reference


def scenario_config():
    return Config.model_validate(
        {"simulation": {"dynamics_rate_hz": 1600, "pre_stabilization_s": 0.1, "acquisition_duration_s": 1.0}}
    )


@pytest.mark.parametrize(
    "name,group",
    [(name, "translation") for name in ["x", "y", "z"]]
    + [(name, "attitude") for name in ["roll", "pitch", "yaw"]],
)
def test_single_factor_response(name, group):
    cfg = scenario_config()
    data = cfg.model_dump()
    data["disturbances"][group][name].update(enabled=True, ramp_s=0.1)
    result = simulate_pass(Config.model_validate(data), 123)
    truth = result["truth_sar"]
    assert np.max(np.abs(result["disturbance_inputs"][name]["values"])) > 1e-5
    assert np.max(np.abs(truth["apc_position_sar_m"] - result["reference"]["apc_position_sar_m"])) > 1e-6
    other = [key for key in result["disturbance_inputs"] if key in ["x", "y", "z", "roll", "pitch", "yaw"]]
    assert other == [name]
    assert np.max(np.abs(np.linalg.norm(truth["q_NB_wxyz"], axis=1) - 1)) < 1e-12


def test_x_velocity_command_position_error_recovers_after_disturbance():
    values = Config().model_dump()
    values["simulation"].update(
        dynamics_rate_hz=800, pre_stabilization_s=0.1, acquisition_duration_s=8.0
    )
    values["disturbances"]["translation"]["x"].update(
        enabled=True, profile="constant", amplitude=0.5, ramp_s=0.1, end_time_s=2.0
    )
    result = simulate_pass(Config.model_validate(values), 123)
    error = (
        result["truth_sar"]["apc_position_sar_m"][:, 0]
        - result["reference"]["apc_position_sar_m"][:, 0]
    )
    time = result["truth_sar"]["time_s"]
    assert error[np.searchsorted(time, 2.0)] > 0.2
    assert abs(error[-1]) < 0.02


def test_joint_seeds_baseline_and_parallel_reproducibility():
    data = scenario_config().model_dump()
    data["simulation"]["acquisition_duration_s"] = 0.3
    data["disturbances"]["translation"]["y"].update(enabled=True, ramp_s=0.05)
    data["disturbances"]["attitude"]["roll"].update(enabled=True, ramp_s=0.05)
    data["nominal_baseline_sar_m"] = [0.0, 0.5, 0.1]
    cfg = Config.model_validate(data)
    serial = run_experiment(cfg, 2, {"workers": 1})
    parallel = run_experiment(cfg, 2, {"workers": 2})
    for name in ["pass1", "pass2"]:
        assert_array_equal(
            serial[name]["truth_full"]["position_N_m"], parallel[name]["truth_full"]["position_N_m"]
        )
        assert_array_equal(serial[name]["truth_full"]["q_NB_wxyz"], parallel[name]["truth_full"]["q_NB_wxyz"])
    first, second = serial["pass1"], serial["pass2"]
    assert not np.array_equal(
        first["disturbance_inputs"]["y"]["values"], second["disturbance_inputs"]["y"]["values"]
    )
    expected = np.broadcast_to([0.0, 0.5, 0.1], first["reference"]["apc_position_sar_m"].shape)
    assert_allclose(
        second["reference"]["apc_position_sar_m"] - first["reference"]["apc_position_sar_m"],
        expected,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    "translation,attitude",
    [("ideal", "ideal"), ("actual", "ideal"), ("ideal", "actual"), ("actual", "actual")],
)
def test_geometry_modes(translation, attitude):
    data = scenario_config().model_dump()
    data["simulation"]["acquisition_duration_s"] = 0.3
    data["geometry"] = {"translation": translation, "attitude": attitude}
    data["disturbances"]["attitude"]["yaw"].update(enabled=True, ramp_s=0.05)
    result = simulate_pass(Config.model_validate(data), 12)
    geometry = result["sar_geometry"]
    p = (
        result["truth_sar"]["position_N_m"]
        if translation == "actual"
        else result["reference"]["position_N_m"]
    )
    r = result["truth_sar"]["rotation_NB"] if attitude == "actual" else result["reference"]["rotation_NB"]
    assert_array_equal(geometry["position_N_m"], p)
    assert_allclose(to_matrix(geometry["q_NB_wxyz"]), r, atol=1e-14)
    assert geometry["label"] == "geometry_decoupled"
    assert not geometry["physical_truth"]


@pytest.mark.parametrize("axis", [0, 1, 2])
def test_so3_command_direction(axis):
    controller = AttitudeController(Config().controller)
    angles = np.zeros(3)
    angles[axis] = 0.1
    command = controller.update(np.eye(3), euler_to_matrix(angles), np.zeros(3))
    assert command[axis] > 0
    assert_allclose(np.delete(command, axis), 0, atol=1e-14)


def test_trajectory_derivatives_and_segments():
    data = scenario_config().model_dump()
    data["trajectory"].update(
        type="segmented_velocity", times_s=[0, 2, 4], velocities_sar_mps=[[9, 0, 0], [10, 0, 0], [9, 0, 0]]
    )
    cfg = Config.model_validate(data)
    t = np.linspace(0.1, 3.9, 2001)
    ref = build_reference(cfg, t)
    dt = t[1] - t[0]
    numeric = np.gradient(ref["apc_position_sar_m"], dt, axis=0)
    assert_allclose(numeric[2:-2], ref["apc_velocity_sar_mps"][2:-2], atol=3e-6)
