import numpy as np
import pytest
from numpy.testing import assert_allclose, assert_array_equal
from scipy.io import loadmat
from scipy.spatial.transform import Rotation
from test_navigation import short_config

from uavsim.config.validator import Config
from uavsim.dynamics.simulator import simulate_pass
from uavsim.experiments.two_pass import run_experiment
from uavsim.sar_adapter.matlab_export import export_experiment


def pose_config():
    data = short_config().model_dump()
    data["disturbances"]["navigation_position_error"]["enabled"] = True
    data["disturbances"]["navigation_velocity_error"]["enabled"] = True
    data["disturbances"]["navigation_attitude_error"]["enabled"] = True
    return Config.model_validate(data)


@pytest.mark.parametrize('imu_gnss', [False, True])
def test_pose_errors_and_truth_isolation(imu_gnss):
    cfg = pose_config()
    cfg.disturbances.imu_gnss.enabled = imu_gnss
    data = simulate_pass(cfg, 123)
    base = simulate_pass(short_config(), 123)
    for field in ["position_N_m", "velocity_N_mps", "q_NB_wxyz", "omega_B_radps"]:
        assert_array_equal(data["truth_full"][field], base["truth_full"][field])
    nav, truth = data["navigation"], data["navigation_truth"]
    assert np.linalg.norm(nav["velocity_error_sar_mps"]) > 0
    assert np.linalg.norm(nav["attitude_error_rotvec_B_rad"]) > 0
    assert_allclose(
        nav["velocity_sar_mps"] - truth["velocity_sar_mps"], nav["velocity_error_sar_mps"], atol=1e-14
    )
    actual = np.swapaxes(truth["rotation_NB"], -1, -2) @ nav["rotation_NB"]
    expected = Rotation.from_rotvec(nav["attitude_error_rotvec_B_rad"]).as_matrix()
    assert_allclose(actual, expected, atol=1e-14)
    assert_allclose(np.linalg.norm(nav["q_NB_wxyz"], axis=1), 1.0, atol=1e-14)
    assert np.all(np.sum(nav["q_NB_wxyz"][1:] * nav["q_NB_wxyz"][:-1], axis=1) > 0)
    assert_allclose(data["navigation_sar"]["apc_position_sar_m"][::8], nav["apc_position_sar_m"], atol=1e-12)
    assert_allclose(data["navigation_sar"]["rotation_NB"][::8], nav["rotation_NB"], atol=1e-14)


def test_error_channels_do_not_shift_other_streams():
    cfg = pose_config()
    first = simulate_pass(cfg, 123)
    data = cfg.model_dump()
    data["disturbances"]["navigation_attitude_error"]["random_seed"] = 567
    second = simulate_pass(Config.model_validate(data), 123)
    for field in ["position_sar_m", "velocity_sar_mps"]:
        assert_array_equal(first["navigation"][field], second["navigation"][field])
    assert not np.array_equal(first["navigation"]["q_NB_wxyz"], second["navigation"]["q_NB_wxyz"])


def test_primary_navigation_exports(tmp_path):
    result = run_experiment(pose_config(), 1, {"workers": 1})
    export_experiment(result, tmp_path / "output")
    nav = loadmat(tmp_path / "output/navigation_pass1.mat", simplify_cells=True)["navigation"]
    assert nav["q_NB_wxyz"].shape == (21, 4)
    csv = np.genfromtxt(tmp_path / "output/navigation_pass1.csv", delimiter=",", names=True)
    assert len(csv) == 21
    assert "roll_ned_deg" in csv.dtype.names and "vx_sar_mps" in csv.dtype.names
    assert_allclose(csv["vx_sar_mps"], nav["velocity_sar_mps"][:, 0])
