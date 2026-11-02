"""仿真杆臂的坐标符号、姿态旋转和外部导出。"""

import json

import numpy as np
import pytest
from numpy.testing import assert_allclose

from uavsim.config.loader import load_config
from uavsim.motion.lever_arm import apc_from_pose, export_apc_from_aligned


def test_simulation_reference_and_yaw_rotation():
    lever = np.array([0.0, 0.1, 0.2])
    q = np.array([[1.0, 0.0, 0.0, 0.0], [2**-0.5, 0.0, 0.0, 2**-0.5]])
    position = np.array([[0.0, -0.1, 300.2], [0.0, 0.0, 0.0]])
    actual = apc_from_pose(position, q, lever)
    assert_allclose(actual[0], [0.0, 0.0, 300.0], atol=1e-14)
    assert_allclose(actual[1], [-0.1, 0.0, -0.2], atol=1e-14)
    assert_allclose(load_config("main/motion_config.yaml").antenna.phase_center_frd_m, lever)


def test_external_export_preserves_cg_and_rejects_overwrite(tmp_path):
    source = tmp_path / "clock_sync"
    source.mkdir()
    (source / "alignment.json").write_text(json.dumps({"files": {
        "truth": "truth_1000hz.csv", "navigation": "navigation_100hz.csv"
    }}), encoding="utf-8")
    (source / "pair_validation.json").write_text(
        json.dumps({"status": "shared_simulation_clock_paired"}), encoding="utf-8"
    )
    header = ("time_s,cg_x_sar_m,cg_y_sar_m,cg_z_sar_m,cg_vx_sar_mps,cg_vy_sar_mps,"
              "cg_vz_sar_mps,qw_nb,qx_nb,qy_nb,qz_nb")
    rows = np.array([[0, 0, -0.1, 300.2, 10, 0, 0, 1, 0, 0, 0],
                     [0.01, 0.1, -0.1, 300.2, 10, 0, 0, 1, 0, 0, 0]])
    for name in ("truth_1000hz.csv", "navigation_100hz.csv"):
        np.savetxt(source / name, rows, delimiter=",", header=header, comments="")
    output = tmp_path / "apc"
    manifest = export_apc_from_aligned(source, output, (0.0, 0.1, 0.2))
    assert manifest["status"] == "simulation_geometry_applied_not_physical_calibration"
    truth = np.genfromtxt(output / "apc_truth.csv", delimiter=",", names=True)
    assert_allclose(truth["cg_z_sar_m"], [300.2, 300.2])
    assert_allclose(truth["apc_z_sar_m"], [300.0, 300.0])
    assert_allclose(truth["apc_y_sar_m"], [0.0, 0.0], atol=1e-14)
    with pytest.raises(FileExistsError):
        export_apc_from_aligned(source, output, (0.0, 0.1, 0.2))
