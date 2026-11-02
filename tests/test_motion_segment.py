"""统一采集段接口的时间、坐标、文件与误差隔离。"""

import json

import numpy as np
import pytest
from numpy.testing import assert_allclose, assert_array_equal
from test_navigation import short_config

from uavsim.config.validator import Config
from uavsim.dynamics.simulator import simulate_pass
from uavsim.motion.export import export_motion_segment, load_motion_segment
from uavsim.motion.segment import MotionSegment, MotionSeries, crop_motion_segment, from_simulated_pass


def test_segment_preserves_native_truth_and_navigation(tmp_path):
    config = short_config()
    run = simulate_pass(config, 123)
    segment = from_simulated_pass(run, config)
    assert segment.truth_rate_hz == 3200
    assert segment.navigation_rate_hz == 100
    assert len(segment.truth.time_s) == 641
    assert len(segment.navigation.time_s) == 21
    assert_allclose(segment.truth.position_sar_m[::4], run["truth_sar"]["position_sar_m"])
    assert_allclose(segment.truth.apc_position_sar_m[::4], run["truth_sar"]["apc_position_sar_m"])
    assert_array_equal(segment.navigation.position_sar_m, run["navigation"]["position_sar_m"])

    export_motion_segment(segment, tmp_path, "pass1")
    truth = np.genfromtxt(tmp_path / "motion_truth_pass1.csv", delimiter=",", names=True)
    nav = np.genfromtxt(tmp_path / "motion_navigation_pass1.csv", delimiter=",", names=True)
    assert_allclose(truth["cg_x_sar_m"], segment.truth.position_sar_m[:, 0])
    assert_allclose(nav["apc_z_sar_m"], segment.navigation.apc_position_sar_m[:, 2])
    manifest = json.loads((tmp_path / "motion_segment_pass1.json").read_text(encoding="utf-8"))
    assert manifest["truth_samples"] == 641
    assert manifest["navigation_samples"] == 21
    loaded = load_motion_segment(tmp_path, "pass1")
    assert_allclose(loaded.truth.apc_position_sar_m, segment.truth.apc_position_sar_m)
    assert_allclose(loaded.navigation.q_NB_wxyz, segment.navigation.q_NB_wxyz)
    with pytest.raises(FileExistsError):
        export_motion_segment(segment, tmp_path, "pass1")

    cropped = crop_motion_segment(segment, 0.05, 0.15)
    assert len(cropped.truth.time_s) == 321
    assert len(cropped.navigation.time_s) == 11
    assert_allclose(cropped.truth.position_sar_m[0], segment.truth.position_sar_m[160])
    assert_allclose(cropped.navigation.position_sar_m[-1], segment.navigation.position_sar_m[15])
    with pytest.raises(ValueError, match="both truth and navigation"):
        crop_motion_segment(segment, 0.0503125, 0.15)


def test_navigation_error_changes_estimate_only():
    config = short_config()
    clean = from_simulated_pass(simulate_pass(config, 123), config)
    data = config.model_dump()
    data["disturbances"]["navigation_position_error"]["enabled"] = True
    noisy_config = Config.model_validate(data)
    noisy = from_simulated_pass(simulate_pass(noisy_config, 123), noisy_config)
    assert_array_equal(clean.truth.position_sar_m, noisy.truth.position_sar_m)
    assert not np.array_equal(clean.navigation.position_sar_m, noisy.navigation.position_sar_m)


def test_contract_rejects_unaligned_time_and_nonunit_attitude():
    time = np.array([0.0, 0.01, 0.02])
    position = np.zeros((3, 3))
    q = np.tile([1.0, 0.0, 0.0, 0.0], (3, 1))
    series = MotionSeries(time, position, position, q, position)
    with pytest.raises(ValueError, match="common time grid"):
        MotionSegment(series, series, "external", 100, 100, "receipt_nearest", "flight_controller")
    with pytest.raises(ValueError, match="unit norm"):
        MotionSeries(time, position, position, q * 2, position)
