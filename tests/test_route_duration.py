"""按长度自动规划时间，并验证可裁剪的连续匀速航段。"""

import numpy as np
import pytest

from uavsim.analysis.route_quality import evaluate_route_quality
from uavsim.config.loader import load_config
from uavsim.config.validator import Config
from uavsim.trajectory.reference_manager import build_reference


def test_1000m_route_time_and_central_window():
    config = load_config("configs/route_1000m.yaml")
    assert config.simulation.acquisition_duration_s == 110
    reference = build_reference(config, np.array([0.0, 5.0, 105.0, 110.0]))
    np.testing.assert_allclose(reference["apc_position_sar_m"][:, 0], [-550, -500, 500, 550])
    np.testing.assert_allclose(reference["apc_velocity_sar_mps"][:, 0], 10)


def test_duration_rounds_up_without_shortening_margin():
    config = Config.model_validate({
        "simulation": {"duration_mode": "route_length", "duration_margin_s": 1.23},
        "trajectory": {"route_length_m": 23.0, "speed_mps": 7.0},
    })
    assert config.simulation.acquisition_duration_s >= 23 / 7 + 1.23
    assert abs(config.simulation.acquisition_duration_s * 100 - round(config.simulation.acquisition_duration_s * 100)) < 1e-8


def test_route_quality_does_not_hide_bad_speed_samples():
    config = Config.model_validate({
        "simulation": {"duration_mode": "route_length", "duration_margin_s": 1.0,
                       "pre_stabilization_s": 0.0, "dynamics_rate_hz": 1600},
        "trajectory": {"route_length_m": 20.0, "speed_mps": 10.0},
    })
    count = round(config.simulation.acquisition_duration_s * 1600) + 1
    time = np.arange(count) / 1600
    position = np.zeros((count, 3))
    position[:, 0] = 10 * time
    velocity = np.zeros_like(position)
    velocity[:, 0] = 10
    data = {"truth_full": {"position_sar_m": position, "velocity_N_mps": velocity}}
    good = evaluate_route_quality(data, config)
    assert good["passed"]
    assert good["crop_window"]["forward_distance_m"] >= 20
    assert good["crop_window"]["start_time_s"] == pytest.approx(0.5)
    assert good["crop_window"]["end_time_s"] == pytest.approx(2.5)
    velocity[count // 2, 0] = 8.9
    bad = evaluate_route_quality(data, config)
    assert not bad["passed"]
    assert bad["crop_window"] is None
    assert bad["longest_contiguous_valid_distance_m"] < 20

    velocity[:, 0] = 10
    velocity[(time < 0.501) | (time > 2.509), 0] = 8.9
    unaligned = evaluate_route_quality(data, config)
    assert not unaligned["passed"]
    assert unaligned["longest_contiguous_valid_distance_m"] >= 20


def test_disturbed_route_keeps_speed_fluctuation_as_observation():
    values = Config().model_dump()
    values["simulation"].update(duration_mode="route_length", duration_margin_s=1.0,
                                pre_stabilization_s=0.0, dynamics_rate_hz=1600)
    values["trajectory"].update(route_length_m=20.0, speed_mps=10.0)
    values["disturbances"]["translation"]["x"].update(enabled=True, amplitude=1.0)
    config = Config.model_validate(values)
    count = round(config.simulation.acquisition_duration_s * 1600) + 1
    time = np.arange(count) / 1600
    position = np.zeros((count, 3))
    position[:, 0] = 10 * time
    velocity = np.zeros_like(position)
    velocity[:, 0] = 10
    velocity[count // 2, 0] = 8.9
    data = {"truth_full": {"position_sar_m": position, "velocity_N_mps": velocity}}
    quality = evaluate_route_quality(data, config)
    assert quality["passed"]
    assert quality["assessment_mode"] == "disturbed_physical_coverage"
    assert not quality["speed_band_enforced"]
    assert quality["fraction_within_speed_band"] < 1
    assert quality["crop_window"]["forward_distance_m"] >= 20
    assert quality["crop_window"]["speed_min_mps"] == pytest.approx(8.9)

    position[:, 0] = 5 * time
    too_short = evaluate_route_quality(data, config)
    assert not too_short["passed"]
    assert too_short["crop_window"] is None


def test_route_length_mode_requires_straight_reference():
    with pytest.raises(ValueError, match="straight"):
        Config.model_validate({
            "simulation": {"duration_mode": "route_length"},
            "trajectory": {"type": "waypoint", "times_s": [0, 1],
                           "positions_sar_m": [[0, 0, 300], [10, 0, 300]]},
        })
