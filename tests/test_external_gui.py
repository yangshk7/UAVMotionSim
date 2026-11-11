"""External GUI contracts: route mapping, native clocks and navigation separation."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "main" / "internal"))
from external_job import convert_pass, interpolate_pose, route_spec, wsl_path

from uavsim.config.validator import Config
from uavsim.dynamics.simulator import simulate_pass
from uavsim.motion.export import load_motion_segment


def test_external_route_maps_baseline_and_lever():
    cfg = Config(nominal_baseline_sar_m=(2, 3, 4))
    spec = route_spec(cfg, 1)
    lever = cfg.antenna.phase_center_frd_m
    assert spec["duration_s"] == cfg.simulation.acquisition_duration_s
    assert spec["altitude_m"] == 304+lever[2]
    assert spec["center_y_m"] == 3-lever[1]
    assert spec["start_x_m"] == -102.4+2-lever[0]
    assert spec["reposition_x_m"] < spec["start_x_m"]
    assert wsl_path(r"C:\Some Folder\中文\route.json") == "/mnt/c/Some Folder/中文/route.json"


def test_external_unsupported_route_is_rejected():
    cfg = Config()
    cfg.trajectory.yaw_deg = 20
    with pytest.raises(ValueError, match="偏航"):
        route_spec(cfg, 0)
    cfg.trajectory.yaw_deg = 0
    cfg.simulation.navigation_rate_hz = 50
    with pytest.raises(ValueError, match="100 Hz"):
        route_spec(cfg, 0)


@pytest.mark.parametrize("backend,rate", [("sitl", 300), ("px4", 1000)])
@pytest.mark.parametrize("source", ["shared_model", "flight_controller"])
def test_external_exports_keep_truth_and_navigation_sources_separate(tmp_path, backend, rate, source):
    cfg = Config()
    cfg.runtime.backend = backend
    cfg.runtime.external_navigation_source = source
    cfg.simulation.acquisition_duration_s = 2
    for key in ("navigation_position_error", "navigation_velocity_error", "navigation_attitude_error", "imu_gnss"):
        getattr(cfg.disturbances, key).enabled = False
    native = tmp_path / "native"
    native.mkdir()
    output = tmp_path / "output"
    output.mkdir()
    truth_file = "truth_native.csv" if backend == "sitl" else "truth_1000hz.csv"
    header = "time_s,cg_x_sar_m,cg_y_sar_m,cg_z_sar_m,cg_vx_sar_mps,cg_vy_sar_mps,cg_vz_sar_mps,qw_nb,qx_nb,qy_nb,qz_nb"
    for file, frequency, offset in ((truth_file, rate, 0), ("navigation_100hz.csv", 100, 1)):
        t = np.arange(2*frequency+1)/frequency
        rows = np.column_stack((t, 10*t+offset, np.zeros_like(t), np.full_like(t, 300),
                                np.full_like(t, 10), np.zeros((len(t), 2)),
                                np.ones_like(t), np.zeros((len(t), 3))))
        np.savetxt(native/file, rows, delimiter=",", header=header, comments="")
    original = (native/truth_file).read_bytes()
    (native/"alignment.json").write_text(json.dumps({"files": {"truth": truth_file}}))
    (native/"pair_validation.json").write_text(json.dumps({"status": "shared_simulation_clock_paired"}))
    result = convert_pass(cfg, native, output, "pass1", 0)
    assert (native/truth_file).read_bytes() == original
    expected = 0 if source == "shared_model" else 1
    np.testing.assert_allclose(result["navigation"]["position_error_sar_m"][:, 0], expected, atol=1e-10)
    np.testing.assert_allclose(result["flight_controller_navigation"]["position_error_sar_m"][:, 0], 1, atol=1e-10)
    segment = load_motion_segment(output, "pass1")
    assert segment.truth_rate_hz == rate
    assert len(segment.truth.time_s) == 2*rate+1
    assert len(segment.navigation.time_s) == 201
    np.testing.assert_allclose(segment.truth.position_sar_m[:, 0], 10*segment.truth.time_s, atol=1e-10)
    manifest = json.loads((output/"motion_segment_pass1.json").read_text(encoding="utf-8"))
    assert "interpolated" in manifest["provenance"]["truth_export_sampling"]
    assert (output/"flight_controller_navigation_pass1.csv").exists()
    assert (output/"shared_navigation_pass1.csv").exists()


def test_external_adapter_never_extrapolates():
    t = np.array([0., 1.])
    packet = t, np.zeros((2, 3)), np.zeros((2, 3)), np.array([[1., 0, 0, 0]]*2)
    with pytest.raises(ValueError, match="outside"):
        interpolate_pose(packet, np.array([0., 2.]))


@pytest.mark.parametrize("backend", ["sitl", "px4"])
def test_external_backend_cannot_silently_use_python_physics(backend):
    cfg = Config()
    cfg.runtime.backend = backend
    with pytest.raises(ValueError, match="上位机"):
        simulate_pass(cfg, 123)
