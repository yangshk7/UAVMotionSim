"""Matched physical-core comparison against the sibling MATLAB UAV project."""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
from scipy.io import loadmat, savemat
from scipy.spatial.transform import Rotation

from uavsim.config.loader import load_config, save_config
from uavsim.disturbances.injection_manager import DisturbanceInputs
from uavsim.dynamics.simulator import simulate_pass

PROJECT = Path(__file__).resolve().parents[2]


def run_matlab_motion_comparison(root: Path, matlab_exe=None):
    folder = root / "matlab_motion_comparison"
    folder.mkdir()
    record = {"status": "failed", "cases": []}
    try:
        executable = matlab_exe or shutil.which("matlab")
        legacy = PROJECT.parent / "无人机航迹仿真"
        if not executable or not legacy.is_dir():
            record.update(status="skipped", detail="MATLAB or the legacy UAV project is unavailable")
            return record
        config = load_config(PROJECT / "main/config.yaml")
        config.disturbances.all_enabled = False
        for group in (config.disturbances.translation, config.disturbances.attitude):
            for factor in group.values():
                factor.enabled = False
        config.disturbances.imu_gnss.enabled = False
        config.disturbances.navigation_position_error.enabled = False
        config.disturbances.navigation_velocity_error.enabled = False
        config.disturbances.navigation_attitude_error.enabled = False
        s, v, c = config.simulation, config.vehicle, config.controller
        parameters = {
            "speed": config.trajectory.speed_mps, "height": config.trajectory.apc_height_m,
            "duration": s.acquisition_duration_s, "pre": s.pre_stabilization_s,
            "rate": s.dynamics_rate_hz, "sarRate": s.sar_rate_hz,
            "mass": v.mass_kg, "gravity": v.gravity_mps2, "inertia": np.array(v.inertia_kg_m2),
            "arm": v.arm_length_m, "kf": v.kf, "km": v.km,
            "motorTau": v.motor_time_constant_s, "maximum": v.max_rotor_thrust_N,
            "lever": config.antenna.phase_center_frd_m,
            "controlRates": [c.position_rate_hz, c.velocity_rate_hz, c.attitude_rate_hz, c.angular_rate_hz],
            "positionKp": c.kp_position_per_s, "velocityKp": c.kp_velocity_per_s,
            "velocityKi": c.ki_velocity_per_s2, "velocityIntegral": c.velocity_integral_limit_m,
            "attitudeKp": c.kp_attitude_per_s, "rateKp": c.kp_rate_per_s,
            "rateKi": c.ki_rate_per_s2, "rateIntegral": c.rate_integral_limit_rad,
            "weights": c.allocation_weights, "speedLimit": c.max_speed_mps,
            "horizontalAcceleration": c.max_horizontal_acceleration_mps2,
            "verticalAcceleration": c.max_vertical_acceleration_mps2,
            "rateLimit": c.max_body_rate_radps, "tilt": c.max_tilt_deg,
        }
        parameters = {key: np.asarray(value, dtype=np.float64) for key, value in parameters.items()}
        results = {}
        for name in ("baseline", "force_y"):
            if name == "force_y":
                factor = config.disturbances.translation["y"]
                factor.enabled = True
                factor.injection_mode = "dynamics"
                factor.unit = "N"
                factor.amplitude = 2.0
                factor.profile = "constant"
                factor.start_time_s = 5.0
                factor.end_time_s = 7.0
                factor.ramp_s = 0.5
            save_config(config, folder / f"python_{name}.yaml")
            print(f"[MATLAB UAV 对照] RotorPy {name}", flush=True)
            data = simulate_pass(config, config.pass_seeds[0])
            truth = data["truth_full"]
            results[name] = truth
            savemat(folder / f"python_{name}.mat", truth)
        time = results["force_y"]["time_s"]
        inputs = DisturbanceInputs(config, config.pass_seeds[0])
        force, _ = inputs.dynamics_arrays(time)
        savemat(folder / "shared_inputs.mat",
                {"parameters": parameters, "time_s": time[:, None], "force_y": force[:, 1, None]})
        def literal(path):
            return "'" + path.resolve().as_posix().replace("'", "''") + "'"
        command = (f"addpath({literal(PROJECT / 'matlab')}); "
                   f"compare_legacy_motion({literal(folder)}, {literal(legacy)});")
        print("[MATLAB UAV 对照] 运行 MATLAB 原生动力学与控制器", flush=True)
        with (folder / "matlab.log").open("w", encoding="utf-8") as log:
            completed = subprocess.run([str(executable), "-batch", command], cwd=PROJECT,
                                       stdout=log, stderr=subprocess.STDOUT, timeout=900)
        if completed.returncode:
            raise RuntimeError("MATLAB UAV comparison failed; see matlab_motion_comparison/matlab.log")
        for name, truth in results.items():
            data = loadmat(folder / f"matlab_{name}.mat", simplify_cells=True)
            np.testing.assert_allclose(data["time_s"], truth["time_s"], rtol=0, atol=1e-9)
            state = data["states"]
            if not (np.isfinite(state).all() and np.isfinite(truth["position_N_m"]).all()
                    and np.isfinite(truth["velocity_N_mps"]).all()):
                raise ValueError(f"Nonfinite comparison state in {name}")
            expected_force = np.zeros_like(data["external"])
            if name == "force_y":
                expected_force[:, 1] = force[:, 1]
            np.testing.assert_allclose(data["external"], expected_force, rtol=0, atol=1e-12)
            mask = time >= 0
            delta_p = state[mask, :3] - truth["position_N_m"][mask]
            delta_v = state[mask, 3:6] - truth["velocity_N_mps"][mask]
            angles = (Rotation.from_quat(truth["q_NB_wxyz"][mask], scalar_first=True).inv()
                      * Rotation.from_quat(state[mask, 6:10], scalar_first=True)).magnitude()
            def rms(values):
                return np.sqrt(np.mean(values**2, axis=0))
            record["cases"].append({
                "case": name, "samples": int(mask.sum()),
                "position_xyz_rms_m": rms(delta_p).tolist(),
                "velocity_xyz_rms_mps": rms(delta_v).tolist(),
                "attitude_angle_rms_deg": float(np.rad2deg(rms(angles))),
                "maximum_position_difference_m": float(np.linalg.norm(delta_p, axis=1).max()),
            })
        record.update(status="passed", detail="Two matched physical-core experiments executed; differences reported, no equivalence threshold set")
    except (OSError, ValueError, RuntimeError, AssertionError, subprocess.TimeoutExpired) as error:
        record["detail"] = str(error)
    finally:
        (folder / "result.json").write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
    return record
