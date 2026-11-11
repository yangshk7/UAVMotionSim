"""MATLAB legacy physical core behind the common simulation contract."""

import hashlib
import os
import queue
import shutil
import subprocess
import tempfile
import threading
from collections import deque
from pathlib import Path
from time import monotonic, perf_counter

import numpy as np
from scipy.io import loadmat, savemat

from uavsim.disturbances.injection_manager import DisturbanceInputs
from uavsim.frames.coordinate import polar_to_sar
from uavsim.frames.quaternion import from_matrix
from uavsim.frames.rotation import euler_to_matrix
from uavsim.runtime.progress import report_progress
from uavsim.trajectory.reference_manager import build_reference
from uavsim.vehicle.hex_x12 import HexX12

from .state import initial_state

PROJECT = Path(__file__).resolve().parents[3]
PROGRESS_PREFIX = "UAVSIM_MATLAB_PROGRESS "


def resolve_matlab(config):
    executable = config.runtime.matlab_executable
    resolved = shutil.which(executable or "matlab")
    if resolved is None:
        raise RuntimeError("MATLAB仿真链路需要已安装的MATLAB，请填写有效的MATLAB程序路径。")
    return resolved


def run_matlab(executable, input_file, output_file, timeout_s, pass_index):
    """Stream native solver progress, enforce timeout, retain diagnostics in the task log."""
    def literal(path):
        return "'" + path.resolve().as_posix().replace("'", "''") + "'"

    expression = (f"addpath({literal(PROJECT / 'matlab')}); "
                  f"uav_matlab_backend({literal(input_file)}, {literal(output_file)});")
    process = subprocess.Popen(
        [executable, "-batch", expression], cwd=PROJECT,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        encoding="utf-8", errors="replace",
        creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
    )
    lines = queue.Queue()

    def read_stdout():
        try:
            for line in process.stdout:
                lines.put(line)
        finally:
            lines.put(None)

    reader = threading.Thread(target=read_stdout, daemon=True)
    reader.start()
    tail = deque(maxlen=100)
    deadline = monotonic() + timeout_s
    try:
        while True:
            if monotonic() >= deadline:
                raise TimeoutError(f"MATLAB simulation exceeded {timeout_s} s")
            try:
                line = lines.get(timeout=0.2)
            except queue.Empty:
                continue
            if line is None:
                break
            if line.startswith(PROGRESS_PREFIX):
                fraction = float(line.removeprefix(PROGRESS_PREFIX))
                if not np.isfinite(fraction) or not 0 <= fraction <= 1:
                    raise ValueError("Invalid native MATLAB progress")
                report_progress(pass_index=pass_index, fraction=fraction)
            else:
                tail.append(line.rstrip())
                print("[MATLAB] " + line.rstrip(), flush=True)
        returncode = process.wait(timeout=max(1, deadline - monotonic()))
        if returncode or not output_file.is_file():
            raise RuntimeError("MATLAB物理仿真失败：\n" + "\n".join(tail))
    finally:
        if process.poll() is None:
            if os.name == "nt":
                subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                               capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW, check=False)
            else:
                process.kill()
            process.wait(timeout=15)
        process.stdout.close()
        reader.join(timeout=2)


def _numeric_fields(model):
    return {name: np.asarray(value, dtype=np.float64) for name, value in model.model_dump().items()
            if not isinstance(value, str)}


def simulate_matlab_pass(config, pass_seed, pass_index=0):
    """Shared references and disturbances → MATLAB physics → shared GNSS/INS and SAR output.

    Units are SI, frames NED/FRD, state layout matches the 19-state Python model.
    Native source is vendored under matlab/legacy_uav; no sibling project is required.
    """
    from .simulator import assemble_pass

    executable = resolve_matlab(config)
    start = perf_counter()
    s = config.simulation
    count = round((s.pre_stabilization_s + s.acquisition_duration_s) * s.dynamics_rate_hz) + 1
    pre = round(s.pre_stabilization_s * s.dynamics_rate_hz)
    time = (np.arange(count) - pre) / s.dynamics_rate_hz
    baseline = pass_index * np.asarray(config.nominal_baseline_sar_m)
    reference = build_reference(config, time, baseline)
    inputs = DisturbanceInputs(config, pass_seed)
    offsets = inputs.command_arrays(time)
    native_reference = {key: value.copy() for key, value in reference.items()}
    for field, offset in (("position_N_m", "position_offset_N_m"),
                          ("velocity_N_mps", "velocity_offset_N_mps"),
                          ("acceleration_N_mps2", "acceleration_offset_N_mps2")):
        native_reference[field] += offsets[offset]
    native_reference["attitudeBias_rad"] = offsets["attitude_offset_rad"]
    half_time = (np.arange(2 * count - 1) / 2 - pre) / s.dynamics_rate_hz
    force, torque = inputs.dynamics_arrays(half_time)
    external = np.column_stack((force[::2], torque[::2]))
    external_half = np.column_stack((force[1::2], torque[1::2]))
    vehicle = HexX12(config.vehicle)
    state = initial_state(
        reference["position_N_m"][0] + polar_to_sar(s.initial_position_error_sar_m),
        reference["velocity_N_mps"][0] + polar_to_sar(s.initial_velocity_error_sar_mps),
        from_matrix(reference["rotation_NB"][0] @ euler_to_matrix(np.deg2rad(s.initial_attitude_error_rpy_deg))),
        vehicle.hover_speed_radps,
    )
    state[10:13] = s.initial_body_rate_radps
    rates = [s.dynamics_rate_hz] + [getattr(config.controller, key + "_rate_hz")
                                 for key in ("position", "velocity", "attitude", "angular")]
    with tempfile.TemporaryDirectory(prefix="uavsim_matlab_") as temporary:
        folder = Path(temporary)
        input_file, output_file = folder / "input.mat", folder / "output.mat"
        savemat(input_file, {
            "reference": native_reference, "external": external, "external_half": external_half,
            "initial_state": state, "vehicle": _numeric_fields(config.vehicle),
            "controller": _numeric_fields(config.controller), "rates": np.asarray(rates, dtype=float),
            "pre": float(s.pre_stabilization_s), "allocation_matrix": vehicle.allocation_matrix,
        }, oned_as="column", long_field_names=True)
        report_progress(pass_index=pass_index, fraction=0)
        run_matlab(executable, input_file, output_file, config.runtime.matlab_timeout_s, pass_index)
        output = loadmat(output_file, simplify_cells=True)
    np.testing.assert_allclose(output["time_s"], time, atol=1e-10, rtol=0)
    for field, width in (("states", 19), ("acceleration", 3), ("angular_acceleration", 3),
                         ("motor_commands", 6), ("allocation_residuals", 4)):
        array = np.asarray(output[field])
        if array.shape != (count, width) or not np.isfinite(array).all():
            raise ValueError(f"Invalid MATLAB output {field}: expected {(count, width)}")
    updates = np.array([(count - 1) // (s.dynamics_rate_hz // rate) + 1 for rate in rates[1:]])
    digest = hashlib.sha256()
    for path in sorted((PROJECT / "matlab" / "legacy_uav").rglob("*.m")):
        digest.update(path.relative_to(PROJECT).as_posix().encode())
        digest.update(path.read_bytes())
    return assemble_pass(
        config, time, output["states"], output["acceleration"], output["angular_acceleration"],
        output["motor_commands"], output["allocation_residuals"], reference, inputs, baseline,
        pass_seed, pass_index, start, updates,
        {"matlab_version": str(output["matlab_version"]), "matlab_physics_sha256": digest.hexdigest()},
    )
