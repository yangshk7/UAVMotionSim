from time import perf_counter

import numpy as np
from threadpoolctl import threadpool_limits

from uavsim.controllers.cascade import CascadeController
from uavsim.disturbances.injection_manager import DisturbanceInputs
from uavsim.experiments.geometry_decoupling import decouple_geometry
from uavsim.frames.coordinate import polar_to_sar, rotation_to_sar
from uavsim.frames.quaternion import from_matrix, to_matrix
from uavsim.frames.rotation import euler_to_matrix, matrix_to_euler
from uavsim.navigation.position_error import navigation_estimate
from uavsim.navigation.resampling import resample_navigation
from uavsim.navigation.state_error import navigation_pose
from uavsim.runtime.memory import peak_working_set_bytes
from uavsim.runtime.progress import report_progress
from uavsim.sar_adapter.antenna_geometry import antenna_geometry
from uavsim.sar_adapter.pulse_sync import sample_indices
from uavsim.trajectory.reference_manager import build_reference
from uavsim.vehicle.hex_x12 import HexX12

from .equations import RigidBodyDynamics
from .integrator import rk4_step
from .state import initial_state


def _simulate(config, pass_seed: int, pass_index: int) -> dict:
    start = perf_counter()
    s = config.simulation
    rate = s.dynamics_rate_hz
    count = round((s.pre_stabilization_s + s.acquisition_duration_s) * rate) + 1
    pre = round(s.pre_stabilization_s * rate)
    time = (np.arange(count) - pre) / rate
    baseline = pass_index * np.array(config.nominal_baseline_sar_m)
    reference = build_reference(config, time, baseline)
    inputs = DisturbanceInputs(config, pass_seed)
    command_offsets = inputs.command_arrays(time)
    # 预计算起点/半步/终点外力，实现不同积分频率共享同一连续扰动。
    half_time = (np.arange(2 * count - 1) / 2 - pre) / rate
    force, torque = inputs.dynamics_arrays(half_time)
    controller = CascadeController(config, HexX12(config.vehicle))
    vehicle = controller.vehicle
    if config.runtime.backend == "rotorpy":
        from uavsim.dynamics.rotorpy_hex import RotorPyHexDynamics

        dynamics = RotorPyHexDynamics(vehicle)
    else:
        dynamics = RigidBodyDynamics(vehicle)
    state = initial_state(
        reference["position_N_m"][0] + polar_to_sar(s.initial_position_error_sar_m),
        reference["velocity_N_mps"][0] + polar_to_sar(s.initial_velocity_error_sar_mps),
        from_matrix(
            reference["rotation_NB"][0] @ euler_to_matrix(np.deg2rad(s.initial_attitude_error_rpy_deg))
        ),
        vehicle.hover_speed_radps,
    )
    state[10:13] = s.initial_body_rate_radps
    states = np.empty((count, 19))
    accelerations = np.empty((count, 3))
    angular_accelerations = np.empty((count, 3))
    motor_commands = np.empty((count, 6))
    allocation_residuals = np.empty((count, 4))
    fields = ["position_N_m", "velocity_N_mps", "acceleration_N_mps2", "yaw_rad", "yaw_rate_radps"]
    progress_stride = max(1, (count - 1) // 100)
    for index in range(count):
        if index % progress_stride == 0:
            report_progress(pass_index=pass_index, fraction=index / count)
        current = {key: reference[key][index] for key in fields}
        for key, offset in [
            ("position_N_m", "position_offset_N_m"),
            ("velocity_N_mps", "velocity_offset_N_mps"),
            ("acceleration_N_mps2", "acceleration_offset_N_mps2"),
        ]:
            current[key] = current[key] + command_offsets[offset][index]
        motor_command = controller.update(
            index,
            state,
            current,
            command_offsets["attitude_offset_rad"][index],
            command_offsets["rate_offset_radps"][index],
        )
        states[index] = state
        rhs = dynamics.derivative(state, motor_command, force[2 * index], torque[2 * index])
        accelerations[index] = rhs[3:6]
        angular_accelerations[index] = rhs[10:13]
        motor_commands[index] = motor_command
        allocation_residuals[index] = controller.residual
        if index < count - 1:
            state = rk4_step(
                dynamics,
                state,
                motor_command,
                1 / rate,
                (force[2 * index], force[2 * index + 1], force[2 * index + 2]),
                (torque[2 * index], torque[2 * index + 1], torque[2 * index + 2]),
            )
    report_progress(pass_index=pass_index, fraction=1.0)
    return assemble_pass(
        config, time, states, accelerations, angular_accelerations, motor_commands,
        allocation_residuals, reference, inputs, baseline, pass_seed, pass_index,
        start, controller.update_counts,
    )


def assemble_pass(
    config, time, states, accelerations, angular_accelerations, motor_commands,
    allocation_residuals, reference, inputs, baseline, pass_seed, pass_index,
    start, update_counts, backend_metadata=None,
):
    """Build the common truth, navigation, antenna and SAR contract for all backends."""
    s = config.simulation
    truth_full = {
        "time_s": time,
        "position_N_m": states[:, :3],
        "velocity_N_mps": states[:, 3:6],
        "q_NB_wxyz": states[:, 6:10],
        "omega_B_radps": states[:, 10:13],
        "rotor_speed_radps": states[:, 13:],
        "acceleration_N_mps2": accelerations,
        "angular_acceleration_B_radps2": angular_accelerations,
        "motor_command_radps": motor_commands,
        "allocation_residual": allocation_residuals,
        "position_sar_m": polar_to_sar(states[:, :3]),
        "frame": "NED_FRD_truth",
    }
    indices = sample_indices(config)
    ids = indices["sar"]
    truth = {key: value[ids] if isinstance(value, np.ndarray) else value for key, value in truth_full.items()}
    truth["time_s"] = np.arange(len(ids)) / s.sar_rate_hz
    truth["dynamics_sample_index"] = ids
    truth["pulse_index"] = np.arange(len(ids), dtype=np.int64)
    rotation = to_matrix(truth["q_NB_wxyz"])
    truth["rotation_NB"] = rotation
    truth["euler_NB_rpy_rad"] = matrix_to_euler(rotation)
    truth["euler_sar_xyz_rad"] = matrix_to_euler(rotation_to_sar(rotation))
    for name in ["velocity_N_mps", "acceleration_N_mps2"]:
        truth[name.replace("_N_", "_sar_")] = polar_to_sar(truth[name])
    truth.update(
        antenna_geometry(
            truth["position_N_m"],
            truth["velocity_N_mps"],
            rotation,
            truth["omega_B_radps"],
            np.array(config.antenna.phase_center_frd_m),
            config.antenna.look_angle_deg,
            truth["acceleration_N_mps2"],
            truth["angular_acceleration_B_radps2"],
            config.antenna.squint_angle_deg,
        )
    )
    for key in ["apc_position_N_m", "apc_velocity_N_mps", "apc_acceleration_N_mps2", "beam_N_unit"]:
        truth[key.replace("_N_", "_sar_")] = polar_to_sar(truth[key])
    nav_ids = indices["navigation"]
    nav_time = np.arange(len(nav_ids)) / s.navigation_rate_hz
    navigation = navigation_estimate(config, nav_time, truth_full["position_sar_m"][nav_ids], pass_seed)
    sampled_truth = {
        key: value[nav_ids] for key, value in truth_full.items() if isinstance(value, np.ndarray)
    }
    navigation, navigation_truth = navigation_pose(config, navigation, sampled_truth, pass_seed)
    nav_sar = resample_navigation(navigation, truth["time_s"])
    nav_sar["source_rate_hz"] = s.navigation_rate_hz
    nav_sar["apc_position_sar_m"] = nav_sar["position_sar_m"] + polar_to_sar(
        np.einsum("nij,j->ni", nav_sar["rotation_NB"], config.antenna.phase_center_frd_m)
    )
    sar_reference = {key: value[ids] for key, value in reference.items()}
    sar_reference["time_s"] = truth["time_s"]
    geometry = decouple_geometry(config, truth, sar_reference)
    return {
        "truth_full": truth_full,
        "truth_sar": truth,
        "navigation": navigation,
        "navigation_truth": navigation_truth,
        "navigation_sar": nav_sar,
        "sar_geometry": geometry,
        "reference": sar_reference,
        "disturbance_inputs": inputs.export(truth["time_s"]),
        "metadata": {
            "physical_backend": config.runtime.backend,
            **(backend_metadata or {}),
            "pass_seed": pass_seed,
            "pass_index": pass_index + 1,
            "elapsed_s": perf_counter() - start,
            "peak_working_set_bytes": peak_working_set_bytes(),
            "nominal_baseline_sar_m": baseline,
            "controller_update_counts": update_counts,
        },
    }


def simulate_pass(config, pass_seed: int, pass_index: int = 0) -> dict:
    """单航次入口；float64，预分配，限制 BLAS 线程以避免多进程过量争用。"""
    if config.runtime.backend in ("sitl", "px4"):
        raise ValueError("SITL/PX4 请从上位机运行；外部链路具有独立原生时钟与数据适配流程，不能使用内置积分器。")
    with threadpool_limits(limits=config.runtime.blas_threads_per_worker):
        if config.runtime.backend == "matlab":
            from .matlab_backend import simulate_matlab_pass

            return simulate_matlab_pass(config, pass_seed, pass_index)
        return _simulate(config, pass_seed, pass_index)
