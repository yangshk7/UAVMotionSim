import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, RotationSpline

from uavsim.frames.coordinate import polar_to_sar
from uavsim.frames.quaternion import from_matrix
from uavsim.sar_adapter.antenna_geometry import antenna_geometry


def decouple_geometry(config, truth: dict, reference: dict) -> dict:
    """独立实验状态；连续位置/姿态样条的导数保证所选运动学一致，不写回真值。"""
    t = truth["time_s"]
    p = truth["position_N_m"] if config.geometry.translation == "actual" else reference["position_N_m"]
    rotation = truth["rotation_NB"] if config.geometry.attitude == "actual" else reference["rotation_NB"]
    ps = CubicSpline(t, p)
    rs = RotationSpline(t, Rotation.from_matrix(rotation))
    v, a = ps(t, 1), ps(t, 2)
    omega, alpha = rs(t, 1), rs(t, 2)
    antenna = antenna_geometry(
        p,
        v,
        rotation,
        omega,
        np.array(config.antenna.phase_center_frd_m),
        config.antenna.look_angle_deg,
        a,
        alpha,
        config.antenna.squint_angle_deg,
    )
    result = {
        "label": "geometry_decoupled",
        "physical_truth": False,
        "time_s": t,
        "translation_source": config.geometry.translation,
        "attitude_source": config.geometry.attitude,
        "position_N_m": p.copy(),
        "velocity_N_mps": v,
        "acceleration_N_mps2": a,
        "q_NB_wxyz": from_matrix(rotation),
        "rotation_NB": rotation,
        "omega_B_radps": omega,
        "angular_acceleration_B_radps2": alpha,
    }
    result.update(antenna)
    for key in [
        "position_N_m",
        "velocity_N_mps",
        "acceleration_N_mps2",
        "apc_position_N_m",
        "apc_velocity_N_mps",
        "apc_acceleration_N_mps2",
        "beam_N_unit",
    ]:
        result[key.replace("_N_", "_sar_")] = polar_to_sar(result[key])
    return result
