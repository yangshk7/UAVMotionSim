"""以 SITL 共同仿真时钟对齐 DataFlash 真值与 MAVLink 导航。"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from align_px4_sim_clock import _bracket_gaps, _csv
from scipy.spatial.transform import Rotation, Slerp


def align(directory: Path, output: Path) -> dict:
    """真值保留原始 SIM2 时刻，导航在同钟 100 Hz 网格插值；单位 s、m、m/s。"""
    if not output.is_dir():
        raise ValueError("Existing output directory required")
    targets = [output / name for name in ("truth_native.csv", "navigation_100hz.csv", "alignment.json")]
    if any(path.exists() for path in targets):
        raise FileExistsError("Refusing to overwrite an aligned SITL route")
    sim = _csv(output / "dataflash_sim2.csv")
    sim_attitude = _csv(output / "dataflash_sim.csv")
    ekf = _csv(output / "dataflash_xkf1.csv")
    route = _csv(directory / "route_navigation_sar.csv")
    nav = _csv(directory / "navigation_position_sar.csv")
    nav_attitude = _csv(directory / "navigation_attitude_ned.csv")
    sim_time = sim["TimeUS"].astype(np.float64) / 1e6
    sim_att_time = sim_attitude["TimeUS"].astype(np.float64) / 1e6
    ekf_time = ekf["TimeUS"].astype(np.float64) / 1e6
    nav_time_raw = nav["time_boot_s"]
    nav_att_time = nav_attitude["time_boot_s"]
    route_time = route["time_boot_s"]
    if any(np.any(np.diff(values) <= 0) for values in
           (sim_time, sim_att_time, ekf_time, nav_time_raw, nav_att_time, route_time)):
        raise ValueError("SITL source time is not strictly increasing")

    # The EKF position is logged independently at 10 Hz. Its interpolation must
    # reproduce 50 Hz MAVLink route positions on the shared SITL boot clock.
    ekf_position_at_route = np.column_stack((
        np.interp(route_time, ekf_time, ekf["PN"]),
        np.interp(route_time, ekf_time, ekf["PE"]),
        -np.interp(route_time, ekf_time, ekf["PD"]),
    ))
    route_position = np.column_stack((route["x_sar_m"], route["y_sar_m"], route["z_sar_m"]))
    clock_check_difference = ekf_position_at_route - route_position
    clock_check_max = np.max(np.abs(clock_check_difference), axis=0)
    if np.max(clock_check_max) > 0.05:
        raise ValueError("MAVLink boot time and DataFlash TimeUS do not align within 5 cm")
    begin = max(route_time[0], sim_time[0], sim_att_time[0], nav_time_raw[0], nav_att_time[0])
    finish = min(route_time[-1], sim_time[-1], sim_att_time[-1], nav_time_raw[-1], nav_att_time[-1])
    start = math.ceil((begin - 1e-9) * 100) / 100
    end = math.floor((finish + 1e-9) * 100) / 100
    if end - start < 1:
        raise ValueError("Insufficient common SITL route coverage")
    nav_time = start + np.arange(round((end - start) * 100) + 1) / 100
    first_native = max(0, int(np.searchsorted(sim_time, start)) - 1)
    last_native = min(len(sim_time), int(np.searchsorted(sim_time, end, side="right")) + 1)
    native_mask = np.zeros(len(sim_time), dtype=bool)
    native_mask[first_native:last_native] = True
    truth_time = sim_time[native_mask]
    truth_position = np.column_stack((sim["PN"][native_mask], sim["PE"][native_mask],
                                      -sim["PD"][native_mask]))
    truth_velocity = np.column_stack((sim["VN"][native_mask], sim["VE"][native_mask],
                                      -sim["VD"][native_mask]))
    _, truth_attitude_gap = _bracket_gaps(sim_att_time, truth_time)
    _, nav_position_gap = _bracket_gaps(nav_time_raw, nav_time)
    _, nav_attitude_gap = _bracket_gaps(nav_att_time, nav_time)
    if truth_attitude_gap > 0.2 or nav_position_gap > 0.06 or nav_attitude_gap > 0.06:
        raise ValueError("SITL position or attitude stream has excessive gaps")
    truth_att_q = np.column_stack([sim_attitude[key] for key in ("Q2", "Q3", "Q4", "Q1")])
    truth_q = Slerp(sim_att_time, Rotation.from_quat(truth_att_q))(truth_time).as_quat()[:, [3, 0, 1, 2]]
    nav_position = np.column_stack([np.interp(nav_time, nav_time_raw, nav[key]) for key in
                                    ("x_sar_m", "y_sar_m", "z_sar_m")])
    nav_velocity = np.column_stack([np.interp(nav_time, nav_time_raw, nav[key]) for key in
                                    ("vx_sar_mps", "vy_sar_mps", "vz_sar_mps")])
    nav_euler = np.column_stack([nav_attitude[key] for key in ("roll_rad", "pitch_rad", "yaw_rad")])
    nav_rotation = Slerp(nav_att_time, Rotation.from_euler("xyz", nav_euler))(nav_time)
    nav_q = nav_rotation.as_quat()[:, [3, 0, 1, 2]]
    header = "time_s,cg_x_sar_m,cg_y_sar_m,cg_z_sar_m,cg_vx_sar_mps,cg_vy_sar_mps,cg_vz_sar_mps,qw_nb,qx_nb,qy_nb,qz_nb"
    truth_values = np.column_stack((truth_time - start, truth_position, truth_velocity, truth_q))
    nav_values = np.column_stack((nav_time - start, nav_position, nav_velocity, nav_q))
    intervals = np.diff(truth_time)
    report = {
        "clock_alignment": "shared_sitl_boot_simulation_clock_verified_against_independent_ekf_log",
        "truth_source": "ArduPilot DataFlash SIM2 native position and velocity",
        "truth_native_boundary_samples": "one native sample before and after the 100 Hz window for interpolation",
        "truth_attitude_source": "DataFlash SIM quaternion interpolated at SIM2 timestamps",
        "navigation_source": "MAVLink LOCAL_POSITION_NED and ATTITUDE at native about 50 Hz, interpolated to 100 Hz",
        "clock_evidence": "DataFlash TimeUS and MAVLink time_boot_ms agree via independent XKF1 route-position comparison",
        "apc_status": "not applied; CG-to-APC lever arm remains uncalibrated",
        "simulation_time_window_s": [start, end],
        "truth_samples": len(truth_time),
        "truth_median_interval_s": float(np.median(intervals)),
        "truth_max_interval_s": float(np.max(intervals)),
        "truth_effective_rate_hz": float((len(truth_time) - 1) / (truth_time[-1] - truth_time[0])),
        "navigation_samples": len(nav_time),
        "navigation_output_rate_hz": 100,
        "source_navigation_median_interval_s": float(np.median(np.diff(nav_time_raw))),
        "truth_attitude_max_interpolation_gap_s": truth_attitude_gap,
        "navigation_position_max_interpolation_gap_s": nav_position_gap,
        "navigation_attitude_max_interpolation_gap_s": nav_attitude_gap,
        "independent_clock_check_samples": len(route_time),
        "ekf_minus_mavlink_position_rms_m": np.sqrt(np.mean(clock_check_difference**2, axis=0)).tolist(),
        "ekf_minus_mavlink_position_max_abs_m": clock_check_max.tolist(),
        "files": {"truth": targets[0].name, "navigation": targets[1].name},
    }
    np.savetxt(targets[0], truth_values, delimiter=",", header=header, comments="", fmt="%.16g")
    np.savetxt(targets[1], nav_values, delimiter=",", header=header, comments="", fmt="%.16g")
    targets[2].write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(align(args.directory, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
