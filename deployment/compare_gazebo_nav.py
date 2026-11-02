"""按接收时刻对照 PX4 导航和 Gazebo 位姿；保留时钟/原点不确定性。"""

import argparse
import bisect
import csv
import json
import math
import statistics
from pathlib import Path


def read_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def quaternion_product(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, ...]:
    """Hamilton scalar-first quaternion product."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def gazebo_to_ned_frd(q_enu_flu: tuple[float, ...]) -> tuple[float, ...]:
    """Map Gazebo ENU/FLU pose to PX4 NED/FRD body-to-earth attitude."""
    half_sqrt_two = 1 / math.sqrt(2)
    q_ned_enu = (0, half_sqrt_two, half_sqrt_two, 0)
    q_flu_frd = (0, 1, 0, 0)
    return quaternion_product(quaternion_product(q_ned_enu, q_enu_flu), q_flu_frd)


def attitude_distance_deg(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    """Shortest SO(3) angle, allowing quaternion sign ambiguity."""
    a_norm = math.sqrt(sum(value * value for value in a))
    b_norm = math.sqrt(sum(value * value for value in b))
    if not math.isfinite(a_norm * b_norm) or a_norm == 0 or b_norm == 0:
        raise ValueError("invalid attitude quaternion")
    cosine = abs(sum(x * y for x, y in zip(a, b))) / (a_norm * b_norm)
    return math.degrees(2 * math.acos(min(1, cosine)))


def compare(directory: Path, aligned_output: Path | None = None) -> dict:
    """输入 PX4/Gazebo CSV，返回采集段位置差；m，时间差 s。

    Gazebo ENU 映射至局部 SAR 为 (Y, X, Z)。用两进程的单调时钟接收时间近邻配对，
    不补偿飞控估计延迟、通信延迟或局部原点偏移，因此差值不是已标定导航精度。
    """
    nav = read_rows(directory / "navigation_position_sar.csv")
    truth = read_rows(directory / "gazebo_truth_enu.csv")
    attitude_path = directory / "navigation_attitude.csv"
    attitudes = read_rows(attitude_path) if attitude_path.exists() else []
    route = read_rows(directory / "route_navigation_sar.csv")
    commands = read_rows(directory / "route_target_sar.csv") if (directory / "route_target_sar.csv").exists() else []
    if not nav or not truth or not route:
        raise ValueError("navigation, Gazebo truth and route CSV must all be nonempty")
    begin, end = float(route[0]["timestamp_s"]), float(route[-1]["timestamp_s"])
    if end <= begin:
        raise ValueError("route timestamps are not increasing")
    # PX4 DDS may briefly emit boot-time timestamps while its ROS clock resynchronizes.
    # This route launcher runs for at most 480 s; reject only stamps over 600 s away
    # from the capture window, and retain the raw CSV for inspection.
    nav_count, attitude_count = len(nav), len(attitudes)
    nav = [row for row in nav if begin - 600 <= float(row["timestamp_s"]) <= end + 600]
    attitudes = [row for row in attitudes if begin - 600 <= float(row["timestamp_s"]) <= end + 600]
    truth_times = [float(row["receipt_monotonic_s"]) for row in truth]
    command_times = [float(row["timestamp_s"]) for row in commands]
    attitude_times = [float(row["timestamp_s"]) for row in attitudes]
    if any(b <= a for a, b in zip(truth_times, truth_times[1:])):
        raise ValueError("Gazebo receipt timestamps are not strictly increasing")
    if any(b < a for a, b in zip(attitude_times, attitude_times[1:])):
        raise ValueError("navigation attitude timestamps are not increasing")
    preflight_differences = []
    for row in nav:
        if (float(row["timestamp_s"]) >= begin or row["xy_valid"].lower() != "true"
                or row["z_valid"].lower() != "true" or abs(float(row["z_sar_m"])) > 2):
            continue
        arrival = float(row["receipt_monotonic_s"])
        k = bisect.bisect_left(truth_times, arrival)
        j = min((max(0, k - 1), min(k, len(truth) - 1)), key=lambda index: abs(truth_times[index] - arrival))
        if abs(truth_times[j] - arrival) <= 0.02:
            reference = truth[j]
            difference = [
                float(row["x_sar_m"]) - float(reference["y_enu_m"]),
                float(row["y_sar_m"]) - float(reference["x_enu_m"]),
                float(row["z_sar_m"]) - float(reference["z_up_m"]),
            ]
            if all(math.isfinite(value) for value in difference):
                preflight_differences.append(difference)
    origin_offset = ([statistics.median(row[i] for row in preflight_differences) for i in range(3)]
                     if len(preflight_differences) >= 10 else None)
    differences = []
    relative_differences = []
    timing = []
    attitude_errors = []
    aligned = []
    for row in nav:
        nav_stamp = float(row["timestamp_s"])
        if not begin <= nav_stamp <= end or row["xy_valid"].lower() != "true" or row["z_valid"].lower() != "true":
            continue
        arrival = float(row["receipt_monotonic_s"])
        k = bisect.bisect_left(truth_times, arrival)
        j = min((max(0, k - 1), min(k, len(truth) - 1)), key=lambda index: abs(truth_times[index] - arrival))
        delta = abs(truth_times[j] - arrival)
        if delta > 0.02:
            continue
        reference = truth[j]
        actual = (float(reference["y_enu_m"]), float(reference["x_enu_m"]), float(reference["z_up_m"]))
        estimated = (float(row["x_sar_m"]), float(row["y_sar_m"]), float(row["z_sar_m"]))
        if not all(math.isfinite(x) for x in (*actual, *estimated)):
            continue
        raw_difference = [estimate - target for estimate, target in zip(estimated, actual)]
        differences.append(raw_difference)
        relative_difference = ([value - offset for value, offset in zip(raw_difference, origin_offset)]
                               if origin_offset is not None else None)
        if relative_difference is not None:
            relative_differences.append(relative_difference)
        timing.append(delta)
        attitude_gap = ""
        attitude_error = ""
        if attitudes:
            attitude_index = bisect.bisect_left(attitude_times, nav_stamp)
            attitude_index = min((max(0, attitude_index - 1), min(attitude_index, len(attitudes) - 1)),
                                 key=lambda index: abs(attitude_times[index] - nav_stamp))
            attitude_gap = abs(attitude_times[attitude_index] - nav_stamp)
            if attitude_gap <= 0.02:
                nav_attitude = tuple(float(attitudes[attitude_index][key]) for key in
                                     ("qw_nb", "qx_nb", "qy_nb", "qz_nb"))
                truth_attitude = gazebo_to_ned_frd(tuple(float(reference[key]) for key in
                                                         ("qw", "qx", "qy", "qz")))
                try:
                    attitude_error = attitude_distance_deg(nav_attitude, truth_attitude)
                    attitude_errors.append(attitude_error)
                except ValueError:
                    pass
        command = None
        if commands:
            command_index = bisect.bisect_left(command_times, nav_stamp)
            command = commands[min(
                (max(0, command_index - 1), min(command_index, len(commands) - 1)),
                key=lambda index: abs(command_times[index] - nav_stamp),
            )]
        aligned.append({
            "nav_timestamp_s": nav_stamp,
            "nav_receipt_monotonic_s": arrival,
            "truth_sim_time_s": float(reference["sim_time_s"]),
            "truth_receipt_monotonic_s": truth_times[j],
            "receipt_time_gap_s": delta,
            "nav_x_sar_m": estimated[0], "nav_y_sar_m": estimated[1], "nav_z_sar_m": estimated[2],
            "truth_x_sar_m": actual[0], "truth_y_sar_m": actual[1], "truth_z_sar_m": actual[2],
            "nav_minus_truth_x_m": estimated[0] - actual[0],
            "nav_minus_truth_y_m": estimated[1] - actual[1],
            "nav_minus_truth_z_m": estimated[2] - actual[2],
            "preflight_origin_corrected_x_m": relative_difference[0] if relative_difference else "",
            "preflight_origin_corrected_y_m": relative_difference[1] if relative_difference else "",
            "preflight_origin_corrected_z_m": relative_difference[2] if relative_difference else "",
            "command_timestamp_s": float(command["timestamp_s"]) if command else "",
            "command_x_sar_m": float(command["x_sar_m"]) if command else "",
            "command_y_sar_m": float(command["y_sar_m"]) if command else "",
            "command_z_sar_m": float(command["z_sar_m"]) if command else "",
            "control_mode": command.get("control_mode", "") if command else "",
            "attitude_nav_timestamp_gap_s": attitude_gap,
            "nav_minus_truth_attitude_angle_deg": attitude_error,
        })
    if len(differences) < 100:
        raise ValueError("fewer than 100 route samples have a Gazebo match within 20 ms")
    if aligned_output is not None:
        with aligned_output.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(aligned[0]))
            writer.writeheader()
            writer.writerows(aligned)
    count = len(differences)
    return {
        "status": "matched_raw_positions",
        "samples": count,
        "route_navigation_samples": len(route),
        "navigation_timestamp_outlier_samples": nav_count - len(nav),
        "attitude_timestamp_outlier_samples": attitude_count - len(attitudes),
        "receipt_time_gap_max_s": max(timing),
        "receipt_time_gap_mean_s": sum(timing) / count,
        "navigation_minus_gazebo_mean_sar_m": [sum(row[j] for row in differences) / count for j in range(3)],
        "navigation_minus_gazebo_rms_sar_m": [math.sqrt(sum(row[j] ** 2 for row in differences) / count) for j in range(3)],
        "preflight_origin_samples": len(preflight_differences),
        "preflight_origin_offset_sar_m": origin_offset,
        "preflight_origin_corrected_rms_sar_m": (
            [math.sqrt(sum(row[j] ** 2 for row in relative_differences) / count) for j in range(3)]
            if origin_offset is not None else None
        ),
        "attitude_matched_samples": len(attitude_errors),
        "navigation_minus_gazebo_attitude_rms_deg": (
            math.sqrt(sum(value ** 2 for value in attitude_errors) / len(attitude_errors))
            if attitude_errors else None
        ),
        "frame_mapping": "Gazebo ENU (Y, X, Z) -> SAR (X forward, Y right, Z up)",
        "attitude_mapping": "Gazebo ENU/FLU body-to-earth -> PX4 NED/FRD body-to-earth",
        "time_mapping": "nearest local monotonic receipt time; no estimator or transport delay correction",
        "interpretation": "Raw and stationary-preflight-baseline-corrected differences; baseline also includes estimator bias. Timing/estimator latency remains; neither is calibrated navigation accuracy or SAR pulse truth",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    aligned_path = args.directory / "route_aligned_sar.csv"
    result = compare(args.directory, aligned_path)
    path = args.directory / "navigation_truth_comparison.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"{result['samples']} matched samples; {aligned_path.name}, {path.name}")


if __name__ == "__main__":
    main()
