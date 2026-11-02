"""Align ArduPilot SITL navigation with the simulator's independent MAVLink state.

Position is local north/east/up in metres. Pairing uses the host's monotonic
receipt clock; it does not compensate estimator latency or transport delay.
"""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def read_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def nearest(times: list[float], stamp: float) -> int:
    index = bisect.bisect_left(times, stamp)
    return min((max(0, index - 1), min(index, len(times) - 1)),
               key=lambda i: abs(times[i] - stamp))


def local_neu(lat_deg: float, lon_deg: float, alt_m: float,
              origin_lat_deg: float, origin_lon_deg: float, origin_alt_m: float) -> tuple[float, float, float]:
    """WGS84 tangent plane at origin; valid for this sub-kilometre route."""
    radius_equator = 6378137.0
    eccentricity_sq = 6.69437999014e-3
    latitude = math.radians(origin_lat_deg)
    sin_latitude = math.sin(latitude)
    normal_radius = radius_equator / math.sqrt(1 - eccentricity_sq * sin_latitude**2)
    meridian_radius = radius_equator * (1 - eccentricity_sq) / (1 - eccentricity_sq * sin_latitude**2)**1.5
    north = math.radians(lat_deg - origin_lat_deg) * (meridian_radius + origin_alt_m)
    east = math.radians(lon_deg - origin_lon_deg) * (normal_radius + origin_alt_m) * math.cos(latitude)
    return north, east, alt_m - origin_alt_m


def euler_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """ZYX roll/pitch/yaw to scalar-first NED/FRD attitude quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def compare(directory: Path, aligned_output: Path | None = None) -> dict:
    """Return relative position and velocity differences; units m, m/s, s."""
    nav = read_rows(directory / "navigation_position_sar.csv")
    precise = read_rows(directory / "simulation_truth_precise_wgs84.csv")
    state = read_rows(directory / "simulation_truth_wgs84.csv")
    attitude = read_rows(directory / "navigation_attitude_ned.csv")
    route = read_rows(directory / "route_navigation_sar.csv")
    if not nav or not precise or not state or not attitude or not route:
        raise ValueError("navigation, attitude, both simulation state streams, and route must be nonempty")
    begin, end = float(route[0]["time_boot_s"]), float(route[-1]["time_boot_s"])
    if end <= begin:
        raise ValueError("route timestamps are not increasing")
    precise_times = [float(row["receipt_monotonic_s"]) for row in precise]
    state_times = [float(row["receipt_monotonic_s"]) for row in state]
    attitude_times = [float(row["time_boot_s"]) for row in attitude]
    if any(b <= a for times in (precise_times, state_times, attitude_times)
           for a, b in zip(times, times[1:])):
        raise ValueError("simulation receipt or attitude timestamps are not strictly increasing")
    origin_lat = int(precise[0]["lat_degE7"]) * 1e-7
    origin_lon = int(precise[0]["lon_degE7"]) * 1e-7
    origin_alt = float(state[0]["alt_m"])
    aligned = []
    all_relative = []
    all_velocity = []
    gaps = []
    attitude_errors_deg = []
    nav_origin = None
    truth_origin = None
    origin_nav_time_boot_s = None
    for row in nav:
        receipt = float(row["receipt_monotonic_s"])
        precise_index = nearest(precise_times, receipt)
        state_index = nearest(state_times, receipt)
        gap = max(abs(precise_times[precise_index] - receipt),
                  abs(state_times[state_index] - receipt))
        if gap > 0.02:
            continue
        p = precise[precise_index]
        s = state[state_index]
        truth_position = local_neu(int(p["lat_degE7"]) * 1e-7,
                                   int(p["lon_degE7"]) * 1e-7, float(s["alt_m"]),
                                   origin_lat, origin_lon, origin_alt)
        nav_position = tuple(float(row[key]) for key in ("x_sar_m", "y_sar_m", "z_sar_m"))
        boot_time = float(row["time_boot_s"])
        if nav_origin is None:
            if boot_time >= begin:
                raise ValueError("no pre-capture sample to establish relative origin")
            nav_origin, truth_origin = nav_position, truth_position
            origin_nav_time_boot_s = boot_time
        if not begin <= boot_time <= end:
            continue
        relative_error = tuple((n - n0) - (t - t0) for n, n0, t, t0 in
                               zip(nav_position, nav_origin, truth_position, truth_origin))
        truth_velocity = (float(s["vn_mps"]), float(s["ve_mps"]), -float(s["vd_mps"]))
        nav_velocity = tuple(float(row[key]) for key in ("vx_sar_mps", "vy_sar_mps", "vz_sar_mps"))
        velocity_error = tuple(n - t for n, t in zip(nav_velocity, truth_velocity))
        if not all(math.isfinite(value) for value in (*relative_error, *velocity_error)):
            continue
        attitude_index = nearest(attitude_times, boot_time)
        attitude_gap = abs(attitude_times[attitude_index] - boot_time)
        attitude_error_deg = None
        if attitude_gap <= 0.05:
            nav_attitude = attitude[attitude_index]
            nav_q = euler_quaternion(*(float(nav_attitude[key]) for key in
                                       ("roll_rad", "pitch_rad", "yaw_rad")))
            truth_q = tuple(float(s[key]) for key in ("qw", "qx", "qy", "qz"))
            norm = math.sqrt(sum(value * value for value in truth_q))
            if math.isfinite(norm) and norm > 0:
                cosine = abs(sum(a * b for a, b in zip(nav_q, truth_q))) / norm
                attitude_error_deg = math.degrees(2 * math.acos(min(1.0, cosine)))
                attitude_errors_deg.append(attitude_error_deg)
        aligned.append({
            "nav_time_boot_s": boot_time, "nav_receipt_monotonic_s": receipt,
            "truth_precise_receipt_monotonic_s": precise_times[precise_index],
            "truth_state_receipt_monotonic_s": state_times[state_index],
            "receipt_time_gap_s": gap,
            **{f"nav_{axis}_sar_m": nav_position[i] for i, axis in enumerate("xyz")},
            **{f"truth_{axis}_sar_m": truth_position[i] for i, axis in enumerate("xyz")},
            **{f"relative_nav_minus_truth_{axis}_m": relative_error[i] for i, axis in enumerate("xyz")},
            **{f"nav_v{axis}_sar_mps": nav_velocity[i] for i, axis in enumerate("xyz")},
            **{f"truth_v{axis}_sar_mps": truth_velocity[i] for i, axis in enumerate("xyz")},
            **{f"nav_minus_truth_v{axis}_mps": velocity_error[i] for i, axis in enumerate("xyz")},
            "truth_qw": float(s["qw"]), "truth_qx": float(s["qx"]),
            "truth_qy": float(s["qy"]), "truth_qz": float(s["qz"]),
            "attitude_nav_boot_gap_s": attitude_gap,
            "attitude_error_deg": attitude_error_deg if attitude_error_deg is not None else "",
        })
        all_relative.append(relative_error)
        all_velocity.append(velocity_error)
        gaps.append(gap)
    if len(aligned) < 100:
        raise ValueError("fewer than 100 route navigation samples have both truth streams within 20 ms")
    if aligned_output is not None:
        with aligned_output.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(aligned[0]))
            writer.writeheader()
            writer.writerows(aligned)

    def rms(rows: list[tuple[float, ...]]) -> list[float]:
        return [math.sqrt(sum(row[i] ** 2 for row in rows) / len(rows)) for i in range(3)]

    return {
        "status": "matched_relative_trajectory",
        "samples": len(aligned),
        "route_navigation_samples": len(route),
        "receipt_time_gap_max_s": max(gaps),
        "relative_navigation_minus_truth_position_rms_sar_m": rms(all_relative),
        "navigation_minus_truth_velocity_rms_sar_mps": rms(all_velocity),
        "attitude_matched_samples": len(attitude_errors_deg),
        "navigation_minus_truth_attitude_rms_deg": math.sqrt(
            sum(value**2 for value in attitude_errors_deg) / len(attitude_errors_deg)
        ) if attitude_errors_deg else None,
        "truth_origin_wgs84_deg_m": [origin_lat, origin_lon, origin_alt],
        "origin_nav_time_boot_s": origin_nav_time_boot_s,
        "relative_position_origin": "first time-matched navigation and truth sample before capture",
        "truth_sources": "SIMSTATE(164) degE7 latitude/longitude; SIM_STATE(108) altitude, velocity, quaternion",
        "sim_state_float_coordinates": "ignored: deployed ArduPilot sent degE7-like float values, despite MAVLink degree-unit declaration",
        "time_mapping": "nearest local monotonic receipt time; no estimator/transport latency correction",
        "interpretation": "SITL-only relative trajectory comparison; not calibrated absolute navigation accuracy or 800 Hz SAR pulse truth",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    aligned_path = args.directory / "route_truth_aligned_sar.csv"
    result = compare(args.directory, aligned_path)
    report_path = args.directory / "navigation_truth_comparison.json"
    report_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"{result['samples']} matched samples; {aligned_path.name}, {report_path.name}")


if __name__ == "__main__":
    main()
