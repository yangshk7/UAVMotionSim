"""真实 SITL 解锁、起飞、降落验证；不连接物理设备，仅启动本次子进程。"""

import argparse
import csv
import hashlib
import json
import math
import shutil
import signal
import socket
import subprocess
import sys
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aligned_route import AlignedRoute  # noqa: E402
from scenario_profile import load_profile  # noqa: E402


def main():
    def terminate(signum, frame):
        raise KeyboardInterrupt("External GUI job cancelled")
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGHUP, terminate)
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--defaults", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--route", action="store_true", help="Fly the 300 m, 20.48 s collection route")
    parser.add_argument("--scenario", type=Path, help="Optional external simulator scenario JSON")
    parser.add_argument("--truth-rate-hz", type=int, default=300, help="Requested MAVLink SIMSTATE/SIM_STATE Hz")
    parser.add_argument("--aligned-route", action="store_true", help="Use the common external baseline protocol")
    parser.add_argument("--route-config", type=Path, help="Shared route JSON; requires --aligned-route")
    args = parser.parse_args()
    if args.route_config and not args.aligned_route:
        parser.error("--route-config requires --aligned-route")
    if args.aligned_route and (not args.route or args.scenario):
        parser.error("--aligned-route requires --route and the no-added-wind baseline")
    aligned = (AlignedRoute.from_file(args.route_config) if args.route_config else AlignedRoute()) if args.aligned_route else None
    ready_since = None
    last_progress_clock = -math.inf
    if not 1 <= args.truth_rate_hz <= 1000:
        parser.error("--truth-rate-hz must be between 1 and 1000")
    scenario = load_profile(args.scenario) if args.scenario else None
    root = args.output.resolve() / ("sitl_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    root.mkdir(parents=True)
    config = Path(__file__).resolve().parent
    binary = args.binary.resolve()
    for source in (config / "hex_x12.json", config / "hex_x12.parm", args.defaults.resolve()):
        shutil.copy2(source, root / source.name)
    defaults = f"{args.defaults.name},hex_x12.parm"
    if scenario:
        shutil.copy2(args.scenario, root / "scenario.json")
        sitl = scenario["sitl"]
        model_path = root / "hex_x12.json"
        model = json.loads(model_path.read_text(encoding="utf-8"))
        model["mdrag_coef"] = sitl["momentum_drag"]
        model_path.write_text(json.dumps(model, indent=2) + "\n", encoding="utf-8")
        parameters = {
            "SIM_WIND_SPD": sitl["wind_speed_mps"],
            "SIM_WIND_DIR": sitl["wind_from_deg"],
            "SIM_WIND_TURB": sitl["wind_turbulence"],
            "SIM_ACC1_RND": sitl["accel_noise_mps2"],
            "SIM_ACC2_RND": sitl["accel_noise_mps2"],
            "SIM_ACC3_RND": sitl["accel_noise_mps2"],
        }
        (root / "scenario.parm").write_text(
            "".join(f"{key} {value}\n" for key, value in parameters.items()), encoding="ascii"
        )
        defaults += ",scenario.parm"
    if aligned:
        (root / "aligned_route.parm").write_text("SIM_WIND_SPD 0\nSIM_WIND_TURB 0\n", encoding="ascii")
        defaults += ",aligned_route.parm"
    command = [
        str(binary),
        "-I",
        "20",
        "--model",
        "hexax:hex_x12.json",
        "--defaults",
        defaults,
        "--speedup",
        "5",
        "--home",
        "-35.363261,149.165230,584,0",
        "-w",
    ]
    report = {
        "aligned_route": asdict(aligned) if aligned else None,
        "command": command,
        "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "scenario": scenario["name"] if scenario else "baseline",
        "scenario_seed": scenario["seed"] if scenario else None,
        "scenario_seed_note": "ArduPilot SITL does not expose a run seed through this launcher; scenario seed is metadata only",
        "scenario_model_momentum_drag": scenario["sitl"]["momentum_drag"] if scenario else None,
        "passed": False,
        "truth_rate_requested_hz": args.truth_rate_hz,
        "scope": (
            (f"12 kg Hex-X {aligned.altitude_m:g} m, {aligned.speed_mps:g} m/s, {aligned.duration_s:g} s collection route" if aligned else "12 kg Hex-X 300 m, 10 m/s, 20.48 s collection route; navigation and simulator truth")
            if args.route else "12 kg Hex-X 10 m takeoff/land smoke; not SAR-rate truth"
        ),
    }
    position, attitude, simulation_truth, simulation_truth_precise, messages = [], [], [], [], []
    route_positions, target_log = [], []
    connection = None
    with (root / "sitl.log").open("w") as log:
        process = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT)
        try:
            for _ in range(60):
                try:
                    with socket.create_connection(("127.0.0.1", 5960), timeout=0.5):
                        pass
                    connection = mavutil.mavlink_connection("tcp:127.0.0.1:5960")
                    break
                except OSError:
                    if process.poll() is not None:
                        raise RuntimeError("SITL exited before MAVLink connection")
                    time.sleep(0.5)
            if connection is None or connection.wait_heartbeat(timeout=15) is None:
                raise TimeoutError("No SITL heartbeat")
            if scenario:
                loaded = {}
                for parameter, expected in parameters.items():
                    connection.param_fetch_one(parameter)
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        reply = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                        if reply and reply.param_id.rstrip("\x00") == parameter:
                            loaded[parameter] = float(reply.param_value)
                            break
                    if parameter not in loaded or abs(loaded[parameter] - expected) > 1e-3:
                        raise RuntimeError(f"SITL scenario parameter {parameter} not confirmed")
                report["scenario_parameters_confirmed"] = loaded
            for msg_id in (32, 30, 108, 164):
                connection.mav.command_long_send(
                    connection.target_system,
                    connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    round(1_000_000 / (args.truth_rate_hz if msg_id in (108, 164) else 50)),
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            started = time.monotonic()
            last_arm = 0.0
            takeoff_sent = False
            land_sent = False
            armed_once = False
            highest = 0.0
            route_phase = "takeoff"
            route_start_s = None
            route_positions = []
            last_target_sent = -math.inf
            capture_yaw = 0.0
            target_log = []

            def target(x_m: float, y_m: float, z_down_m: float, boot_ms: int, velocity_mode: bool):
                vx, vy, vz = (10.0, 0.0, 0.0) if velocity_mode else (0.0, 0.0, 0.0)
                yaw_rate = 0.0
                mask = 0x0DC7 if velocity_mode else 0x0DF8
                if aligned:
                    if velocity_mode:
                        vx, vy, vz = aligned.velocity_ned(position[-1][2], -position[-1][3])
                        yaw_rate = aligned.yaw_rate(capture_yaw)
                        mask &= ~(1 << 11)
                    else:
                        mask &= ~(1 << 10)
                connection.mav.set_position_target_local_ned_send(
                    boot_ms, connection.target_system, connection.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                    0 if velocity_mode else x_m, 0 if velocity_mode else y_m,
                    0 if velocity_mode else z_down_m,
                    vx, vy, vz, 0, 0, 0, 0, yaw_rate,
                )
                target_log.append([boot_ms / 1000, x_m, y_m, -z_down_m, "velocity" if velocity_mode else "position"])

            while time.monotonic() - started < (max(260, 180 + aligned.duration_s / 2) if aligned else 260 if args.route else 120):
                elapsed = time.monotonic() - started
                msg = connection.recv_match(blocking=True, timeout=0.2)
                if msg:
                    received_at = time.monotonic()
                    kind = msg.get_type()
                    if kind == "STATUSTEXT":
                        messages.append({"elapsed_s": elapsed, "text": msg.text})
                    elif kind == "SIM_STATE":
                        # This ArduPilot build transmits degE7-like values in these float
                        # fields; preserve them raw and use SIMSTATE's int32 coordinates.
                        simulation_truth.append(
                            [received_at, msg.lat, msg.lon, msg.alt,
                             msg.vn, msg.ve, msg.vd, msg.q1, msg.q2, msg.q3, msg.q4]
                        )
                    elif kind == "SIMSTATE":
                        simulation_truth_precise.append(
                            [received_at, msg.lat, msg.lng, msg.roll, msg.pitch, msg.yaw]
                        )
                    elif kind == "LOCAL_POSITION_NED":
                        highest = max(highest, -msg.z)
                        position.append(
                            [msg.time_boot_ms / 1000, msg.x, msg.y, -msg.z, msg.vx, msg.vy, -msg.vz,
                             received_at]
                        )
                        if args.route and takeoff_sent and not land_sent:
                            if route_phase == "takeoff" and -msg.z >= (aligned.altitude_m-1 if aligned else 295):
                                route_phase = "reposition"
                            if aligned:
                                state = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
                                clock = msg.time_boot_ms / 1000
                                if route_phase == "reposition":
                                    ready_since = aligned.settle_start(ready_since, clock, state, capture_yaw)
                                    if ready_since is not None and clock - ready_since >= aligned.settle_s:
                                        route_phase = "accelerate"
                                if route_phase == "accelerate" and aligned.capture_ready(*state, capture_yaw):
                                    route_phase, route_start_s = "capture", clock
                                    report["capture_start_nav_ned"] = [*state, capture_yaw]
                            else:
                                if route_phase == "reposition" and abs(msg.x + 150) <= 3 and abs(msg.y) <= 3 and abs(msg.z + 300) <= 5:
                                    route_phase = "accelerate"
                                if route_phase == "accelerate" and msg.x >= -102.4 and msg.vx >= 9 and abs(msg.z + 300) <= 5:
                                    route_phase = "capture"
                                    route_start_s = msg.time_boot_ms / 1000
                            if route_phase == "capture":
                                route_positions.append([msg.time_boot_ms / 1000, msg.x, msg.y, -msg.z])
                                if msg.time_boot_ms / 1000 - route_start_s >= (aligned.duration_s if aligned else 20.48):
                                    route_phase = "complete"
                    elif kind == "ATTITUDE":
                        capture_yaw = msg.yaw
                        attitude.append(
                            [
                                msg.time_boot_ms / 1000,
                                msg.roll,
                                msg.pitch,
                                msg.yaw,
                                msg.rollspeed,
                                msg.pitchspeed,
                                msg.yawspeed,
                            ]
                        )
                    elif kind == "HEARTBEAT":
                        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        if armed:
                            armed_once = True
                            if not takeoff_sent:
                                connection.mav.command_long_send(
                                    connection.target_system,
                                    connection.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    (aligned.altitude_m if aligned else 300) if args.route else 10,
                                )
                                takeoff_sent = True
                        elif armed_once and land_sent:
                            report["passed"] = (
                                route_phase == "complete"
                                and len(route_positions) >= 100
                                and route_positions[-1][1] - route_positions[0][1] >= (0.85 * aligned.speed_mps * aligned.duration_s if aligned else 180)
                                and abs(sum(row[3] for row in route_positions) / len(route_positions) - (aligned.altitude_m if aligned else 300)) <= 10
                                and math.sqrt(sum((row[2] - (aligned.center_y_m if aligned else 0)) ** 2 for row in route_positions) / len(route_positions)) <= 10
                                if args.route else highest >= 8.0
                            )
                            break
                if aligned and position and position[-1][0] - last_progress_clock >= 1:
                    last_progress_clock = position[-1][0]
                    print("UAVSIM_EXTERNAL_PROGRESS " + json.dumps({"phase": route_phase,
                        "elapsed_s": max(0, last_progress_clock-route_start_s) if route_start_s is not None else 0,
                        "duration_s": aligned.duration_s}), flush=True)
                if not armed_once and elapsed > 12 and elapsed - last_arm > 4:
                    connection.set_mode("GUIDED")
                    connection.arducopter_arm()
                    last_arm = elapsed
                if args.route and takeoff_sent and not land_sent and position:
                    now = position[-1][0] if aligned else time.monotonic()
                    if route_phase == "complete":
                        connection.set_mode("LAND")
                        land_sent = True
                    elif route_phase in ("reposition", "accelerate", "capture") and now - last_target_sent >= (aligned.command_period_s if aligned else 0.1) - 1e-9:
                        current = position[-1]
                        x_goal = (
                            (aligned.reposition_x_m if aligned else -150.0) if route_phase == "reposition"
                            else (aligned.start_x_m if aligned else -102.4) if route_phase == "accelerate"
                            else (aligned.start_x_m + aligned.speed_mps * min(aligned.duration_s, current[0]-route_start_s) if aligned else min(102.4, -102.4 + 10 * (current[0] - route_start_s)))
                        )
                        target(x_goal, aligned.center_y_m if aligned else 0.0, -aligned.altitude_m if aligned else -300.0, round(current[0] * 1000), route_phase != "reposition")
                        last_target_sent = now
                elif takeoff_sent and highest >= 8.0 and not land_sent:
                    connection.set_mode("LAND")
                    land_sent = True
            if args.route:
                report.update(
                    route_phase=route_phase,
                    route_samples=len(route_positions),
                    route_duration_s=(route_positions[-1][0] - route_positions[0][0]) if len(route_positions) > 1 else 0,
                    route_x_span_m=(route_positions[-1][1] - route_positions[0][1]) if len(route_positions) > 1 else 0,
                    route_altitude_mean_m=(sum(row[3] for row in route_positions) / len(route_positions)) if route_positions else None,
                    route_cross_track_rms_m=(math.sqrt(sum((row[2] - (aligned.center_y_m if aligned else 0)) ** 2 for row in route_positions) / len(route_positions))) if route_positions else None,
                )
            report.update(
                armed=armed_once,
                takeoff_command_sent=takeoff_sent,
                landing_requested=land_sent,
                max_relative_height_m=highest,
                position_samples=len(position),
                attitude_samples=len(attitude),
                simulation_truth_samples=len(simulation_truth),
                simulation_truth_precise_samples=len(simulation_truth_precise),
            )
        except Exception as error:
            report["error"] = repr(error)
        finally:
            if connection:
                connection.close()
            process.terminate()
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    for filename, header, rows in [
        (
            "navigation_position_sar.csv",
            ["time_boot_s", "x_sar_m", "y_sar_m", "z_sar_m", "vx_sar_mps", "vy_sar_mps", "vz_sar_mps",
             "receipt_monotonic_s"],
            position,
        ),
        (
            "navigation_attitude_ned.csv",
            ["time_boot_s", "roll_rad", "pitch_rad", "yaw_rad", "p_radps", "q_radps", "r_radps"],
            attitude,
        ),
        (
            "simulation_truth_wgs84.csv",
            ["receipt_monotonic_s", "lat_raw_float", "lon_raw_float", "alt_m", "vn_mps", "ve_mps", "vd_mps",
             "qw", "qx", "qy", "qz"],
            simulation_truth,
        ),
        (
            "simulation_truth_precise_wgs84.csv",
            ["receipt_monotonic_s", "lat_degE7", "lon_degE7", "roll_rad", "pitch_rad", "yaw_rad"],
            simulation_truth_precise,
        ),
    ]:
        with (root / filename).open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)
    if args.truth_rate_hz > 50 and not simulation_truth_precise:
        report["passed"] = False
        report["error"] = "No SIMSTATE truth messages at the requested higher rate"
    (root / "messages.json").write_text(json.dumps(messages, indent=2), encoding="utf-8")
    if args.route:
        with (root / "route_target_sar.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_boot_s", "x_sar_m", "y_sar_m", "z_sar_m", "control_mode"])
            writer.writerows(target_log)
        with (root / "route_navigation_sar.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_boot_s", "x_sar_m", "y_sar_m", "z_sar_m"])
            writer.writerows(route_positions)
    (root / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"directory": str(root), **report}, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
