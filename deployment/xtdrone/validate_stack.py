"""启动自有 Gazebo/PX4/XTDrone2 进程，验证真实遥测及服务控制闭环。"""

import argparse
import csv
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Twist
from px4_msgs.msg import VehicleAttitude, VehicleCommandAck, VehicleLocalPosition, VehicleStatus
from rclpy.qos import qos_profile_sensor_data
from xtd2_msgs.msg import XTD2VehicleState
from xtd2_msgs.srv import XTD2Cmd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aligned_route import AlignedRoute  # noqa: E402
from scenario_profile import load_profile  # noqa: E402


def main():
    def terminate(signum, frame):
        raise KeyboardInterrupt("External GUI job cancelled")
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGHUP, terminate)
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", choices=("hex_x12", "x500"), default="hex_x12")
    parser.add_argument("--route", action="store_true", help="Fly the 300 m, 20.48 s collection route")
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--truth-rate-hz", type=int, default=0)
    parser.add_argument("--aligned-route", action="store_true", help="Use the common external baseline protocol")
    parser.add_argument("--route-config", type=Path, help="Shared route JSON; requires --aligned-route")
    args = parser.parse_args()
    if args.route_config and not args.aligned_route:
        parser.error("--route-config requires --aligned-route")
    if args.aligned_route and (not args.route or args.scenario):
        parser.error("--aligned-route requires --route and the no-added-wind baseline")
    if args.aligned_route and not args.truth_rate_hz:
        parser.error("--aligned-route requires a native Gazebo truth stream")
    aligned = (AlignedRoute.from_file(args.route_config) if args.route_config else AlignedRoute()) if args.aligned_route else None
    ready_since = None
    last_progress_clock = -math.inf
    if not 0 <= args.truth_rate_hz <= 1000:
        parser.error("--truth-rate-hz must be between 0 and 1000")
    scenario = load_profile(args.scenario) if args.scenario else None
    if scenario and args.model != "hex_x12":
        parser.error("scenario mode supports hex_x12 only")
    namespace = args.model + "_0"
    root = args.output.resolve() / ("xtdrone_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    root.mkdir(parents=True)
    if args.truth_rate_hz:
        shutil.copy2("/opt/uav_motion/models/hex_x12/model.sdf", root / "model_snapshot.sdf")
        shutil.copy2("/opt/uav_motion/models/hex_world.sdf", root / "world_snapshot.sdf")
    if scenario:
        shutil.copy2(args.scenario, root / "scenario.json")
        shutil.copy2("/opt/uav_motion/models/hex_x12/model.sdf", root / "scenario_model.sdf")
        shutil.copy2("/opt/uav_motion/models/hex_world.sdf", root / "scenario_world.sdf")
    base = Path("/opt/uav_motion")
    px4 = base / "PX4-Autopilot"
    env = os.environ.copy()
    env.update(
        GZ_PARTITION="uav_motion_hex_x12",
        PX4_UXRCE_DDS_NS=namespace,
        PX4_GZ_WORLD="default",
        PX4_SYS_AUTOSTART="6001" if args.model == "hex_x12" else "4001",
        PX4_SIM_MODEL=args.model,
        PX4_GZ_STANDALONE="1",
        PX4_GZ_MODEL_POSE="0,0,0.23,0,0,0",
    )
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        [
            str(base / "models"),
            str(px4 / "Tools/simulation/gz/models"),
            str(base / "xtdrone_ws/src/XTDrone2/xtd2_gz_sim/models"),
            env.get("GZ_SIM_RESOURCE_PATH", ""),
        ]
    )
    processes, streams = [], []
    report = {
        "aligned_route": asdict(aligned) if aligned else None,
        "passed": False,
        "truth_rate_requested_hz": args.truth_rate_hz or None,
        "model": args.model,
        "scenario": scenario["name"] if scenario else "baseline",
        "scenario_seed": scenario["seed"] if scenario else None,
        "diagnostic_reference_only": args.model != "hex_x12",
        "scope": (
            (f"PX4/XTDrone2 {aligned.altitude_m:g} m, {aligned.speed_mps:g} m/s, {aligned.duration_s:g} s collection route" if aligned else "PX4/XTDrone2 300 m, 10 m/s, 20.48 s collection route; navigation and Gazebo truth")
            if args.route else "PX4/XTDrone2 5 m flight smoke"
        ),
        "timestamp_reference": "PX4 DDS timestamp synchronized to ROS clock; seconds",
        "position_reference": "local EKF NED origin, Z negated to up; not APC or SAR pulse truth",
    }
    rows, attitudes, acks, truth_rows = [], [], [], []
    route_positions, target_rows = [], []
    route_phase, route_start_s = "takeoff", None
    last_target_sent = -math.inf
    latest = {"position": None, "status": None, "debug_samples": 0}
    node = None

    def start(name, command):
        stream = (root / (name + ".log")).open("w")
        streams.append(stream)
        process = subprocess.Popen(
            command, cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True
        )
        processes.append(process)
        return process

    try:
        gazebo_command = ["gz", "sim", "-s", "-r", "-v", "2"]
        if scenario:
            gazebo_command += ["--seed", str(scenario["seed"])]
        gazebo_command += [str(base / "models/hex_world.sdf")]
        start("gazebo", gazebo_command)
        truth_log = (root / "gazebo_truth.log").open("w")
        streams.append(truth_log)
        truth_topic = "/uav_motion/truth_pose" if args.truth_rate_hz else "/world/default/dynamic_pose/info"
        truth_process = subprocess.Popen(
            ["gz", "topic", "-e", "--json-output", "-t", truth_topic],
            cwd=root, env=env, stdout=subprocess.PIPE, stderr=truth_log,
            text=True, bufsize=1, start_new_session=True,
        )
        processes.append(truth_process)

        def capture_truth():
            last_time = float("-inf")
            for line in truth_process.stdout:
                try:
                    item = json.loads(line)
                    pose = next(p for p in item["pose"] if p["name"] == f"{args.model}_0")
                    stamp = (pose if args.truth_rate_hz else item)["header"]["stamp"]
                    stamp_s = int(stamp["sec"]) + int(stamp.get("nsec", 0)) / 1e9
                    if stamp_s <= last_time or (not args.truth_rate_hz and stamp_s - last_time < 0.01):
                        continue
                    point, rotation = pose.get("position", {}), pose.get("orientation", {})
                    truth_rows.append([
                        stamp_s, time.monotonic(),
                        point.get("x", 0.0), point.get("y", 0.0), point.get("z", 0.0),
                        rotation.get("w", 0.0), rotation.get("x", 0.0),
                        rotation.get("y", 0.0), rotation.get("z", 0.0),
                    ])
                    last_time = stamp_s
                except (ValueError, KeyError, StopIteration, TypeError):
                    continue

        truth_thread = threading.Thread(target=capture_truth, daemon=True)
        truth_thread.start()
        start("agent", ["MicroXRCEAgent", "udp4", "-p", "8888", "-v", "3"])
        time.sleep(3)
        start(
            "px4",
            [
                str(px4 / "build/px4_sitl_default/bin/px4"),
                "-d",
                "-i",
                "0",
                "-s",
                str(px4 / "build/px4_sitl_default/etc/init.d-posix/rcS"),
                str(px4 / "ROMFS/px4fmu_common"),
                "-w",
                str(root),
            ],
        )
        start(
            "xtdrone",
            [
                "ros2",
                "run",
                "xtd2_communication",
                "multirotor_communication",
                "--model",
                "gz_" + args.model,
                "--id",
                "0",
                "--namespace",
                namespace,
                "--debug",
                "true",
            ],
        )
        rclpy.init()
        node = rclpy.create_node("uav_motion_deployment_validation")

        def position(msg):
            latest["position"] = msg
            rows.append(
                [
                    msg.timestamp / 1e6,
                    msg.x,
                    msg.y,
                    -msg.z,
                    msg.vx,
                    msg.vy,
                    -msg.vz,
                    msg.xy_valid,
                    msg.z_valid,
                    time.monotonic(),
                ]
            )
            if args.route and route_phase == "capture":
                route_positions.append([msg.timestamp / 1e6, msg.x, msg.y, -msg.z])

        def attitude(msg):
            attitudes.append([msg.timestamp / 1e6, *msg.q])

        def status(msg):
            latest["status"] = msg

        def debug(msg):
            latest["debug_samples"] += 1
            latest["offboard_state"] = msg.offboard_state

        def ack(msg):
            acks.append({"command": int(msg.command), "result": int(msg.result)})

        for message, topic, callback in [
            (VehicleLocalPosition, f"/{namespace}/fmu/out/vehicle_local_position", position),
            (VehicleAttitude, f"/{namespace}/fmu/out/vehicle_attitude", attitude),
            (VehicleStatus, f"/{namespace}/fmu/out/vehicle_status", status),
            (VehicleCommandAck, f"/{namespace}/fmu/out/vehicle_command_ack", ack),
            (XTD2VehicleState, f"/xtdrone2/{namespace}/debug/vehicle_state", debug),
        ]:
            node.create_subscription(message, topic, callback, qos_profile_sensor_data)
        client = node.create_client(XTD2Cmd, f"/xtdrone2/{namespace}/cmd")
        publisher = node.create_publisher(Pose, f"/xtdrone2/{namespace}/cmd_pose_local_ned", 10)
        velocity_publisher = node.create_publisher(Twist, f"/xtdrone2/{namespace}/cmd_vel_ned", 10)

        def send(command):
            request = XTD2Cmd.Request()
            request.command = command
            client.call_async(request)

        started = time.monotonic()
        enabled = False
        warm = 0.0
        last_arm = 0.0
        armed_once = False
        land_sent = False
        peak = 0.0
        while time.monotonic() - started < (max(480, 300 + aligned.duration_s * 3) if aligned else 480 if args.route else 100):
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.monotonic() - started
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("A stack component exited; inspect component logs")
            pos = latest["position"]
            state = latest["status"]
            simulation_clock = truth_rows[-1][0] if aligned and truth_rows else None
            if pos and pos.z_valid:
                peak = max(peak, -pos.z)
            if args.route and armed_once and pos and pos.xy_valid and pos.z_valid:
                if route_phase == "takeoff" and -pos.z >= (aligned.altitude_m-1 if aligned else 295):
                    route_phase = "reposition"
                if aligned and simulation_clock is not None:
                    pose_state = (pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz)
                    clock = simulation_clock
                    if route_phase == "reposition":
                        ready_since = aligned.settle_start(ready_since, clock, pose_state, pos.heading)
                        if ready_since is not None and clock - ready_since >= aligned.settle_s:
                            route_phase = "accelerate"
                    if route_phase == "accelerate" and aligned.capture_ready(*pose_state, pos.heading):
                        route_phase, route_start_s = "capture", clock
                        report["capture_start_nav_ned"] = [*pose_state, pos.heading]
                        route_positions.append([pos.timestamp / 1e6, pos.x, pos.y, -pos.z])
                elif not aligned:
                    if route_phase == "reposition" and abs(pos.x + 150) <= 3 and abs(pos.y) <= 3 and abs(pos.z + 300) <= 5:
                        route_phase = "accelerate"
                    if route_phase == "accelerate" and pos.x >= -102.4 and pos.vx >= 9 and abs(pos.z + 300) <= 5:
                        route_phase = "capture"
                        route_start_s = pos.timestamp / 1e6
                        route_positions.append([route_start_s, pos.x, pos.y, -pos.z])
                clock = simulation_clock if aligned else pos.timestamp / 1e6
                if route_phase == "capture" and clock - route_start_s >= (aligned.duration_s if aligned else 20.48):
                    route_phase = "complete"
                    if aligned:
                        report["capture_gazebo_clock_s"] = [route_start_s, clock]
            if (
                not enabled
                and elapsed > 15
                and pos
                and pos.xy_valid
                and pos.z_valid
                and client.service_is_ready()
            ):
                send("OFFBOARD")
                enabled = True
                warm = elapsed
            if aligned and simulation_clock is not None and simulation_clock-last_progress_clock >= 1:
                last_progress_clock = simulation_clock
                print("UAVSIM_EXTERNAL_PROGRESS " + json.dumps({"phase": route_phase,
                    "elapsed_s": max(0, simulation_clock-route_start_s) if route_start_s is not None else 0,
                    "duration_s": aligned.duration_s}), flush=True)
            send_target = not aligned or (pos is not None and simulation_clock is not None
                and simulation_clock - last_target_sent >= aligned.command_period_s - 1e-9)
            if enabled and not land_sent and send_target:
                if aligned:
                    last_target_sent = simulation_clock
                if args.route and route_phase in ("accelerate", "capture"):
                    velocity = Twist()
                    if aligned:
                        velocity.linear.x, velocity.linear.y, velocity.linear.z = aligned.velocity_ned(pos.y, pos.z)
                        velocity.angular.z = aligned.yaw_rate(pos.heading)
                    else:
                        velocity.linear.x = 10.0
                    velocity_publisher.publish(velocity)
                    if pos is not None:
                        clock = simulation_clock if aligned else pos.timestamp / 1e6
                        reference_x = ((aligned.start_x_m if aligned else -102.4) if route_phase == "accelerate" else (aligned.start_x_m + aligned.speed_mps * min(aligned.duration_s, clock-route_start_s) if aligned else min(102.4, -102.4 + 10 * (clock - route_start_s))))
                        target_rows.append([pos.timestamp / 1e6, reference_x, aligned.center_y_m if aligned else 0.0, aligned.altitude_m if aligned else 300.0, "velocity"])
                else:
                    pose = Pose()
                    pose.position.z = (-aligned.altitude_m if aligned else -300.0) if args.route else -5.0
                    pose.position.x = (aligned.reposition_x_m if aligned else -150.0) if args.route and route_phase == "reposition" else 0.0
                    pose.position.y = aligned.center_y_m if aligned else 0.0
                    pose.orientation.w = 1.0
                    publisher.publish(pose)
                    if args.route and pos is not None:
                        target_rows.append([pos.timestamp / 1e6, pose.position.x, 0.0, -pose.position.z, "position"])
                if elapsed - warm > 2 and elapsed - last_arm > 4 and not armed_once:
                    send("OFFBOARD")
                    send("ARM")
                    last_arm = elapsed
            if state and state.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                armed_once = True
            elif armed_once and not land_sent:
                raise RuntimeError("PX4 disarmed before landing request; inspect ULog failsafe flags")
            if armed_once and (route_phase == "complete" if args.route else peak > 4.0) and not land_sent:
                send("LAND")
                land_sent = True
            if armed_once and land_sent and state and state.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                report["passed"] = (
                    route_phase == "complete"
                    and len(route_positions) >= 100
                    and route_positions[-1][1] - route_positions[0][1] >= (0.85 * aligned.speed_mps * aligned.duration_s if aligned else 180)
                    and abs(sum(row[3] for row in route_positions) / len(route_positions) - (aligned.altitude_m if aligned else 300)) <= 10
                    and math.sqrt(sum((row[2] - (aligned.center_y_m if aligned else 0)) ** 2 for row in route_positions) / len(route_positions)) <= 10
                    if args.route else peak > 4 and latest["debug_samples"] > 10 and len(attitudes) > 10
                )
                break
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
            landing_requested=land_sent,
            max_relative_height_m=peak,
            position_samples=len(rows),
            attitude_samples=len(attitudes),
            xtdrone_state_samples=latest["debug_samples"],
            xtdrone_offboard_state_end=latest.get("offboard_state"),
        )
    except Exception as error:
        report["error"] = repr(error)
    finally:
        if node:
            node.destroy_node()
            rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if "truth_thread" in locals():
            truth_thread.join(timeout=3)
        for stream in streams:
            stream.close()
    report["gazebo_truth_samples"] = len(truth_rows)
    if args.truth_rate_hz and not truth_rows:
        report["passed"] = False
        report["error"] = "No Gazebo truth messages at the requested higher rate"
    for filename, labels, data in [
        (
            "navigation_position_sar.csv",
            [
                "timestamp_s",
                "x_sar_m",
                "y_sar_m",
                "z_sar_m",
                "vx_sar_mps",
                "vy_sar_mps",
                "vz_sar_mps",
                "xy_valid",
                "z_valid",
                "receipt_monotonic_s",
            ],
            rows,
        ),
        ("navigation_attitude.csv", ["timestamp_s", "qw_nb", "qx_nb", "qy_nb", "qz_nb"], attitudes),
    ]:
        with (root / filename).open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(labels)
            writer.writerows(data)
    (root / "acks.json").write_text(json.dumps(acks, indent=2), encoding="utf-8")
    with (root / "gazebo_truth_enu.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sim_time_s", "receipt_monotonic_s", "x_enu_m", "y_enu_m", "z_up_m", "qw", "qx", "qy", "qz"])
        writer.writerows(truth_rows)
    if args.route:
        for filename, data in (("route_navigation_sar.csv", route_positions), ("route_target_sar.csv", target_rows)):
            with (root / filename).open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["timestamp_s", "x_sar_m", "y_sar_m", "z_sar_m", "control_mode"] if filename == "route_target_sar.csv" else ["timestamp_s", "x_sar_m", "y_sar_m", "z_sar_m"])
                writer.writerows(data)
    (root / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"directory": str(root), **report}, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
