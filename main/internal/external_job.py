"""GUI adapter for native WSL flights, clock alignment and common pose exports.

Native truth files are retained. Uniform truth exports interpolate native samples
at 300/1000 Hz; this does not create additional physical information.
"""

import json
import os
import subprocess
import sys
import threading
from pathlib import Path, PureWindowsPath

import numpy as np
from review_plots import plot_pose_comparisons
from review_report import create_report
from scipy.io import savemat
from scipy.spatial.transform import Rotation, Slerp

from uavsim.frames.coordinate import polar_to_sar
from uavsim.motion.export import export_motion_segment
from uavsim.motion.lever_arm import apc_from_pose
from uavsim.motion.segment import MotionSegment, MotionSeries
from uavsim.navigation.position_error import navigation_estimate
from uavsim.navigation.state_error import navigation_pose
from uavsim.runtime.progress import report_progress
from uavsim.sar_adapter.matlab_export import _mat_safe, _navigation_csv, write_json
from uavsim.trajectory.reference_manager import build_reference
from uavsim.visualization.navigation_plot import plot_navigation
from uavsim.visualization.trajectory_3d import plot_trajectories

PROJECT = Path(__file__).resolve().parents[2]
FIELDS = ("cg_x_sar_m", "cg_y_sar_m", "cg_z_sar_m",
          "cg_vx_sar_mps", "cg_vy_sar_mps", "cg_vz_sar_mps")
Q_FIELDS = ("qw_nb", "qx_nb", "qy_nb", "qz_nb")
LIMITATIONS = (
    "外部飞控使用固定12 kg Hex-X机型、原生控制器和原生传感器模型；"
    "上位机A/B扰动、Python控制增益、机体参数、初始状态误差、积分频率和预稳定时长不写入外部飞控。"
    "本入口运行无额外注入风扰的基准场景，仍保留原生传感器噪声及闭环跟踪误差。"
    "统一导航误差参数仅作用于shared_model，不作用于飞控自身估计。"
    "SITL位置/速度真值为SIM2，姿态由SIM四元数插值；PX4位置/姿态真值来自Gazebo，速度为位置差分。"
    "导航统一输出100 Hz；飞控原始导航可能低于100 Hz，插值不提高原始测量频率。"
)


def wsl_path(path):
    """Convert a Windows local drive path to its WSL /mnt mount path."""
    path = PureWindowsPath(str(path))
    if not path.drive or len(path.drive) != 2 or path.drive[1] != ":":
        raise ValueError("外部链路当前要求项目位于 Windows 本地盘符目录。")
    return "/mnt/" + path.drive[0].lower() + "/" + "/".join(path.parts[1:])


def route_spec(config, pass_index):
    """Straight collection leg; acceleration occurs before the capture gate."""
    tr, s = config.trajectory, config.simulation
    if tr.type != "straight_constant_velocity" or abs(tr.yaw_deg) > 1e-9:
        raise ValueError("外部链路当前支持直线、名义偏航0度的航线。")
    if s.navigation_rate_hz != 100:
        raise ValueError("外部链路导航输出固定100 Hz，请将导航频率设为100。")
    if not (0.5 <= tr.speed_mps <= 12 and 2 <= s.acquisition_duration_s <= 3600):
        raise ValueError("外部链路速度需为0.5–12 m/s，采集时长需为2–3600 s。")
    baseline = pass_index * np.asarray(config.nominal_baseline_sar_m)
    lever = np.asarray(config.antenna.phase_center_frd_m)
    start = -(tr.speed_mps*s.acquisition_duration_s/2 if tr.center_aperture else 0) + baseline[0]-lever[0]
    altitude = tr.apc_height_m + baseline[2] + lever[2]
    if not 5 <= altitude <= 1000:
        raise ValueError("外部链路质心高度需为5–1000 m。")
    return {"altitude_m": float(altitude), "start_x_m": float(start),
            "reposition_x_m": float(start-max(47.6, tr.speed_mps**2/2+10)),
            "center_y_m": float(baseline[1]-lever[1]),
            "duration_s": s.acquisition_duration_s, "speed_mps": tr.speed_mps}


def read_pose(path):
    rows = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8", ndmin=1)
    time = rows["time_s"]
    values = np.column_stack([rows[k] for k in (*FIELDS, *Q_FIELDS)])
    if len(time) < 2 or not np.isfinite(values).all() or not np.isfinite(time).all() or np.any(np.diff(time) <= 0):
        raise ValueError(f"Invalid external pose: {path}")
    q = values[:, 6:]
    if np.max(np.abs(np.linalg.norm(q, axis=1)-1)) > 1e-5:
        raise ValueError("External attitude quaternion is not normalized")
    return time, values[:, :3], values[:, 3:6], q


def interpolate_pose(packet, time):
    """Linear position/velocity, shortest-path SO(3) SLERP; never extrapolate."""
    t, p, v, q = packet
    if time[0] < t[0]-1e-9 or time[-1] > t[-1]+1e-9:
        raise ValueError("Requested pose extends outside native truth")
    clipped = np.clip(time, t[0], t[-1])
    rotation = Slerp(t, Rotation.from_quat(q[:, [1, 2, 3, 0]]))(clipped)
    return (np.column_stack([np.interp(clipped, t, p[:, k]) for k in range(3)]),
            np.column_stack([np.interp(clipped, t, v[:, k]) for k in range(3)]),
            rotation.as_quat()[:, [3, 0, 1, 2]])


def convert_pass(config, directory, output, name, pass_index):
    """Apply the configured lever and shared navigation model to verified native data."""
    alignment = json.loads((directory / "alignment.json").read_text(encoding="utf-8"))
    pair = json.loads((directory / "pair_validation.json").read_text(encoding="utf-8"))
    if pair.get("status") != "shared_simulation_clock_paired":
        raise ValueError("External clock pairing did not pass")
    native = read_pose(directory / alignment["files"]["truth"])
    time, nav_p, nav_v, nav_q = read_pose(directory / "navigation_100hz.csv")
    if abs(time[0]) > 1e-8 or not np.allclose(np.diff(time), .01, atol=1e-7, rtol=0):
        raise ValueError("External navigation is not a zero-origin 100 Hz grid")
    p, v, q = interpolate_pose(native, time)
    rotation = Rotation.from_quat(q[:, [1, 2, 3, 0]])
    omega = (rotation[:-1].inv()*rotation[1:]).as_rotvec()/np.diff(time)[:, None]
    omega = np.vstack((omega, omega[-1]))
    sampled = {"time_s": time, "position_N_m": polar_to_sar(p),
               "velocity_N_mps": polar_to_sar(v), "q_NB_wxyz": q,
               "acceleration_N_mps2": polar_to_sar(np.gradient(v, time, axis=0, edge_order=2)),
               "omega_B_radps": omega}
    seed = config.pass_seeds[pass_index]
    shared, truth = navigation_pose(config, navigation_estimate(config, time, p, seed), sampled, seed)
    flight = dict(truth)
    flight_rotation = Rotation.from_quat(nav_q[:, [1, 2, 3, 0]])
    flight.update(position_sar_m=nav_p, position_N_m=polar_to_sar(nav_p), velocity_sar_mps=nav_v,
                  velocity_N_mps=polar_to_sar(nav_v), q_NB_wxyz=nav_q, rotation_NB=flight_rotation.as_matrix(),
                  euler_NB_rpy_rad=flight_rotation.as_euler("xyz"),
                  euler_NB_rpy_deg=flight_rotation.as_euler("xyz", degrees=True),
                  position_error_sar_m=nav_p-p, velocity_error_sar_mps=nav_v-v,
                  attitude_error_rotvec_B_rad=(rotation.inv()*flight_rotation).as_rotvec(),
                  label="native_flight_controller_navigation_resampled_100hz",
                  estimator_model="External flight controller native estimator; configured shared errors not applied",
                  apc_position_sar_m=apc_from_pose(nav_p, nav_q, config.antenna.phase_center_frd_m))
    selected = shared if config.runtime.external_navigation_source == "shared_model" else flight
    for packet in (shared, flight, truth):
        packet["derivative_source"] = "Acceleration/body rate derived from native pose on the 100 Hz grid"
    _navigation_csv(selected, output / f"navigation_{name}.csv")
    _navigation_csv(truth, output / f"navigation_truth_{name}.csv")
    _navigation_csv(flight, output / f"flight_controller_navigation_{name}.csv")
    _navigation_csv(shared, output / f"shared_navigation_{name}.csv")
    rate = 300 if config.runtime.backend == "sitl" else 1000
    grid = np.arange(round(time[-1]*rate)+1)/rate
    gp, gv, gq = interpolate_pose(native, grid)
    def series(t, position, velocity, quaternion):
        return MotionSeries(t, position, velocity, quaternion,
                            apc_from_pose(position, quaternion, config.antenna.phase_center_frd_m))
    segment = MotionSegment(series(grid, gp, gv, gq),
                            series(time, selected["position_sar_m"], selected["velocity_sar_mps"], selected["q_NB_wxyz"]),
                            config.runtime.backend, rate, 100, "exact_common_grid", selected["estimator_model"])
    provenance = {"base_seed": config.base_seed, "pass_seed": seed, "native_alignment": alignment,
                  "native_aligned_directory": os.path.relpath(directory, output).replace("\\", "/"),
                  "truth_export_sampling": "native truth interpolated to uniform grid; original files retained",
                  "external_physics_random_seed": "not controlled by shared-model seeds",
                  "limitations": LIMITATIONS, "config": config.model_dump(mode="json")}
    export_motion_segment(segment, output, name, provenance=provenance)
    reference = build_reference(config, time, pass_index*np.asarray(config.nominal_baseline_sar_m))
    result = {"navigation": selected, "navigation_truth": truth, "flight_controller_navigation": flight,
              "shared_navigation": shared, "reference": reference,
              "truth_sar": {"apc_position_sar_m": segment.truth.apc_position_sar_m},
              "metadata": {"physical_backend": config.runtime.backend, "pass_seed": seed, "alignment": alignment}}
    distance = float(p[-1, 0]-p[0, 0])
    speed = np.linalg.norm(v, axis=1)
    coverage = {"forward_distance_m": distance, "target_length_m": config.trajectory.route_length_m,
                "assessment_mode": "target_length_coverage" if config.simulation.duration_mode == "route_length" else "fixed_duration_distance_report_only",
                "speed_min_mps": float(speed.min()), "speed_max_mps": float(speed.max()),
                "duration_s": float(time[-1]), "speed_hard_gate_applied": False,
                "passed": config.simulation.duration_mode != "route_length" or distance >= config.trajectory.route_length_m,
                "crop_window": None}
    # Crop only a contiguous forward leg, on the common 100 Hz truth/navigation grid.
    if np.all(np.diff(p[:, 0]) > 0) and distance >= config.trajectory.route_length_m:
        first = int(np.searchsorted(p[:, 0], (p[0, 0]+p[-1, 0]-config.trajectory.route_length_m)/2))
        last = int(np.searchsorted(p[:, 0], p[first, 0]+config.trajectory.route_length_m))
        if last < len(time):
            coverage["crop_window"] = {"start_time_s": float(time[first]), "end_time_s": float(time[last]),
                                       "forward_distance_m": float(p[last, 0]-p[first, 0])}
    result["metadata"]["route_coverage"] = coverage
    write_json(output / f"route_coverage_{name}.json", coverage)
    savemat(output / f"uav_motion_{name}.mat", {"pass_result": _mat_safe(result)},
            long_field_names=True, do_compression=True, oned_as="column")
    return result


def run_external_job(root, config, passes, mode):
    """Reserve shared external ports for one GUI task at a time."""
    if os.name != "nt":
        raise RuntimeError("本入口需要Windows WSL2；Linux下请使用deployment原生入口。")
    import msvcrt

    with (PROJECT / "main" / "gui_outputs" / "external_runtime.lock").open("a+b") as lock:
        if lock.tell() == 0:
            lock.write(b"0")
            lock.flush()
        lock.seek(0)
        try:
            msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
        except OSError as error:
            raise RuntimeError("已有上位机任务占用外部仿真平台，请待其完成或停止后重试。") from error
        try:
            return _run_external_job(root, config, passes, mode)
        finally:
            lock.seek(0)
            msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)


def _run_external_job(root, config, passes, mode):
    """Run one selected external chain serially, then reuse GUI pose/report views."""
    if os.name != "nt":
        raise RuntimeError("本入口需要Windows WSL2；Linux下请使用deployment原生入口。")
    routes = [route_spec(config, i) for i in range(passes)]
    output = root / "artifacts"
    output.mkdir()
    backend = config.runtime.backend
    wsl = ["wsl", "-d", config.runtime.wsl_distribution, "-u", "root", "--cd", str(PROJECT), "--exec"]
    flags = subprocess.CREATE_NO_WINDOW
    owner = root / "external_owner.heartbeat"
    owner.touch()
    stop = threading.Event()
    def heartbeat():
        while not stop.wait(2):
            owner.touch()
    thread = threading.Thread(target=heartbeat, daemon=True)
    thread.start()
    commands = []
    def execute(label, command, timeout=120, flight=False, index=0):
        print(f"[外部链路] {label}", flush=True)
        commands.append({"step": label, "command": command})
        log = root / (label + ".log")
        if not flight:
            with log.open("w", encoding="utf-8") as stream:
                completed = subprocess.run(command, cwd=PROJECT, stdout=stream, stderr=subprocess.STDOUT,
                                           timeout=timeout, creationflags=flags)
            if completed.returncode:
                raise RuntimeError(f"{label}失败，请查看 {log}；确认所选WSL及外部平台已部署。")
            return
        with log.open("w", encoding="utf-8") as stream:
            process = subprocess.Popen(command, cwd=PROJECT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, encoding="utf-8", errors="replace", creationflags=flags)
            with process.stdout:
                for line in process.stdout:
                    stream.write(line)
                    stream.flush()
                    if line.startswith("UAVSIM_EXTERNAL_PROGRESS "):
                        item = json.loads(line.split(" ", 1)[1])
                        phase = item["phase"]
                        fraction = {"takeoff": .04, "reposition": .10, "accelerate": .16, "complete": .85}.get(phase, .04)
                        if phase == "capture":
                            fraction = .2 + .6*min(1, item["elapsed_s"]/item["duration_s"])
                        label = {"takeoff": "爬升", "reposition": "航前定位",
                                 "accelerate": "加速并等待采集条件", "capture": "水平采集",
                                 "complete": "降落并结束原生日志"}.get(phase, phase)
                        report_progress(pass_index=index, fraction=fraction, stage=f"航次{index+1} · {label}")
                    else:
                        print(line.rstrip(), flush=True)
            code = process.wait()
        if code:
            raise RuntimeError(f"{label}失败，返回{code}；详见 {log}")
    try:
        print("[外部链路能力说明] " + LIMITATIONS, flush=True)
        report_progress(percent=1, stage=f"检查 {backend} 的WSL环境")
        python = "/opt/uav_motion/sitl/4.7.2/venv/bin/python" if backend == "sitl" else "/opt/uav_motion/px4_venv/bin/python"
        execute("preflight", wsl+["test", "-x", python], 30)
        result, stages = {}, []
        for i, route in enumerate(routes):
            name = f"pass{i+1}"
            route_file = root / f"external_route_{name}.json"
            write_json(route_file, route)
            native_base = output / name
            linux_output = wsl_path(native_base)
            if backend == "sitl":
                command = [python, "-u", "deployment/sitl/validate_flight.py", "--binary",
                           "/opt/uav_motion/sitl/4.7.2/bin/arducopter", "--defaults",
                           "/opt/uav_motion/sitl/4.7.2/copter.parm", "--output", linux_output,
                           "--route", "--aligned-route", "--truth-rate-hz", "300"]
            else:
                command = ["bash", "deployment/xtdrone/run_validation.sh", "--output", linux_output,
                           "--route", "--aligned-route", "--truth-rate-hz", "1000"]
            command += ["--route-config", wsl_path(route_file)]
            execute(name+"_flight", wsl+["python3", "deployment/external_supervisor.py",
                    "--cancel-file", wsl_path(root/"cancel.request"), "--owner-file", wsl_path(owner),
                    "--timeout", str(600+route["duration_s"]*3), "--", *command], flight=True, index=i)
            runs = [p for p in native_base.iterdir() if p.is_dir()]
            if len(runs) != 1:
                raise RuntimeError("本任务外部运行目录不唯一")
            native = runs[0]
            validation = json.loads((native/"validation.json").read_text(encoding="utf-8"))
            if not validation.get("passed"):
                raise RuntimeError("外部飞行验收未通过，请查看原生 validation.json")
            logs = list(native.glob("logs/*.BIN" if backend == "sitl" else "log/**/*.ulg"))
            if len(logs) != 1:
                raise RuntimeError("未找到唯一原生飞行日志")
            aligned = native / "alignment"
            aligned.mkdir()
            script = "extract_sitl_dataflash.py" if backend == "sitl" else "extract_px4_ulog.py"
            execute(name+"_extract", wsl+[python, "deployment/"+script,
                    "--log" if backend == "sitl" else "--ulog", wsl_path(logs[0]), "--output", wsl_path(aligned)])
            execute(name+"_align", [sys.executable, f"deployment/align_{backend}_sim_clock.py",
                    "--directory", str(native), "--output", str(aligned)])
            execute(name+"_pair", [sys.executable, "deployment/validate_clock_aligned.py", "--directory",
                    str(aligned), "--truth", "truth_native.csv" if backend == "sitl" else "truth_1000hz.csv"])
            result[name] = convert_pass(config, aligned, output, name, i)
            report_progress(pass_index=i, fraction=1)
            stages.append({"name": f"{name} {backend}飞行及严格对时", "status": "passed",
                           "detail": "真实启动外部平台；保留原生真值/日志；导航对齐100 Hz；杆臂已应用。"})
            coverage = result[name]["metadata"]["route_coverage"]
            coverage_mode = (f"目标长度{coverage['target_length_m']:g} m；" if config.simulation.duration_mode == "route_length"
                             else "固定时长模式，仅报告距离，不验收目标长度；")
            stages.append({"name": f"{name} 水平航段检查", "status": "passed" if coverage["passed"] else "failed",
                           "detail": coverage_mode+f"实测前进{coverage['forward_distance_m']:.3f} m；速度{coverage['speed_min_mps']:.3f}–{coverage['speed_max_mps']:.3f} m/s。裁剪窗与完整统计见route_coverage_{name}.json。"})
        stages.append({"name": "外部链路参数适用范围", "status": "passed", "detail": LIMITATIONS})
        if mode == "acceptance":
            stages.append({"name": "跨平台对照、OU统计、积分收敛及SAR端到端", "status": "skipped",
                           "detail": "本入口仅验收所选外部链路；未执行项不能视为完整科研验收通过。"})
        report_progress(percent=90, stage="生成外部链路三维航迹和导航对照")
        plot_trajectories(result, output)
        plot_pose_comparisons(result, output)
        plot_navigation(result, output)
        summary = {"data_directory": str(output), "stages": stages,
                   "message": f"{backend} · 导航来源 {config.runtime.external_navigation_source}。"+LIMITATIONS,
                   "supported_figures": [file for file in ("trajectory_3d.png", "navigation_position_comparison.png",
                                         "navigation_velocity_comparison.png", "navigation_attitude_comparison.png",
                                         "navigation_pose_errors.png", "imu_gnss_diagnostics.png") if (output/file).exists()],
                   "metrics": {"navigation_samples": len(result["pass1"]["navigation"]["time_s"]),
                               "truth_rate_hz": 300 if backend == "sitl" else 1000}}
        write_json(root/"acceptance.json", summary)
        write_json(output/"metadata.json", {"backend": backend, "navigation_source": config.runtime.external_navigation_source,
                                           "limitations": LIMITATIONS, "commands": commands})
        create_report(root, summary, result)
        report_progress(percent=100, stage="外部链路结果已生成")
        failed = any(stage["status"] == "failed" for stage in stages)
        return {"type": "result", "mode": mode, "status": "failed" if failed else "partial" if mode == "acceptance" else "simulated",
                "report": str(root/"index.html"), "data_directory": str(output),
                "job_directory": str(root), "returncode": int(failed)}
    finally:
        stop.set()
        thread.join(timeout=3)
        write_json(root/"external_commands.json", {"commands": commands, "limitations": LIMITATIONS})
