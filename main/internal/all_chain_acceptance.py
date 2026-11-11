"""All distinct engines, complete applicable checks, fresh results and one report.

Auto aliases CPU and is not repeated. External controllers do not expose the
Python RK4 integrator; their applicable checks use native logs and clock evidence.
Cross-engine differences are descriptive, never an invented equivalence threshold.
"""

import hashlib
import json
import shutil
import sys
import threading
import traceback
from itertools import combinations
from pathlib import Path
from time import perf_counter

import numpy as np
from control_jobs import EVENT_PREFIX, launch_job, parse_event, stop_job
from review_plots import plot_geometry_and_actuators, plot_pose_comparisons
from review_report import acceptance_status, create_report
from run import check_figures, collect_checks, run_matlab
from scipy.spatial.transform import Rotation, Slerp

from uavsim.cli import main as core_main
from uavsim.config.loader import save_config
from uavsim.config.validator import Config
from uavsim.motion.export import load_motion_segment
from uavsim.runtime.progress import report_progress
from uavsim.sar_adapter.matlab_export import load_exported_pass, write_json

PROJECT = Path(__file__).resolve().parents[2]
CHAINS = ("rotorpy", "matlab", "cpu", "sitl", "px4")
INTERNAL_CHECKS = ("刚体运动与时间同步", "长序列 OU 随机统计", "1600 / 3200 / 6400 Hz 数值收敛",
                   "自动化测试套件", "导航位姿与关键产出可视化", "MATLAB 独立导航/几何接口")


def required_check_names(backend):
    if backend in ("sitl", "px4"):
        return tuple(f"pass{i} {label}" for i in (1, 2)
                     for label in (backend+"飞行及严格对时", "水平航段检查")) + ("外部链路参数适用范围",)
    return INTERNAL_CHECKS


def payload(root, mode, summary):
    status = acceptance_status(summary["stages"])
    summary["status"] = status
    write_json(root / "acceptance.json", summary)
    return {"type": "result", "mode": mode, "status": status,
            "report": str(root / "index.html"), "data_directory": summary.get("data_directory") or str(root),
            "job_directory": str(root), "returncode": {"passed": 0, "failed": 1, "partial": 2}[status]}


def run_internal_scientific(root, config, matlab_exe=""):
    """Full-length two-pass scientific workflow and independent MATLAB interface."""
    if config.runtime.backend not in ("rotorpy", "matlab", "cpu"):
        raise ValueError("Scientific subtask requires an internal physical engine")
    summary = {"stages": [], "metrics": {}, "data_directory": None,
               "message": f"{config.runtime.backend} 双航次、全量测试、长OU统计、全时长收敛、MATLAB独立几何接口；不含SAR回波/成像。"}
    result = None
    try:
        code = core_main(["validate", "--config", str(root / "input_config.yaml"),
                          "--passes", "2", "--output", str(root / "artifacts")])
        directories = list((root / "artifacts").glob("validate_*"))
        if len(directories) != 1:
            raise RuntimeError("未生成唯一完整数值验收目录，请查看本链路worker.log")
        directory = directories[0]
        summary["data_directory"] = str(directory)
        summary["stages"], summary["metrics"] = collect_checks(directory)
        if code:
            summary["stages"].append({"name": "完整数值流程", "status": "failed", "detail": f"返回码{code}"})
        result = {f"pass{i}": load_exported_pass(directory / f"uav_motion_pass{i}.mat") for i in (1, 2)}
        plot_pose_comparisons(result, directory)
        plot_geometry_and_actuators(result, directory, config)
        summary["stages"].append(check_figures(directory, config.disturbances.imu_gnss.enabled))
        executable = matlab_exe or config.runtime.matlab_executable or shutil.which("matlab")
        logs = root / "logs"
        logs.mkdir()
        report_progress(percent=96, stage="MATLAB独立检查完整双航次导航和几何接口")
        summary["stages"].append(run_matlab(directory, "required", executable, PROJECT, logs, navigation_only=True))
    except Exception as error:
        traceback.print_exc()
        summary["stages"].append({"name": "链路验收执行", "status": "failed", "detail": str(error)})
    response = payload(root, "scientific", summary)
    create_report(root, summary, result)
    return response


class ChildProgress:
    """Map nested simulation/convergence events to one monotonic chain percentage."""

    def __init__(self, scientific):
        self.weight = 40 if scientific else 80
        self.passes = [0., 0.]
        self.convergence = {1600: 0., 3200: 0., 6400: 0.}
        self.percent = 0.

    def update(self, event):
        if "percent" in event:
            value = event["percent"]
        elif event.get("phase") == "convergence":
            rate = event["rate"]
            if rate not in self.convergence:
                raise ValueError("Unexpected convergence rate")
            self.convergence[rate] = max(self.convergence[rate], event["fraction"])
            value = 60 + 30*sum(k*v for k, v in self.convergence.items())/11200
        else:
            index = event["pass_index"]
            if index not in (0, 1):
                raise ValueError("Unexpected pass index")
            self.passes[index] = max(self.passes[index], event["fraction"])
            value = self.weight*sum(self.passes)/2
        self.percent = max(self.percent, value)
        return self.percent


def chain_required_stages(backend, summary):
    """Do not promote unknown skipped stages; exclude one explicitly inapplicable bundle."""
    stages = summary.get("stages", [])
    if backend in ("sitl", "px4"):
        excluded = "跨平台对照、OU统计、积分收敛及SAR端到端"
        stages = [stage for stage in stages if not (stage["name"] == excluded and stage["status"] == "skipped")]
    if not stages:
        return [{"name": "检查结果缺失", "status": "failed", "detail": "子链路没有适用验收记录"}]
    missing = sorted(set(required_check_names(backend)) - {stage["name"] for stage in stages})
    if missing:
        stages = [*stages, {"name": "必需检查缺失", "status": "failed", "detail": "；".join(missing)}]
    return stages


def compare_segments(directories, root):
    """Report all pairwise position/velocity/SO(3) differences on the common 100 Hz grid.

    Time origins are capture starts. No fitted offsets, clock warps or truth changes.
    Native flight controllers have different disturbance/control policies, so these
    statistics must not be interpreted as matched-physics accuracy or equivalence.
    """
    if set(directories) != set(CHAINS):
        raise ValueError("跨链路对照需要本次五套链路的完整产出")
    rows = []
    for pass_name in ("pass1", "pass2"):
        segments = {name: load_motion_segment(directory, pass_name) for name, directory in directories.items()}
        end = min(segment.truth.time_s[-1] for segment in segments.values())
        time = np.arange(int(np.floor(end*100+1e-8))+1)/100
        if len(time) < 2:
            raise ValueError("共同时间窗过短")
        sampled = {}
        for name, segment in segments.items():
            truth = segment.truth
            p = np.column_stack([np.interp(time, truth.time_s, truth.position_sar_m[:, k]) for k in range(3)])
            v = np.column_stack([np.interp(time, truth.time_s, truth.velocity_sar_mps[:, k]) for k in range(3)])
            q = Slerp(truth.time_s, Rotation.from_quat(truth.q_NB_wxyz[:, [1, 2, 3, 0]]))(time)
            sampled[name] = p, v, q
        for left, right in combinations(CHAINS, 2):
            lp, lv, lq = sampled[left]
            rp, rv, rq = sampled[right]
            rows.append({"pass": pass_name, "left": left, "right": right, "samples": len(time),
                         "duration_s": float(time[-1]),
                         "position_xyz_rms_m": np.sqrt(np.mean((rp-lp)**2, axis=0)).tolist(),
                         "velocity_xyz_rms_mps": np.sqrt(np.mean((rv-lv)**2, axis=0)).tolist(),
                         "attitude_angle_rms_deg": float(np.rad2deg(np.sqrt(np.mean((lq.inv()*rq).magnitude()**2))))})
    record = {"status": "passed", "pairs": rows,
              "meaning": "Data integrity and pairwise metrics computed; no numerical equivalence threshold imposed",
              "limitations": "采集开始相对时间100 Hz公共网格；不扣除初始偏差。外部链路不接收内部A/B扰动及控制增益，这些数值不是完全同物理工况的精度排名。"}
    write_json(root / "cross_chain_metrics.json", record)
    return record


def compare_external(directories, root):
    """Reuse this job's fresh two-pass native logs; never rerun or reuse historical flights."""
    sys.path.insert(0, str(PROJECT / "deployment"))
    from compare_aligned_routes import compare

    rows = []
    for i in (1, 2):
        aligned = {}
        for name in ("sitl", "px4"):
            matches = list((directories[name] / f"pass{i}").glob("*/alignment"))
            if len(matches) != 1:
                raise ValueError(f"{name}/pass{i}本次原生对时目录不唯一")
            aligned[name] = matches[0]
        rows.append(compare(aligned["sitl"], aligned["px4"]))
    folder = root / "external_comparison"
    folder.mkdir()
    record = {"status": "passed", "detail": "本次双航次原生飞控同工况对照已计算；差异不设等价门槛。",
              "comparison": rows[0], "passes": rows, "steps": []}
    write_json(folder / "result.json", record)
    return record


def run_all_chains(root, config, matlab_exe=""):
    """Serial independent children, failure isolation, cancellation and aggregate report."""
    started = perf_counter()
    chains = root / "chains"
    chains.mkdir()
    summary = {"stages": [], "chain_acceptance": [], "metrics": {}, "data_directory": None,
               "message": "全链路验收：五套独立实现各跑双航次；auto等同cpu不重复。内部核心执行全量科学验收；外部飞控执行真实飞行和原生日志严格对时。SAR回波/成像端到端不在本次范围。"}
    digest = hashlib.sha256()
    files = list((PROJECT / "main" / "internal").glob("*.py"))
    files += list((PROJECT / "deployment").rglob("*.py")) + list((PROJECT / "deployment").rglob("*.sh"))
    for path in sorted(files):
        digest.update(path.relative_to(PROJECT).as_posix().encode("utf-8"))
        digest.update(path.read_bytes())
    summary["entry_sha256"] = digest.hexdigest()
    summary["chain_order"] = list(CHAINS)
    active, stop = {}, threading.Event()
    (root / "external_owner.heartbeat").touch()
    def monitor_cancel():
        while not stop.wait(.3):
            (root / "external_owner.heartbeat").touch()
            if (root / "cancel.request").exists():
                process = active.get("process")
                if process is not None and process.poll() is None:
                    stop_job(process)
                return
    monitor = threading.Thread(target=monitor_cancel, daemon=True)
    monitor.start()
    directories = {}
    try:
        for index, backend in enumerate(CHAINS):
            if (root / "cancel.request").exists():
                raise RuntimeError("全链路验收已取消，不再启动后续链路")
            child = chains / backend
            child.mkdir()
            data = config.model_dump()
            data["runtime"]["backend"] = backend
            if matlab_exe:
                data["runtime"]["matlab_executable"] = matlab_exe
            child_config = Config.model_validate(data)
            save_config(child_config, child / "input_config.yaml")
            scientific = backend in ("rotorpy", "matlab", "cpu")
            progress = ChildProgress(scientific)
            item = {"backend": backend, "status": "failed", "report": None, "stages": []}
            report_progress(percent=index*17, stage=f"全链路 {index+1}/5 · {backend}")
            print(f"[全链路验收] 开始 {backend}，配置快照：{child/'input_config.yaml'}", flush=True)
            try:
                process = launch_job(PROJECT, child, "scientific" if scientific else "acceptance", 2,
                                     "required", matlab_exe or config.runtime.matlab_executable or "")
                active["process"] = process
                with (child / "worker.log").open("w", encoding="utf-8") as stream, process.stdout:
                    for line in process.stdout:
                        stream.write(line)
                        stream.flush()
                        event = parse_event(line) if line.startswith(EVENT_PREFIX) else None
                        if event and event["type"] == "progress":
                            percent = progress.update(event)
                            report_progress(percent=index*17 + .17*min(percent, 100),
                                            stage=f"全链路 {index+1}/5 · {backend} · {event.get('stage', '双航次计算')}")
                        elif event and event["type"] == "error":
                            print(f"[{backend}] {event['message']}", flush=True)
                        elif not event:
                            print(f"[{backend}] {line.rstrip()}", flush=True)
                code = process.wait()
                active.pop("process", None)
                if (root / "cancel.request").exists():
                    raise RuntimeError("用户取消")
                child_summary = json.loads((child / "acceptance.json").read_text(encoding="utf-8"))
                item["stages"] = chain_required_stages(backend, child_summary)
                item["status"] = acceptance_status(item["stages"])
                if code not in (0, 2):
                    item["status"] = "failed"
                item["report"] = (child / "index.html").relative_to(root).as_posix()
                directory = child_summary.get("data_directory")
                if directory and all((Path(directory)/f"motion_segment_pass{i}.json").exists() for i in (1, 2)):
                    directories[backend] = Path(directory)
                    if summary["data_directory"] is None:
                        summary["data_directory"] = directory
            except Exception as error:
                traceback.print_exc()
                item["stages"].append({"name": "子链路执行", "status": "failed", "detail": str(error)})
            summary["chain_acceptance"].append(item)
            summary["stages"].append({"name": f"{backend} 完整适用验收", "status": item["status"],
                                      "detail": "；".join(stage["name"]+":"+stage["status"] for stage in item["stages"])})
            write_json(root / "all_chain_progress.json", summary)
        if (root / "cancel.request").exists():
            raise RuntimeError("全链路验收已取消")
        for name, action in (("五链路双航次指标与数据完整性", compare_segments),
                             ("SITL / PX4 双航次原生时钟同工况对照", compare_external)):
            report_progress(percent=90 if action == compare_segments else 95, stage=name)
            try:
                record = action(directories, root)
                if action == compare_segments:
                    summary["cross_chain_metrics"] = record
                else:
                    summary["external_comparison"] = record
                summary["stages"].append({"name": name, "status": "passed", "detail": "本次新产出计算完成；差异如实报告，不据此宣称与实飞等价。"})
            except Exception as error:
                traceback.print_exc()
                summary["stages"].append({"name": name, "status": "failed", "detail": str(error)})
    except Exception as error:
        summary["stages"].append({"name": "全链路编排", "status": "failed", "detail": str(error)})
    finally:
        stop.set()
        monitor.join(timeout=16)
        process = active.get("process")
        if process is not None and process.poll() is None:
            stop_job(process)
    summary["elapsed_s"] = perf_counter()-started
    response = payload(root, "all_acceptance", summary)
    display = None
    if summary["data_directory"]:
        directory = Path(summary["data_directory"])
        files = [directory / f"uav_motion_pass{i}.mat" for i in (1, 2)]
        if all(path.exists() for path in files):
            display = {f"pass{i+1}": load_exported_pass(path) for i, path in enumerate(files)}
    create_report(root, summary, display)
    report_progress(percent=100, stage="全链路验收结束 · "+response["status"])
    return response
