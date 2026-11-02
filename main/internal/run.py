"""完整人工验收入口；在 main/outputs 下保留每次运行，不覆盖历史成果。"""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import traceback
import webbrowser
import xml.etree.ElementTree as ET
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime
from pathlib import Path
from time import perf_counter
from zoneinfo import ZoneInfo

from review_plots import plot_geometry_and_actuators, plot_pose_comparisons
from review_report import acceptance_status, create_report

from uavsim.cli import main as run_core
from uavsim.config.loader import load_config, save_config
from uavsim.config.validator import Config
from uavsim.sar_adapter.matlab_export import load_exported_pass, write_json

PROJECT = Path(__file__).resolve().parents[2]
MAIN = PROJECT / "main"


class Tee:
    """同时写终端和日志，避免后台验收时只得到无上下文结果。"""

    def __init__(self, console, log):
        self.console = console
        self.log = log

    def write(self, message: str) -> int:
        self.console.write(message)
        self.log.write(message)
        self.flush()
        return len(message)

    def flush(self) -> None:
        self.console.flush()
        self.log.flush()


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _matlab_literal(path: Path) -> str:
    """MATLAB 单引号字符串转义，路径只作为数据，不进入 shell。"""
    return "'" + path.resolve().as_posix().replace("'", "''") + "'"


def run_matlab(
    data_dir: Path, matlab_mode: str, matlab_exe: str | None, sar_project: Path, log_dir: Path
) -> dict:
    if matlab_mode == "skip":
        return {
            "name": "MATLAB 导航与 SAR 接口",
            "status": "skipped",
            "detail": "用户显式选择跳过，不能视为完整接口验收通过。",
        }
    executable = matlab_exe or shutil.which("matlab")
    if not executable:
        state = "failed" if matlab_mode == "required" else "skipped"
        return {
            "name": "MATLAB 导航与 SAR 接口",
            "status": state,
            "detail": "未找到 MATLAB；可用 --matlab-exe 指定可执行文件。",
        }
    has_sar = (sar_project / "+lband" / "simulate_echo.m").is_file()
    expression = f"addpath({_matlab_literal(PROJECT / 'matlab')}); disp(version); run_matlab_validation({_matlab_literal(data_dir)}"
    if has_sar:
        expression += "," + _matlab_literal(sar_project)
    expression += ");"
    print("[MATLAB] 读取完整两航次导航/运动数据，检查时间、APC 和回波接口。", flush=True)
    completed = subprocess.run(
        [str(executable), "-batch", expression],
        cwd=PROJECT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
        timeout=600,
    )
    (log_dir / "matlab_output.txt").write_text(completed.stdout + "\n" + completed.stderr, encoding="utf-8")
    result_path = data_dir / "matlab_validation.json"
    if completed.returncode or not result_path.exists():
        return {
            "name": "MATLAB 导航与 SAR 接口",
            "status": "failed",
            "detail": f"MATLAB 返回 {completed.returncode}，详见 logs/matlab_output.txt。",
        }
    result = _read_json(result_path)
    if not result.get("passed"):
        return {"name": "MATLAB 导航与 SAR 接口", "status": "failed", "detail": "MATLAB 数值检查未通过。"}
    echo = result.get("echo", {})
    if not echo.get("passed"):
        return {
            "name": "MATLAB 导航与 SAR 接口",
            "status": "skipped",
            "detail": "导航/几何检查通过；未找到原 L 波段项目，完整回波读取未执行。",
        }
    return {
        "name": "MATLAB 导航与 SAR 接口",
        "status": "passed",
        "detail": f"两航次导航与几何通过；{echo['range_samples']}×{echo['pulses']} 单点回波；斜距核对差 {echo['range_max_error_m']:.3g} m。",
    }


def collect_checks(data_dir: Path) -> tuple[list[dict], dict]:
    """读取本次原始验证结果，缺失检查记为未执行，不补造通过标志。"""
    stages = []
    metrics = {}
    for filename, name in [
        ("validation.json", "刚体运动与时间同步"),
        ("ou_validation.json", "长序列 OU 随机统计"),
        ("convergence_validation.json", "1600 / 3200 / 6400 Hz 数值收敛"),
    ]:
        path = data_dir / filename
        if not path.exists():
            stages.append({"name": name, "status": "skipped", "detail": "本次未生成检查结果。"})
            continue
        result = _read_json(path)
        detail = "数值门槛通过。" if result.get("passed") else "数值门槛未通过，请查看对应 JSON。"
        if filename == "validation.json":
            first = result.get("pass1", {})
            metrics.update(
                sar_samples=first.get("sar_samples", "—"),
                navigation_samples=first.get("navigation_samples", "—"),
            )
            detail = f"每航次 SAR {metrics['sar_samples']} 点、导航 {metrics['navigation_samples']} 点；直接整数步同步。"
        elif filename == "ou_validation.json":
            detail = f"{result['duration_s']:g} s / {result['samples']} 点；方差 {result['variance']:.5f}，ACF 最大差 {result['acf_max_error_0_to_10s']:.5f}。"
        else:
            rms = result["comparisons_to_6400_hz"]["3200"]["slant_range_rms_m"]
            metrics["range_rms_display"] = f"{rms * 1000:.5f} mm"
            detail = f"3200 对 6400 Hz 斜距差 RMS {rms * 1000:.6f} mm，门槛 {result['limit_slant_range_rms_m'] * 1000:g} mm。"
        stages.append(
            {"name": name, "status": "passed" if result.get("passed") else "failed", "detail": detail}
        )
    test_path = data_dir / "pytest_results.xml"
    if test_path.exists():
        suites = ET.parse(test_path).getroot()
        tests = suites.findall("testsuite") if suites.tag == "testsuites" else [suites]
        counts = {
            key: sum(int(s.attrib.get(key, 0)) for s in tests)
            for key in ["tests", "failures", "errors", "skipped"]
        }
        passed = counts["tests"] - counts["failures"] - counts["errors"] - counts["skipped"]
        metrics["tests_display"] = f"{passed} / {counts['tests']} passed"
        status = (
            "failed"
            if counts["failures"] + counts["errors"]
            else "skipped"
            if counts["skipped"]
            else "passed"
        )
        stages.append(
            {
                "name": "自动化测试套件",
                "status": status,
                "detail": f"{passed} 通过，{counts['failures']} 失败，{counts['errors']} 错误，{counts['skipped']} 跳过。",
            }
        )
    else:
        stages.append({"name": "自动化测试套件", "status": "skipped", "detail": "未生成本次测试日志。"})
    return stages, metrics


def main(argv: list[str] | None = None) -> int:
    for stream in [sys.stdout, sys.stderr]:
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="Full manual acceptance workflow and offline visualization")
    parser.add_argument("--config", type=Path, default=MAIN / "config.yaml")
    parser.add_argument("--workers", type=int, default=0, help="0 = automatic resource planning")
    parser.add_argument("--matlab", choices=["auto", "required", "skip"], default="auto")
    parser.add_argument("--matlab-exe", default=None)
    parser.add_argument("--sar-project", type=Path, default=PROJECT.parent / "L波段仿真")
    parser.add_argument("--no-open", action="store_true", help="Generate report without launching a browser")
    parser.add_argument("--output-root", type=Path, default=MAIN / "outputs")
    args = parser.parse_args(argv)
    stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
    root = args.output_root.resolve() / f"acceptance_{stamp}"
    root.mkdir(parents=True, exist_ok=False)
    logs = root / "logs"
    logs.mkdir()
    started = perf_counter()
    summary = {
        "started_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
        "stages": [],
        "metrics": {},
        "data_directory": None,
        "resources": {},
        "message": "请确认参数和坐标后，对本次实际输出进行人工验收。",
    }
    digest = hashlib.sha256()
    for path in sorted((MAIN / "internal").glob("*.py")):
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    summary["acceptance_entry_sha256"] = digest.hexdigest()
    result = None
    with (root / "run.log").open("w", encoding="utf-8") as log:
        with redirect_stdout(Tee(sys.stdout, log)), redirect_stderr(Tee(sys.stderr, log)):
            try:
                data = load_config(args.config).model_dump()
                data["output"].update(save_mat=True, save_csv=True, save_plots=True)
                data["runtime"]["max_workers"] = args.workers
                config = Config.model_validate(data)
                if len(config.pass_seeds) < 2:
                    raise ValueError("人工验收需要至少两个独立 pass_seeds。")
                snapshot = root / "resolved_config.yaml"
                save_config(config, snapshot)
                print("老大，开始完整人工验收：双航次、全量测试、长 OU、全时长积分收敛。", flush=True)
                print(f"输出目录：{root}", flush=True)
                core_code = run_core(
                    ["validate", "--config", str(snapshot), "--passes", "2", "--output", str(root)]
                )
                matches = list(root.glob("validate_*"))
                if len(matches) != 1:
                    raise RuntimeError("未找到本次唯一的完整输出目录，请查看 run.log。")
                data_dir = matches[0]
                summary["data_directory"] = str(data_dir)
                summary["stages"], summary["metrics"] = collect_checks(data_dir)
                if core_code:
                    summary["stages"].append(
                        {"name": "Python 完整流程", "status": "failed", "detail": f"返回码 {core_code}。"}
                    )
                metadata = _read_json(data_dir / "metadata.json")
                summary["resources"] = metadata["resource_plan"]
                summary["metrics"]["simulation_display"] = f"{metadata['elapsed_s']:.2f} s"
                files = [data_dir / f"uav_motion_pass{index}.mat" for index in [1, 2]]
                if all(path.exists() for path in files):
                    result = {
                        f"pass{index + 1}": load_exported_pass(path) for index, path in enumerate(files)
                    }
                    print("[可视化] 绘制导航真值/估计对照、天线几何和执行器检查。", flush=True)
                    plot_pose_comparisons(result, data_dir)
                    plot_geometry_and_actuators(result, data_dir, config)
                    figure_count = len(list(data_dir.glob("*.png")))
                    summary["stages"].append(
                        {
                            "name": "导航位姿与关键产出可视化",
                            "status": "passed" if figure_count == 12 else "skipped",
                            "detail": f"真值/估计逐帧对照；已生成 {figure_count}/12 张结果图；原始 MAT/CSV 不改动。",
                        }
                    )
                else:
                    summary["stages"].append(
                        {
                            "name": "导航位姿与关键产出可视化",
                            "status": "failed",
                            "detail": "完整 MAT 数据缺失。",
                        }
                    )
                if not core_code:
                    summary["stages"].append(
                        run_matlab(data_dir, args.matlab, args.matlab_exe, args.sar_project, logs)
                    )
                else:
                    summary["stages"].append(
                        {
                            "name": "MATLAB 导航与 SAR 接口",
                            "status": "skipped",
                            "detail": "Python 数值检查失败，停止后续验收。",
                        }
                    )
            except Exception as error:
                traceback.print_exc()
                summary["stages"].append({"name": "流程执行", "status": "failed", "detail": str(error)})
                summary["message"] = "流程存在未通过或未执行项。已保留本次数据与日志，请按失败信息检查。"
            summary["elapsed_s"] = perf_counter() - started
            summary["status"] = acceptance_status(summary["stages"])
            summary["scope"] = (
                "Python full validation + navigation review + MATLAB interface; no SITL/Simscape or real-flight calibration"
            )
            write_json(root / "acceptance.json", summary)
            report = create_report(root, summary, result)
            (MAIN / "outputs" / "latest_report.txt").write_text(str(report), encoding="utf-8")
            print(f"人工验收状态：{summary['status']}；总用时 {summary['elapsed_s']:.1f} s", flush=True)
            print(f"验收页面：{report}", flush=True)
    if not args.no_open:
        try:
            webbrowser.open(report.as_uri())
        except OSError:
            print(f"请双击打开验收页面：{report}", flush=True)
    return {"passed": 0, "partial": 2, "failed": 1}[summary["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
