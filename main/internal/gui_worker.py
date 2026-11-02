"""桌面上位机计算子进程；调用同一仿真与验收接口，不另建数值模型。"""

import argparse
import json
import os
import traceback
from pathlib import Path

from control_jobs import EVENT_PREFIX
from review_plots import plot_geometry_and_actuators, plot_pose_comparisons
from review_report import create_report

from uavsim.cli import main as core_main
from uavsim.config.loader import load_config
from uavsim.runtime.progress import report_progress
from uavsim.sar_adapter.matlab_export import load_exported_pass, write_json


def event(value: dict) -> None:
    print(EVENT_PREFIX + json.dumps(value, ensure_ascii=False), flush=True)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--job-dir", required=True, type=Path)
    parser.add_argument("--mode", choices=["simulate", "acceptance"], required=True)
    parser.add_argument("--passes", type=int, choices=[1, 2], default=2)
    parser.add_argument("--matlab", choices=["auto", "required", "skip"], default="auto")
    parser.add_argument("--matlab-exe", default="")
    parser.add_argument("--sar-project", default="")
    args = parser.parse_args(argv)
    root = args.job_dir.resolve()
    config = load_config(root / "input_config.yaml")
    try:
        if args.mode == "acceptance":
            from run import main as acceptance_main

            options = [
                "--config",
                str(root / "input_config.yaml"),
                "--output-root",
                str(root / "artifacts"),
                "--workers",
                str(config.runtime.max_workers),
                "--matlab",
                args.matlab,
                "--no-open",
            ]
            if args.matlab_exe:
                options += ["--matlab-exe", args.matlab_exe]
            if args.sar_project:
                options += ["--sar-project", args.sar_project]
            code = acceptance_main(options)
            matches = list((root / "artifacts").glob("acceptance_*"))
            if len(matches) != 1:
                raise RuntimeError("完整验收输出目录不唯一或未生成")
            report_root = matches[0]
            summary = json.loads((report_root / "acceptance.json").read_text(encoding="utf-8"))
            data_root = Path(summary["data_directory"]) if summary.get("data_directory") else report_root
            status = summary["status"]
        else:
            os.environ["UAVSIM_GUI_PROGRESS"] = "1"
            code = core_main(
                [
                    "run",
                    "--config",
                    str(root / "input_config.yaml"),
                    "--passes",
                    str(args.passes),
                    "--output",
                    str(root / "artifacts"),
                ]
            )
            matches = list((root / "artifacts").glob("run_*"))
            if len(matches) != 1:
                raise RuntimeError("仿真输出目录不唯一或未生成")
            data_root = matches[0]
            metadata = json.loads((data_root / "metadata.json").read_text(encoding="utf-8"))
            validation = json.loads((data_root / "validation.json").read_text(encoding="utf-8"))
            result = {
                f"pass{i}": load_exported_pass(data_root / f"uav_motion_pass{i}.mat")
                for i in range(1, args.passes + 1)
            }
            print("[上位机] 生成导航对照、天线几何与执行器结果图。", flush=True)
            report_progress(percent=95, stage="生成导航对照与天线图表")
            plot_pose_comparisons(result, data_root)
            plot_geometry_and_actuators(result, data_root, config)
            report_root = root
            route_stages = []
            for pass_name in result:
                quality = validation[pass_name].get("route_quality")
                if quality is None:
                    continue
                window = quality["crop_window"]
                if quality["speed_band_enforced"]:
                    detail = (
                        f"无物理扰动，速度门槛 {quality['speed_allowed_min_mps']:g}–"
                        f"{quality['speed_allowed_max_mps']:g} m/s；"
                        f"最长连续达标 {quality['longest_contiguous_valid_distance_m']:.3f} m。"
                    )
                else:
                    detail = (
                        f"物理扰动已开启，速度实测 {quality['full_speed_min_mps']:.3f}–"
                        f"{quality['full_speed_max_mps']:.3f} m/s（不设匀速硬门槛）；"
                        f"最长连续前进 {quality['longest_contiguous_valid_distance_m']:.3f} m。"
                    )
                if window is not None:
                    detail += (f"可裁剪 {window['start_time_s']:.3f}–{window['end_time_s']:.3f} s，"
                               f"前进 {window['forward_distance_m']:.3f} m。")
                else:
                    detail += "未找到同时落在真值与导航采样时刻上的目标长度航段。"
                route_stages.append({"name": f"{pass_name} 有效航段", "status": "passed" if quality["passed"] else "failed",
                                     "detail": detail})
            summary = {
                "data_directory": str(data_root),
                "resources": metadata["resource_plan"],
                "message": "参数仿真已完成；本模式不执行完整测试、OU 统计、收敛和 MATLAB 验收。",
                "metrics": {
                    "sar_samples": len(result["pass1"]["truth_sar"]["time_s"]),
                    "navigation_samples": len(result["pass1"]["navigation"]["time_s"]),
                    "simulation_display": f"{metadata['elapsed_s']:.2f} s",
                },
                "stages": [
                    {
                        "name": "参数仿真与运动检查",
                        "status": "passed" if not code else "failed",
                        "detail": f"{args.passes} 航次；物理与导航输出分别保存。",
                    },
                    *route_stages,
                    {
                        "name": "完整科研/接口验收",
                        "status": "skipped",
                        "detail": "本次为参数试验；需要时在上位机点击“完整验收”。",
                    },
                ],
            }
            write_json(root / "acceptance.json", summary)
            report_progress(percent=98, stage="生成离线报告")
            create_report(root, summary, result)
            status = "simulated" if not code and validation["passed"] else "failed"
        payload = {
            "type": "result",
            "mode": args.mode,
            "status": status,
            "report": str(report_root / "index.html"),
            "data_directory": str(data_root),
            "job_directory": str(root),
            "returncode": code,
        }
        write_json(root / "result.json", payload)
        event(payload)
        return code
    except Exception as error:
        traceback.print_exc()
        payload = {"type": "error", "message": str(error), "job_directory": str(root)}
        write_json(root / "error.json", payload)
        event(payload)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
