import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


def _output_root(base: str, prefix: str) -> Path:
    stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
    return Path(base).resolve() / f"{prefix}_{stamp}"


def _plots(result, root):
    from uavsim.visualization.disturbance_plot import plot_disturbances
    from uavsim.visualization.navigation_plot import plot_navigation
    from uavsim.visualization.state_plot import plot_states
    from uavsim.visualization.trajectory_3d import plot_trajectories

    plot_states(result, root)
    plot_trajectories(result, root)
    plot_disturbances(result, root)
    plot_navigation(result, root)


def main(argv=None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="Hex-X six-DOF UAV motion simulation (SI units)")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ["run", "validate"]:
        command = commands.add_parser(name)
        command.add_argument("--config", default="configs/rotorpy_hex_x12.yaml")
        command.add_argument("--output", default="results")
        command.add_argument("--passes", type=int, default=1 if name == "run" else 2)
        command.add_argument("--workers", type=int, default=None)
        if name == "validate":
            command.add_argument(
                "--quick", action="store_true", help="Only convergence uses 2 s; default is full duration"
            )
    command = commands.add_parser("plot")
    command.add_argument("--input", required=True)
    command = commands.add_parser("hardware")
    command.add_argument("--config", default="configs/rotorpy_hex_x12.yaml")
    command.add_argument("--passes", type=int, default=2)
    args = parser.parse_args(argv)
    try:
        if args.command == "plot":
            from uavsim.sar_adapter.matlab_export import load_exported_pass

            root = Path(args.input)
            files = sorted(root.glob("uav_motion_pass*.mat"))
            if not files:
                raise FileNotFoundError("No exported pass MAT files found")
            result = {f"pass{index + 1}": load_exported_pass(path) for index, path in enumerate(files)}
            _plots(result, root)
            print(f"Plots: {root.resolve()}")
            return 0
        from uavsim.config.loader import load_config
        from uavsim.config.validator import Config
        from uavsim.runtime.planner import plan_resources
        from uavsim.sar_adapter.matlab_export import export_experiment, write_json

        config = load_config(args.config)
        if getattr(args, "workers", None) is not None:
            data = config.model_dump()
            data["runtime"]["max_workers"] = args.workers
            config = Config.model_validate(data)
        plan = plan_resources(config, args.passes)
        if args.command == "hardware":
            import json

            print(json.dumps(plan, indent=2, ensure_ascii=False))
            return 0
        from uavsim.analysis.metrics import experiment_metrics
        from uavsim.experiments.two_pass import run_experiment

        root = _output_root(args.output, args.command)
        print(f"Config: {Path(args.config).resolve()}", flush=True)
        print(
            f"Rates Hz: dynamics={config.simulation.dynamics_rate_hz}, SAR={config.simulation.sar_rate_hz}, navigation={config.simulation.navigation_rate_hz}",
            flush=True,
        )
        print(
            f"Seeds: {config.pass_seeds[: args.passes]}; workers={plan['workers']}; backend={plan['backend']}",
            flush=True,
        )
        active = [
            name
            for group in [config.disturbances.translation, config.disturbances.attitude]
            for name, factor in group.items()
            if factor.enabled or config.disturbances.all_enabled
        ]
        print(
            f"A/B channels: {active}; navigation error={config.disturbances.navigation_position_error.enabled or config.disturbances.all_enabled}",
            flush=True,
        )
        print(
            f"Memory budget: {plan['memory_budget_bytes'] / 1024**3:.2f} GiB; estimated peak: {plan['estimated_peak_bytes'] / 1024**3:.2f} GiB",
            flush=True,
        )
        result = run_experiment(config, args.passes, plan)
        from uavsim.runtime.progress import report_progress

        report_progress(percent=80, stage="检查运动与有效航段")
        result["validation"] = experiment_metrics(result, config)
        report_progress(percent=82, stage="保存 MAT / CSV 数据")
        export_experiment(result, root)
        report_progress(percent=90, stage="生成运动图表")
        if config.output.save_plots:
            _plots(result, root)
        if args.command == "validate":
            from uavsim.analysis.convergence import convergence_study
            from uavsim.analysis.statistics import validate_ou_statistics

            print("Running automated tests, long OU statistics and convergence...", flush=True)
            project = Path(__file__).resolve().parents[2]
            junit = root / "pytest_results.xml"
            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pytest",
                    "-q",
                    f"--junitxml={junit}",
                    f"--basetemp={root / 'pytest_temp'}",
                    "-o",
                    f"cache_dir={root / 'pytest_cache'}",
                ],
                cwd=project,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
            (root / "pytest_output.txt").write_text(
                completed.stdout + "\n" + completed.stderr, encoding="utf-8"
            )
            # 任意基础测试失败立即停止，不进入后续验收。
            if completed.returncode:
                raise RuntimeError(f"Automated tests failed; see {root}/pytest_output.txt")
            ou = validate_ou_statistics(root)
            write_json(root / "ou_validation.json", ou)
            if not ou["passed"]:
                raise RuntimeError("OU statistical validation failed")
            convergence = convergence_study(config, root, 2.0 if args.quick else None)
            write_json(root / "convergence_validation.json", convergence)
            report = {
                "state_checks": result["validation"],
                "pytest_returncode": completed.returncode,
                "ou_statistics": ou,
                "convergence": convergence,
                "external_validation": {
                    "matlab": "Run matlab/run_matlab_validation.m explicitly",
                    "SITL": "Not performed by this command",
                    "Simscape": "Not performed by this command",
                },
            }
            report["passed"] = bool(result["validation"]["passed"] and ou["passed"] and convergence["passed"])
            write_json(root / "scientific_validation.json", report)
            if not report["passed"]:
                raise RuntimeError(f"Scientific validation failed: {root}")
        print(
            f"Elapsed simulation: {result['metadata']['elapsed_s']:.3f} s; state checks passed={result['validation']['passed']}",
            flush=True,
        )
        print(f"Output: {root}", flush=True)
        return 0 if result["validation"]["passed"] else 1
    except (ValueError, OSError, RuntimeError, MemoryError, FloatingPointError) as error:
        print(f"Error: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
