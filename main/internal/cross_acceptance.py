"""Run fresh external flights, align native clocks, and compute their comparison."""
import json
import os
import subprocess
import sys
from pathlib import Path

from uavsim.runtime.progress import report_progress

PROJECT = Path(__file__).resolve().parents[2]


def run_cross_acceptance(root):
    folder = root / "external_comparison"
    folder.mkdir()
    record = {"status": "failed", "steps": [], "comparison": None}
    wsl = ["wsl", "-d", "Ubuntu-24.04", "-u", "root", "--cd", str(PROJECT), "--exec"]
    creationflags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0

    def execute(name, command, timeout=700):
        print(f"[外部交叉对照] {name}", flush=True)
        report_progress(percent=98, stage="外部飞控交叉对照 · " + name)
        log = folder / f"{name}.log"
        with log.open("w", encoding="utf-8") as stream:
            result = subprocess.run(command, cwd=PROJECT, stdout=stream, stderr=subprocess.STDOUT,
                                    timeout=timeout, creationflags=creationflags)
        record["steps"].append({"name": name, "returncode": result.returncode, "log": log.name})
        if result.returncode:
            raise RuntimeError(f"{name} failed; see {log}")

    def fresh_run(name, relative, command):
        base = PROJECT / relative
        before = set(base.glob("*")) if base.exists() else set()
        execute(name, command)
        new = [p for p in base.glob("*") if p.is_dir() and p not in before]
        if len(new) != 1:
            raise RuntimeError(f"{name}: expected exactly one new run, got {len(new)}")
        return new[0]

    try:
        if os.name != "nt":
            record["status"] = "skipped"
            record["detail"] = "External acceptance requires the configured Windows WSL environment"
            return record
        execute("preflight", wsl + ["test", "-x", "/opt/uav_motion/sitl/4.7.2/venv/bin/python"], 30)
        sitl = fresh_run("sitl_flight", "results/sitl_route", wsl + [
            "/opt/uav_motion/sitl/4.7.2/venv/bin/python", "deployment/sitl/validate_flight.py",
            "--binary", "/opt/uav_motion/sitl/4.7.2/bin/arducopter",
            "--defaults", "/opt/uav_motion/sitl/4.7.2/copter.parm",
            "--output", "results/sitl_route", "--route", "--aligned-route", "--truth-rate-hz", "300"])
        px4 = fresh_run("px4_flight", "results/xtdrone_route", wsl + [
            "bash", "deployment/xtdrone/run_validation.sh", "--route", "--aligned-route",
            "--truth-rate-hz", "1000"])
        record["runs"] = {"sitl": str(sitl), "px4": str(px4)}
        for name, directory, pattern, python, script, log_arg, truth in [
            ("sitl", sitl, "logs/*.BIN", "/opt/uav_motion/sitl/4.7.2/venv/bin/python",
             "deployment/extract_sitl_dataflash.py", "--log", "truth_native.csv"),
            ("px4", px4, "log/**/*.ulg", "/opt/uav_motion/px4_venv/bin/python",
             "deployment/extract_px4_ulog.py", "--ulog", "truth_1000hz.csv"),
        ]:
            files = list(directory.glob(pattern))
            if len(files) != 1:
                raise RuntimeError(f"{name}: expected one native flight log")
            aligned = directory / "acceptance_clock"
            aligned.mkdir()
            def rel(path):
                return path.relative_to(PROJECT).as_posix()
            execute(name + "_extract", wsl + [python, script, log_arg, rel(files[0]),
                                            "--output", rel(aligned)])
            execute(name + "_align", [sys.executable, f"deployment/align_{name}_sim_clock.py",
                                     "--directory", str(directory), "--output", str(aligned)])
            execute(name + "_pair", [sys.executable, "deployment/validate_clock_aligned.py",
                                    "--directory", str(aligned), "--truth", truth])
        output = folder / "comparison.json"
        execute("compare", [sys.executable, "deployment/compare_aligned_routes.py",
                           "--sitl", str(sitl / "acceptance_clock"),
                           "--px4", str(px4 / "acceptance_clock"), "--output", str(output)])
        record["comparison"] = json.loads(output.read_text(encoding="utf-8"))
        record["status"] = "passed"
        record["detail"] = "Fresh flights and native-clock checks passed; cross-platform differences reported without an equivalence threshold"
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        record["detail"] = str(error)
    finally:
        (folder / "result.json").write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
    return record
