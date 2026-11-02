"""上位机后台任务：独立进程、UTF-8 日志、任务专属取消，不阻塞 Tk。"""

import json
import os
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from uavsim.config.loader import save_config

EVENT_PREFIX = "UAVSIM_GUI_EVENT "


def create_job(project: Path, config, mode: str) -> Path:
    if mode not in ("simulate", "acceptance"):
        raise ValueError("Unknown run mode")
    stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
    root = project / "main" / "gui_outputs" / f"{mode}_{stamp}"
    root.mkdir(parents=True, exist_ok=False)
    save_config(config, root / "input_config.yaml")
    (root / "session.json").write_text(
        json.dumps({"mode": mode, "status": "created", "created_at": stamp}, indent=2), encoding="utf-8"
    )
    return root


def launch_job(
    project: Path,
    root: Path,
    mode: str,
    passes: int,
    matlab: str,
    matlab_exe: str = "",
    sar_project: str = "",
) -> subprocess.Popen:
    if (
        passes not in (1, 2)
        or mode not in ("simulate", "acceptance")
        or matlab not in ("auto", "required", "skip")
    ):
        raise ValueError("Invalid run options")
    executable = Path(sys.executable)
    if executable.name.lower() == "pythonw.exe":
        executable = executable.with_name("python.exe")
    command = [
        str(executable),
        "-u",
        str(project / "main" / "internal" / "gui_worker.py"),
        "--job-dir",
        str(root),
        "--mode",
        mode,
        "--passes",
        str(passes),
        "--matlab",
        matlab,
    ]
    if matlab_exe:
        command += ["--matlab-exe", matlab_exe]
    if sar_project:
        command += ["--sar-project", sar_project]
    environment = os.environ.copy()
    environment.update(PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
    options = (
        {"creationflags": subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP}
        if os.name == "nt"
        else {"start_new_session": True}
    )
    return subprocess.Popen(
        command,
        cwd=project,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
        env=environment,
        **options,
    )


def stop_job(process: subprocess.Popen) -> None:
    """仅终止本程序启动且仍存活的任务树；不删除任何部分输出。"""
    if process.poll() is not None:
        return
    if os.name == "nt":
        result = subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            capture_output=True,
            creationflags=subprocess.CREATE_NO_WINDOW,
            check=False,
        )
        if result.returncode and process.poll() is None:
            raise RuntimeError("无法停止任务进程树，请查看日志")
    else:
        os.killpg(process.pid, signal.SIGTERM)
    process.wait(timeout=15)


def parse_event(line: str) -> dict | None:
    if not line.startswith(EVENT_PREFIX):
        return None
    event = json.loads(line[len(EVENT_PREFIX) :])
    if not isinstance(event, dict) or event.get("type") not in ("result", "error", "progress"):
        raise ValueError("Invalid GUI event")
    if event["type"] == "progress":
        import math

        key = "percent" if "percent" in event else "fraction"
        value = event.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= (100 if key == "percent" else 1):
            raise ValueError("Invalid progress value")
        if key == "fraction" and (type(event.get("pass_index")) is not int or event["pass_index"] < 0):
            raise ValueError("Invalid progress pass index")
    return event
