"""无重型依赖的硬件探测；探测失败保留 unknown，不把共享显存当独显显存。"""

import csv
import ctypes
import io
import json
import os
import platform
import shutil
import subprocess


def _run(args: list[str], timeout_s: float = 5.0) -> str:
    kwargs = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {}
    return subprocess.run(
        args, capture_output=True, text=True, timeout=timeout_s, check=True, **kwargs
    ).stdout


def memory_status() -> tuple[int | None, int | None]:
    if os.name == "nt":

        class MemoryStatus(ctypes.Structure):
            _fields_ = [("length", ctypes.c_ulong), ("load", ctypes.c_ulong)] + [
                (name, ctypes.c_ulonglong)
                for name in [
                    "total_physical",
                    "available_physical",
                    "total_page",
                    "available_page",
                    "total_virtual",
                    "available_virtual",
                    "available_extended",
                ]
            ]

        status = MemoryStatus()
        status.length = ctypes.sizeof(status)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return status.total_physical, status.available_physical
    elif os.path.exists("/proc/meminfo"):
        with open("/proc/meminfo", encoding="ascii") as stream:
            items = {line.split(":")[0]: int(line.split()[1]) * 1024 for line in stream}
        return items.get("MemTotal"), items.get("MemAvailable")
    return None, None


def detect_hardware() -> dict:
    total, available = memory_status()
    info = {
        "os": platform.platform(),
        "cpu_name": platform.processor(),
        "logical_cpus": os.cpu_count() or 1,
        "physical_cores": None,
        "memory_total_bytes": total,
        "memory_available_bytes": available,
        "gpus": [],
        "probe_notes": [],
    }
    nvidia = shutil.which("nvidia-smi")
    if nvidia:
        try:
            output = _run(
                [
                    nvidia,
                    "--query-gpu=name,memory.total,memory.free,driver_version",
                    "--format=csv,noheader,nounits",
                ]
            )
            for name, total_mb, free_mb, driver in csv.reader(io.StringIO(output), skipinitialspace=True):
                info["gpus"].append(
                    {
                        "name": name,
                        "total_memory_bytes": int(float(total_mb) * 1024**2),
                        "free_memory_bytes": int(float(free_mb) * 1024**2),
                        "driver": driver,
                        "source": "nvidia-smi",
                    }
                )
        except (subprocess.SubprocessError, ValueError, OSError) as error:
            info["probe_notes"].append(f"nvidia-smi unavailable: {error}")
    if os.name == "nt":
        try:
            script = "@{cpu=@(Get-CimInstance Win32_Processor | Select-Object Name,NumberOfCores);gpu=@(Get-CimInstance Win32_VideoController | Select-Object Name)} | ConvertTo-Json -Depth 4 -Compress"
            data = json.loads(
                _run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script], 8.0)
            )
            if data["cpu"]:
                info["physical_cores"] = sum(x["NumberOfCores"] for x in data["cpu"])
                info["cpu_name"] = "; ".join(x["Name"] for x in data["cpu"])
            else:
                info["probe_notes"].append("CIM CPU query returned no data; using logical CPU count")
            known = {g["name"].lower() for g in info["gpus"]}
            for adapter in data["gpu"]:
                if adapter["Name"].lower() not in known:
                    info["gpus"].append(
                        {
                            "name": adapter["Name"],
                            "total_memory_bytes": None,
                            "free_memory_bytes": None,
                            "source": "Win32_VideoController; memory unknown",
                        }
                    )
        except (subprocess.SubprocessError, ValueError, OSError, KeyError, TypeError) as error:
            info["probe_notes"].append(f"CIM probe unavailable: {error}")
    return info
