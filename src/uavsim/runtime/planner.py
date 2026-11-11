import math

from .hardware import detect_hardware


def plan_resources(config, passes: int, hardware: dict | None = None) -> dict:
    """保守内存规划，含父进程保留结果、子进程工作集及 IPC 副本；不减少积分精度。"""
    if passes < 1:
        raise ValueError("At least one pass required")
    hw = detect_hardware() if hardware is None else hardware
    s = config.simulation
    full = round((s.pre_stabilization_s + s.acquisition_duration_s) * s.dynamics_rate_hz) + 1
    inputs = round((s.pre_stabilization_s + s.acquisition_duration_s) * config.disturbances.input_rate_hz) + 1
    retained = full * 8 * 150 + inputs * 8 * 12
    working = full * 8 * 120 + inputs * 8 * 40 + 256 * 1024**2
    if config.runtime.backend == "matlab":
        working += 1024**3
    available = hw.get("memory_available_bytes")
    note = ""
    if available is None:
        # 未知硬件串行执行，但仍以保守预算拒绝超大型任务。
        budget = 1024**3
        note = "Available RAM unknown: conservative 1 GiB budget and serial execution"
    else:
        budget = max(
            0,
            min(
                available * config.runtime.memory_fraction,
                available - config.runtime.reserve_memory_gb * 1024**3,
            ),
        )
    parent = passes * retained + 128 * 1024**2
    maximum_by_memory = math.floor((budget - parent) / (working + retained))
    if maximum_by_memory < 1:
        raise MemoryError(
            f"Estimated RAM budget insufficient: need {(parent + working + retained) / 1024**3:.2f} GiB, "
            f"budget {budget / 1024**3:.2f} GiB. Reduce duration/pass count or free RAM."
        )
    cores = hw.get("physical_cores") or hw.get("logical_cpus", 1)
    cpu_limit = max(1, cores - 1)
    requested = config.runtime.max_workers or min(8, cpu_limit)
    workers = min(passes, requested, cpu_limit, maximum_by_memory)
    if full < 16000 and config.runtime.max_workers == 0:
        workers = 1
        note = "Short workload: serial execution avoids process startup and transfer overhead"
    if available is None:
        workers = 1
    if config.runtime.backend == "matlab":
        workers = 1
        note = "MATLAB backend executes passes serially with a 1 GiB native-process reserve"
    backend = {"rotorpy": "rotorpy_cpu", "matlab": "matlab_cpu"}.get(config.runtime.backend, "cpu")
    reason = {
        "rotorpy": "RotorPy 3.0 multirotor RHS with Hex-X adapter; float64 CPU. ",
        "matlab": "Vendored MATLAB legacy physical core; float64 CPU, serial native sessions. ",
    }.get(config.runtime.backend, "19-state sequential feedback RK4 uses float64 CPU. ")
    return {
        "hardware": hw,
        "workers": workers,
        "blas_threads_per_worker": config.runtime.blas_threads_per_worker,
        "memory_budget_bytes": int(budget),
        "estimated_retained_bytes_per_pass": retained,
        "estimated_working_bytes_per_worker": working,
        "estimated_peak_bytes": parent + workers * (working + retained),
        "backend": backend,
        "gpu_used": False,
        "backend_reason": reason + "GPU memory is detected; computation uses CPU.",
        "note": note,
    }
