"""固定配置/种子，对比串行和并行；报告实际结果，不承诺固定加速比。"""

import argparse
from datetime import datetime
from pathlib import Path
from time import perf_counter

import numpy as np

from uavsim.config.loader import load_config
from uavsim.config.validator import Config
from uavsim.experiments.two_pass import run_experiment
from uavsim.runtime.planner import plan_resources
from uavsim.sar_adapter.matlab_export import write_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.48)
    parser.add_argument("--config", default="configs/experiment_two_pass.yaml")
    args = parser.parse_args()
    config = load_config(args.config)
    base = config.model_dump()
    base["simulation"]["acquisition_duration_s"] = args.duration
    config = Config.model_validate(base)
    timings = {}
    outputs = {}
    for workers in [1, 2]:
        data = config.model_dump()
        data["runtime"]["max_workers"] = workers
        case = Config.model_validate(data)
        plan = plan_resources(case, 2)
        start = perf_counter()
        outputs[workers] = run_experiment(case, 2, plan)
        timings[str(workers)] = perf_counter() - start
    identical = all(
        np.array_equal(outputs[1][name]["truth_full"][field], outputs[2][name]["truth_full"][field])
        for name in ["pass1", "pass2"]
        for field in ["position_N_m", "velocity_N_mps", "q_NB_wxyz", "omega_B_radps"]
    )
    report = {
        "duration_s": args.duration,
        "elapsed_s_by_requested_workers": timings,
        "serial_over_parallel_ratio": timings["1"] / timings["2"],
        "bitwise_identical_states": identical,
        "parallel_resource_plan": outputs[2]["metadata"]["resource_plan"],
        "worker_peak_working_set_bytes": [
            outputs[2][name]["metadata"]["peak_working_set_bytes"] for name in ["pass1", "pass2"]
        ],
    }
    root = Path("results") / ("benchmark_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    root.mkdir(parents=True)
    write_json(root / "benchmark.json", report)
    print(report)
    print(root.resolve())
    return 0 if identical else 1


if __name__ == "__main__":
    raise SystemExit(main())
