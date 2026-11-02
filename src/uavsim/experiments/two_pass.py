import importlib.metadata
import multiprocessing
import platform
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from time import perf_counter

from uavsim import __version__
from uavsim.dynamics.simulator import simulate_pass
from uavsim.runtime.planner import plan_resources
from uavsim.runtime.provenance import source_provenance


def run_experiment(config, passes: int = 2, resource_plan: dict | None = None) -> dict:
    """独立航次可并行；随机流依赖航次种子而不依赖进程/完成顺序。"""
    start = perf_counter()
    plan = resource_plan or plan_resources(config, passes)
    seeds = list(config.pass_seeds)
    if passes > len(seeds):
        raise ValueError("Provide an explicit distinct pass_seeds entry for every pass")
    if plan["workers"] > 1:
        with ProcessPoolExecutor(
            max_workers=plan["workers"], mp_context=multiprocessing.get_context("spawn")
        ) as pool:
            futures = [pool.submit(simulate_pass, config, seeds[index], index) for index in range(passes)]
            results = [future.result() for future in futures]
    else:
        results = [simulate_pass(config, seeds[index], index) for index in range(passes)]
    return {
        **{f"pass{index + 1}": result for index, result in enumerate(results)},
        "config": config.model_dump(mode="json"),
        "metadata": {
            **source_provenance(),
            "version": __version__,
            "python": platform.python_version(),
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "packages": {
                name: importlib.metadata.version(name) for name in ["numpy", "scipy", "pydantic", "PyYAML"]
            },
            "resource_plan": plan,
            "elapsed_s": perf_counter() - start,
            "coordinates": "Internal NED/FRD; external SAR X-forward Y-right Z-up; q_NB wxyz; SI",
        },
        "validation": {},
    }
