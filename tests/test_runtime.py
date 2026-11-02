import pytest

from uavsim.config.validator import Config
from uavsim.runtime.planner import plan_resources


def test_planner_resource_limits_and_gpu_transparency():
    hardware = {
        "logical_cpus": 16,
        "physical_cores": 8,
        "memory_available_bytes": 16 * 1024**3,
        "gpus": [{"name": "test GPU", "free_memory_bytes": 8 * 1024**3}],
    }
    plan = plan_resources(Config(), 2, hardware)
    assert plan["workers"] == 2
    assert plan["estimated_peak_bytes"] <= plan["memory_budget_bytes"]
    assert plan["backend"] == "cpu" and not plan["gpu_used"]
    hardware["physical_cores"] = 1
    assert plan_resources(Config(), 2, hardware)["workers"] == 1
    hardware["memory_available_bytes"] = 100 * 1024**2
    with pytest.raises(MemoryError):
        plan_resources(Config(), 2, hardware)
