"""Check the sampled tube definition with a known outlier."""

import csv
import importlib.util
import json
from pathlib import Path


def test_sampled_route_tube_percentile_and_outlier(tmp_path: Path) -> None:
    script = Path(__file__).resolve().parents[1] / "deployment" / "measure_route_tube.py"
    spec = importlib.util.spec_from_file_location("measure_route_tube", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    (tmp_path / "validation.json").write_text(
        json.dumps({"passed": True, "route_duration_s": 1.0, "route_samples": 22}), encoding="utf-8"
    )
    with (tmp_path / "route_aligned_sar.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "truth_sim_time_s", "truth_y_sar_m", "truth_z_sar_m", "nav_y_sar_m", "nav_z_sar_m"
            ],
        )
        writer.writeheader()
        for index in range(20):
            writer.writerow({
                "truth_sim_time_s": index / 20,
                "truth_y_sar_m": 1 if index < 19 else 2,
                "truth_z_sar_m": 300,
                "nav_y_sar_m": 0,
                "nav_z_sar_m": 300,
            })
        for _ in range(2):
            writer.writerow({
                "truth_sim_time_s": 19 / 20,
                "truth_y_sar_m": 2,
                "truth_z_sar_m": 300,
                "nav_y_sar_m": 0,
                "nav_z_sar_m": 300,
            })

    result = module.measure(tmp_path, "px4")
    assert result["truth"]["diameter_95_m"] == 2
    assert result["truth"]["diameter_all_sampled_m"] == 4
    assert result["navigation"]["diameter_95_m"] == 0
    assert result["unique_truth_samples"] == 20
    assert result["matched_samples"] == 22
    assert result["mean_truth_sample_rate_hz"] == 19
