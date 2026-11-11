"""Independent deployment smoke: compare actual RotorPy truth with analytic straight flight."""

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from uavsim.analysis.metrics import experiment_metrics
from uavsim.config.loader import load_config
from uavsim.experiments.two_pass import run_experiment
from uavsim.sar_adapter.matlab_export import export_experiment, write_json

PROJECT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=PROJECT/"results"/("reproduction_"+datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")))
    args = parser.parse_args()
    config = load_config(PROJECT/"configs/reproduction_baseline.yaml")
    result = run_experiment(config, 2)
    result["validation"] = experiment_metrics(result, config)
    residuals = {}
    for name in ("pass1", "pass2"):
        data = result[name]
        expected = data["reference"]["apc_position_sar_m"]
        actual = data["truth_sar"]["apc_position_sar_m"]
        residuals[name] = float(np.max(np.abs(actual-expected)))
        if data["metadata"]["physical_backend"] != "rotorpy":
            raise RuntimeError("Expected actual RotorPy backend")
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-8)
    if not result["validation"]["passed"]:
        raise RuntimeError("Physical/time contract failed")
    export_experiment(result, args.output)
    report = {"passed": True, "version": result["metadata"]["version"], "python": sys.version,
              "packages": result["metadata"]["packages"], "seeds": list(config.pass_seeds),
              "analytic_position_tolerance_m": 1e-8, "position_max_abs_residual_m": residuals,
              "source_sha256": result["metadata"]["source_sha256"],
              "scope": "Fresh-path RotorPy two-pass analytic smoke; not full scientific/external flight acceptance"}
    write_json(args.output/"reproduction.json", report)
    print(args.output/"reproduction.json")


if __name__ == "__main__":
    main()
