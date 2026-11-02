from datetime import datetime
from pathlib import Path

from uavsim.analysis.metrics import experiment_metrics
from uavsim.cli import _plots
from uavsim.config.loader import load_config, merge
from uavsim.config.validator import Config
from uavsim.experiments.two_pass import run_experiment
from uavsim.sar_adapter.matlab_export import export_experiment


def run_example(name: str, override: dict, passes: int = 1) -> None:
    project = Path(__file__).resolve().parents[1]
    base = load_config(project / "configs/hex_x_12kg.yaml")
    config = Config.model_validate(merge(base.model_dump(), override))
    result = run_experiment(config, passes)
    result["validation"] = experiment_metrics(result, config)
    root = project / "results" / f"{name}_{datetime.now():%Y%m%d_%H%M%S_%f}"
    export_experiment(result, root)
    _plots(result, root)
    print(root)
