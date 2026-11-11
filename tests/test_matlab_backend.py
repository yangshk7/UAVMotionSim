"""MAT-file contract checks without requiring a MATLAB license."""

from pathlib import Path

import pytest
from scipy.io import loadmat

from uavsim.config.loader import load_config
from uavsim.dynamics import matlab_backend


def test_matlab_input_preserves_long_physical_field_names(monkeypatch):
    config = load_config(Path(__file__).resolve().parents[1] / "main" / "config.yaml")
    config.simulation.acquisition_duration_s = 0.1
    config.simulation.pre_stabilization_s = 0.0
    monkeypatch.setattr(matlab_backend, "resolve_matlab", lambda _: "matlab")

    class InputInspected(Exception):
        pass

    def inspect_input(executable, input_file, output_file, timeout_s, pass_index):
        content = loadmat(input_file, simplify_cells=True)
        field = "max_horizontal_acceleration_mps2"
        assert len(field) > 31
        assert content["controller"][field] == config.controller.max_horizontal_acceleration_mps2
        assert content["initial_state"].shape == (19,)
        assert content["external"].shape[1] == 6
        raise InputInspected

    monkeypatch.setattr(matlab_backend, "run_matlab", inspect_input)
    with pytest.raises(InputInspected):
        matlab_backend.simulate_matlab_pass(config, 123)
