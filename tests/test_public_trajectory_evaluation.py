import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def load_module():
    path = Path(__file__).resolve().parents[1] / "deployment/evaluate_public_ntu_viral.py"
    spec = importlib.util.spec_from_file_location("public_trajectory_evaluation", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rigid_alignment_recovers_known_transform_without_scale():
    module = load_module()
    time = np.linspace(0, 10, 200)
    source = np.column_stack([np.sin(time), np.cos(0.7 * time), time / 5])
    known_rotation = Rotation.from_euler("xyz", [12, -7, 31], degrees=True).as_matrix()
    target = source @ known_rotation + [3, -2, 5]
    fitted_rotation, fitted_translation = module.rigid_alignment(source, target)
    assert np.linalg.det(fitted_rotation) == pytest.approx(1)
    assert fitted_rotation == pytest.approx(known_rotation, abs=1e-12)
    assert fitted_translation == pytest.approx([3, -2, 5], abs=1e-12)
    assert module.metrics(source @ fitted_rotation + fitted_translation, target)["ate_rms_m"] < 1e-10


def test_truth_interpolation_rejects_large_time_gaps():
    module = load_module()
    truth_time = np.r_[np.arange(0, 1_600_000_000, 50_000_000), 3_000_000_000]
    truth_position = np.column_stack([truth_time * 1e-9, np.zeros(len(truth_time)), np.zeros(len(truth_time))])
    estimate_time = np.r_[np.arange(10_000_000, 1_500_000_000, 10_000_000),
                          np.arange(2_200_000_000, 2_700_000_000, 10_000_000)]
    estimate = np.column_stack([estimate_time * 1e-9, np.zeros(len(estimate_time)), np.zeros(len(estimate_time))])
    matched_time, matched_estimate, matched_truth = module.paired_positions(
        truth_time, truth_position, estimate_time, estimate
    )
    assert len(matched_time) == 149
    assert matched_time.max() < 1_500_000_000
    assert matched_estimate == pytest.approx(matched_truth)
