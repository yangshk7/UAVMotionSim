"""Fit UAV-FD ESC RPM against its own PWM commands; do not transfer to Hex-X12.

RCOU C9..C14 map to ESC_0..ESC_5 in the dataset author's sync_zoh_exa.m.
TimeUS is microseconds, PWM is microseconds, RPM is revolutions per minute.
The searched time shift aligns streams; it is not a motor time constant.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat
from validate_public_uav_fd import EXPECTED_MD5, audit

VALIDATION_MD5 = "22488aa7ed8fee78c0568149abc8efc0"


def fit_motor(pwm: np.ndarray, esc: np.ndarray) -> dict:
    """Return chronological holdout metrics for a linear active-motor RPM map."""
    pwm_time = pwm[:, 1] * 1e-6
    esc_time = esc[:, 1] * 1e-6
    rpm = esc[:, 3]
    active = (rpm > 1500) & (esc_time >= pwm_time[0] + 0.3) & (esc_time <= pwm_time[-1])
    if np.count_nonzero(active) < 150:
        raise ValueError("too few active ESC samples")
    split_time = float(np.quantile(esc_time[active], 0.7))
    train = active & (esc_time <= split_time)
    test = active & (esc_time > split_time)
    best = None
    for lag_s in np.arange(0, 0.301, 0.02):
        command = np.interp(esc_time - lag_s, pwm_time, pwm[:, 2])
        use_train = train & (command > 1200)
        use_test = test & (command > 1200)
        if np.count_nonzero(use_train) < 100 or np.count_nonzero(use_test) < 50:
            continue
        design = np.column_stack([command[use_train], np.ones(np.count_nonzero(use_train))])
        slope, offset = np.linalg.lstsq(design, rpm[use_train], rcond=None)[0]
        predicted_train = slope * command[use_train] + offset
        train_rmse = float(np.sqrt(np.mean((predicted_train - rpm[use_train]) ** 2)))
        if best is None or train_rmse < best[0]:
            best = (train_rmse, float(lag_s), float(slope), float(offset), command, use_train, use_test)
    if best is None:
        raise ValueError("no valid command/RPM alignment")
    train_rmse, lag_s, slope, offset, command, use_train, use_test = best
    observed = rpm[use_test]
    predicted = slope * command[use_test] + offset
    baseline = float(np.median(rpm[use_train]))
    test_rmse = float(np.sqrt(np.mean((predicted - observed) ** 2)))
    constant_rmse = float(np.sqrt(np.mean((baseline - observed) ** 2)))
    return {
        "active_train_samples": int(np.count_nonzero(use_train)),
        "active_test_samples": int(np.count_nonzero(use_test)),
        "fit_time_shift_s": lag_s,
        "slope_rpm_per_pwm_us": slope,
        "offset_rpm": offset,
        "train_rmse_rpm": train_rmse,
        "test_rmse_rpm": test_rmse,
        "test_constant_baseline_rmse_rpm": constant_rmse,
        "test_r2": float(1 - np.sum((predicted - observed) ** 2) / np.sum((observed - np.mean(observed)) ** 2)),
        "train_pwm_range_us": [float(np.min(command[use_train])), float(np.max(command[use_train]))],
        "test_pwm_range_us": [float(np.min(command[use_test])), float(np.max(command[use_test]))],
    }


def fit(source: Path) -> dict:
    checked = audit(source)
    if checked["md5"] != EXPECTED_MD5:
        raise ValueError("unexpected source checksum")
    data = loadmat(source, variable_names=["RCOU"] + [f"ESC_{i}" for i in range(6)])
    pwm_time_and_channels = data["RCOU"]
    motors = {}
    for index in range(6):
        pwm_channel = index + 9
        # RCOU columns 0/1 are LineNo/TimeUS; C9 is zero-based column 10.
        pwm = np.column_stack([pwm_time_and_channels[:, :2], pwm_time_and_channels[:, index + 10]])
        motors[f"esc_{index}"] = {"pwm_channel": f"C{pwm_channel}",
                                    **fit_motor(pwm, data[f"ESC_{index}"])}
    return {
        "source": str(source.resolve()),
        "md5": checked["md5"],
        "mapping_source": "UAV-FD sync_zoh_exa.m: RCOU(:,11:16) -> ESC_0..ESC_5",
        "model": "RPM = slope * PWM + offset, only while RPM > 1500 and PWM > 1200",
        "split": "first 70% of active time for fit/shift selection; final 30% untouched holdout",
        "time_shift_note": "grid 0..0.30 s in 0.02 s steps; alignment offset, not motor time constant",
        "motors": motors,
        "transfer_to_hex_x12": False,
        "limitation": "different aircraft/ESC/propeller; no measured thrust or torque; 9.3 Hz ESC logging",
    }


def validate_other_flight(source: Path, fitted: dict) -> dict:
    """Apply the first flight's six maps unchanged to a second flight."""
    checksum = hashlib.md5(source.read_bytes()).hexdigest()  # noqa: S324 - published checksum
    if checksum != VALIDATION_MD5:
        raise ValueError(f"independent flight MD5 mismatch: {checksum}")
    data = loadmat(source, variable_names=["RCOU"] + [f"ESC_{i}" for i in range(6)])
    pwm = data["RCOU"]
    pwm_time = pwm[:, 1] * 1e-6
    motors = {}
    for index in range(6):
        model = fitted["motors"][f"esc_{index}"]
        esc = data[f"ESC_{index}"]
        esc_time = esc[:, 1] * 1e-6
        command = np.interp(esc_time - model["fit_time_shift_s"], pwm_time, pwm[:, index + 10])
        valid = ((esc[:, 3] > 1500) & (command > 1200)
                 & (esc_time >= pwm_time[0] + 0.3) & (esc_time <= pwm_time[-1]))
        if np.count_nonzero(valid) < 100:
            raise ValueError(f"ESC_{index} has too few active validation samples")
        observed = esc[valid, 3]
        predicted = model["slope_rpm_per_pwm_us"] * command[valid] + model["offset_rpm"]
        motors[f"esc_{index}"] = {
            "samples": int(np.count_nonzero(valid)),
            "rmse_rpm": float(np.sqrt(np.mean((predicted - observed) ** 2))),
            "mean_error_rpm": float(np.mean(predicted - observed)),
            "r2": float(1 - np.sum((predicted - observed) ** 2) / np.sum((observed - np.mean(observed)) ** 2)),
        }
    return {"source": str(source.resolve()), "md5": checksum, "motors": motors,
            "method": "first-flight coefficients and shifts fixed; no refitting"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--validation-source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = fit(args.source)
    if args.validation_source:
        result["independent_flight_validation"] = validate_other_flight(args.validation_source, result)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
