"""Audit one UAV-FD flight and compare onboard EKF/GPS speed at matched times.

This is a cross-airframe data-method check, not validation of the 12 kg Hex-X.
TimeUS is microseconds; GPS Spd and XKF1 VN/VE are metres per second.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.io import loadmat

EXPECTED_MD5 = "065e36e917d0e6188a3b766e17a720c7"
VARIABLES = ["IMU_0", "ATT", "XKF1_0", "GPS_0", "RCOU"] + [f"ESC_{i}" for i in range(6)]


def summarize_stream(array: np.ndarray) -> dict:
    if array.ndim != 2 or len(array) < 2 or not np.isfinite(array).all():
        raise ValueError("stream is empty, non-matrix, or contains nonfinite values")
    time_s = array[:, 1] * 1e-6
    increments = np.diff(time_s)
    if np.any(increments <= 0):
        raise ValueError("TimeUS is not strictly increasing")
    return {
        "samples": len(array),
        "start_s": float(time_s[0]),
        "end_s": float(time_s[-1]),
        "duration_s": float(time_s[-1] - time_s[0]),
        "median_sample_interval_s": float(np.median(increments)),
        "median_sample_rate_hz": float(1 / np.median(increments)),
        "largest_sample_gap_s": float(np.max(increments)),
    }


def audit(source: Path) -> dict:
    md5 = hashlib.md5(source.read_bytes()).hexdigest()  # noqa: S324 - verify published checksum
    if md5 != EXPECTED_MD5:
        raise ValueError(f"MD5 mismatch: {md5}")
    data = loadmat(source, variable_names=VARIABLES)
    missing = [name for name in VARIABLES if name not in data]
    if missing:
        raise ValueError(f"missing required data streams: {missing}")
    streams = {name: summarize_stream(data[name]) for name in VARIABLES}

    gps = data["GPS_0"]
    ekf = data["XKF1_0"]
    gps_time = gps[:, 1] * 1e-6
    ekf_time = ekf[:, 1] * 1e-6
    gps_speed = gps[:, 11]
    ekf_speed = np.hypot(ekf[:, 6], ekf[:, 7])
    indices = np.searchsorted(ekf_time, gps_time)
    left = np.clip(indices - 1, 0, len(ekf_time) - 1)
    right = np.clip(indices, 0, len(ekf_time) - 1)
    nearest = np.where(abs(ekf_time[left] - gps_time) <= abs(ekf_time[right] - gps_time), left, right)
    gaps = abs(ekf_time[nearest] - gps_time)
    valid = (gaps <= 0.03) & (gps[:, 3] >= 3) & np.isfinite(gps_speed)
    if valid.sum() < 100:
        raise ValueError("fewer than 100 3D-fix GPS points matched an EKF sample within 30 ms")
    residual = ekf_speed[nearest[valid]] - gps_speed[valid]
    motor_rpm = {f"motor_{i}": {
        "median_rpm": float(np.median(data[f"ESC_{i}"][:, 3])),
        "max_rpm": float(np.max(data[f"ESC_{i}"][:, 3])),
    } for i in range(6)}
    return {
        "source": str(source.resolve()),
        "source_url": "https://zenodo.org/records/7648996/files/NO_FAULT1.mat?download=1",
        "md5": md5,
        "checksum_verified": True,
        "streams": streams,
        "gps_altitude_min_max_m": [float(np.min(gps[:, 10])), float(np.max(gps[:, 10]))],
        "gps_max_ground_speed_mps": float(np.max(gps_speed)),
        "gps_ekf_speed_comparison": {
            "matched_3d_fix_samples": int(valid.sum()),
            "max_time_gap_s": float(np.max(gaps[valid])),
            "mean_ekf_minus_gps_speed_mps": float(np.mean(residual)),
            "rms_ekf_minus_gps_speed_mps": float(math.sqrt(np.mean(residual**2))),
            "method": "nearest TimeUS within 30 ms; EKF sqrt(VN^2+VE^2) minus GPS Spd",
            "limitation": "both values are onboard outputs, not independent position or velocity truth",
        },
        "motors": motor_rpm,
        "target_comparison": {
            "project_mass_kg": 12,
            "project_capture_height_m": 300,
            "project_capture_speed_mps": 10,
            "same_airframe": False,
            "pointwise_trajectory_comparison_valid": False,
            "reason": "different airframe, flight path, altitude reference, and operating envelope",
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
