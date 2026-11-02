"""导出可单独交给 SAR 程序使用的一段运动真值和导航估计。"""

import json
from pathlib import Path

import numpy as np

from uavsim.motion.segment import MotionSegment, MotionSeries

_COLUMNS = (
    "time_s", "cg_x_sar_m", "cg_y_sar_m", "cg_z_sar_m",
    "cg_vx_sar_mps", "cg_vy_sar_mps", "cg_vz_sar_mps",
    "qw_nb", "qx_nb", "qy_nb", "qz_nb",
    "apc_x_sar_m", "apc_y_sar_m", "apc_z_sar_m",
)


def _write_series(series: MotionSeries, path: Path) -> None:
    values = np.column_stack((series.time_s, series.position_sar_m, series.velocity_sar_mps,
                              series.q_NB_wxyz, series.apc_position_sar_m))
    np.savetxt(path, values, delimiter=",", header=",".join(_COLUMNS), comments="",
               fmt="%.16g", encoding="utf-8")


def export_motion_segment(segment: MotionSegment, directory: str | Path, name: str,
                          route_quality: dict | None = None, provenance: dict | None = None) -> dict:
    """写两个 CSV 和契约 JSON；拒绝覆盖既有文件，返回相对文件名。"""
    root = Path(directory)
    if not root.is_dir() or not name or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789_" for c in name):
        raise ValueError("Existing directory and lowercase English segment name required")
    files = {
        "truth": f"motion_truth_{name}.csv",
        "navigation": f"motion_navigation_{name}.csv",
        "manifest": f"motion_segment_{name}.json",
    }
    if any((root / filename).exists() for filename in files.values()):
        raise FileExistsError("Refusing to overwrite an existing motion segment")
    _write_series(segment.truth, root / files["truth"])
    _write_series(segment.navigation, root / files["navigation"])
    manifest = {
        "schema_version": 1,
        "source": segment.source,
        "clock_alignment": segment.clock_alignment,
        "time_origin": "start_of_exported_flight_segment",
        "position_frame": "local_SAR_X_forward_Y_right_Z_up",
        "attitude_frame": "body_FRD_to_local_NED_quaternion_wxyz",
        "position_reference": "vehicle_CG",
        "apc_reference": "configured_FRD_lever_arm_applied_to_pose",
        "navigation_model": segment.navigation_model,
        "truth_kind": "simulated_physical_state",
        "navigation_kind": "estimated_pose",
        "truth_rate_hz": segment.truth_rate_hz,
        "navigation_rate_hz": segment.navigation_rate_hz,
        "truth_samples": len(segment.truth.time_s),
        "navigation_samples": len(segment.navigation.time_s),
        "duration_s": float(segment.truth.time_s[-1]),
        "files": files,
    }
    if route_quality is not None:
        manifest["route_quality"] = route_quality
    if provenance is not None:
        manifest["provenance"] = provenance
    (root / files["manifest"]).write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    return files


def _read_series(path: Path) -> MotionSeries:
    data = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8", ndmin=1)
    if data.dtype.names != _COLUMNS:
        raise ValueError(f"Unexpected motion columns in {path}")
    values = np.column_stack([data[name] for name in _COLUMNS])
    return MotionSeries(values[:, 0], values[:, 1:4], values[:, 4:7],
                        values[:, 7:11], values[:, 11:14])


def load_motion_segment(directory: str | Path, name: str) -> MotionSegment:
    """从标准三文件恢复一段运动，并复核声明的频率与样本数。"""
    root = Path(directory)
    manifest = json.loads((root / f"motion_segment_{name}.json").read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 1:
        raise ValueError("Unsupported motion segment schema")
    truth = _read_series(root / f"motion_truth_{name}.csv")
    navigation = _read_series(root / f"motion_navigation_{name}.csv")
    segment = MotionSegment(
        truth=truth,
        navigation=navigation,
        source=manifest["source"],
        truth_rate_hz=manifest["truth_rate_hz"],
        navigation_rate_hz=manifest["navigation_rate_hz"],
        clock_alignment=manifest["clock_alignment"],
        navigation_model=manifest["navigation_model"],
    )
    if (len(truth.time_s) != manifest["truth_samples"]
            or len(navigation.time_s) != manifest["navigation_samples"]
            or abs(truth.time_s[-1] - manifest["duration_s"]) > 1e-9):
        raise ValueError("Motion segment files do not match manifest")
    return segment
