"""将可配置的仿真 CG→APC 杆臂应用到同钟位姿；不声称物理测量标定。"""

import json
from pathlib import Path

import numpy as np

from uavsim.frames.coordinate import polar_to_sar
from uavsim.frames.quaternion import to_matrix

_INPUT_COLUMNS = (
    "time_s", "cg_x_sar_m", "cg_y_sar_m", "cg_z_sar_m",
    "cg_vx_sar_mps", "cg_vy_sar_mps", "cg_vz_sar_mps",
    "qw_nb", "qx_nb", "qy_nb", "qz_nb",
)


def apc_from_pose(position_sar_m: np.ndarray, q_nb_wxyz: np.ndarray,
                  lever_frd_m: np.ndarray) -> np.ndarray:
    """计算 p_APC,SAR=p_CG,SAR+C R_NB l_FRD；输入 N×3、N×4、3，单位 m。"""
    position = np.asarray(position_sar_m, dtype=np.float64)
    quaternion = np.asarray(q_nb_wxyz, dtype=np.float64)
    lever = np.asarray(lever_frd_m, dtype=np.float64)
    if (position.ndim != 2 or position.shape[1] != 3 or quaternion.shape != (len(position), 4)
            or lever.shape != (3,) or not np.all(np.isfinite(position))
            or not np.all(np.isfinite(quaternion)) or not np.all(np.isfinite(lever))):
        raise ValueError("Finite N×3 position, N×4 quaternion and FRD three-vector required")
    if np.max(np.abs(np.linalg.norm(quaternion, axis=1) - 1)) > 1e-6:
        raise ValueError("Attitude quaternion must have unit norm")
    displacement_n = np.einsum("nij,j->ni", to_matrix(quaternion), lever)
    return position + polar_to_sar(displacement_n)


def _read_series(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8", ndmin=1)
    if rows.dtype.names != _INPUT_COLUMNS or len(rows) < 2:
        raise ValueError(f"Unexpected aligned pose columns: {path}")
    values = np.column_stack([rows[name] for name in _INPUT_COLUMNS])
    if not np.all(np.isfinite(values)) or abs(values[0, 0]) > 0.01 or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError(f"Invalid pose time or values: {path}")
    return values, values[:, 1:4], values[:, 7:11]


def export_apc_from_aligned(source: str | Path, destination: str | Path,
                            lever_frd_m: tuple[float, float, float]) -> dict:
    """从已验证的外部同钟 CSV 导出真值/导航 APC，拒绝覆盖且保留原始 CG。"""
    source, destination = Path(source), Path(destination)
    alignment_path = source / "alignment.json"
    pair_path = source / "pair_validation.json"
    if not alignment_path.is_file() or not pair_path.is_file():
        raise ValueError("Verified clock alignment and pair validation are required")
    alignment = json.loads(alignment_path.read_text(encoding="utf-8"))
    pair = json.loads(pair_path.read_text(encoding="utf-8"))
    if pair.get("status") != "shared_simulation_clock_paired":
        raise ValueError("Clock pairing is not verified")
    filenames = alignment.get("files", {})
    if filenames.get("navigation") != "navigation_100hz.csv" or filenames.get("truth") not in (
            "truth_1000hz.csv", "truth_native.csv"):
        raise ValueError("Unknown aligned pose file names")
    lever = np.asarray(lever_frd_m, dtype=np.float64)
    data = {}
    for role in ("truth", "navigation"):
        values, position, quaternion = _read_series(source / filenames[role])
        apc = apc_from_pose(position, quaternion, lever)
        data[role] = np.column_stack((values, apc))
    if abs(data["truth"][-1, 0] - data["navigation"][-1, 0]) > 0.01:
        raise ValueError("Truth and navigation route end times differ")
    destination.mkdir(parents=True, exist_ok=False)
    output_files = {"truth": "apc_truth.csv", "navigation": "apc_navigation.csv"}
    header = ",".join((*_INPUT_COLUMNS, "apc_x_sar_m", "apc_y_sar_m", "apc_z_sar_m"))
    for role, filename in output_files.items():
        np.savetxt(destination / filename, data[role], delimiter=",", header=header,
                   comments="", fmt="%.16g", encoding="utf-8")
    manifest = {
        "status": "simulation_geometry_applied_not_physical_calibration",
        "source_alignment": str(alignment_path.resolve()),
        "source_pair_validation": str(pair_path.resolve()),
        "lever_reference": "vehicle_CG_to_SAR_APC",
        "lever_frame": "body_FRD_X_forward_Y_right_Z_down",
        "lever_frd_m": lever.tolist(),
        "level_attitude_offset_sar_m": [float(lever[0]), float(lever[1]), -float(lever[2])],
        "source_of_default": "UAV_Motion_Sim and 无人机航迹仿真 simulation specifications",
        "formula": "p_APC,SAR = p_CG,SAR + diag(1,1,-1) R_NB lever_FRD",
        "files": output_files,
        "samples": {role: len(values) for role, values in data.items()},
    }
    (destination / "lever_arm_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return manifest
