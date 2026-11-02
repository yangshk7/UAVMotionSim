import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat, savemat

from uavsim.config.loader import save_config
from uavsim.config.validator import Config
from uavsim.motion.export import export_motion_segment
from uavsim.motion.segment import from_simulated_pass


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def write_json(path: str | Path, value: dict) -> None:
    Path(path).write_text(
        json.dumps(value, indent=2, ensure_ascii=False, default=json_default, allow_nan=False),
        encoding="utf-8",
    )


def _mat_safe(value):
    if value is None:
        return np.empty(0)
    if isinstance(value, dict):
        return {key: _mat_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        if not value:
            return np.empty(0)
        if all(isinstance(x, (int, float, bool)) for x in value):
            return np.array(value)
        return np.array([_mat_safe(x) for x in value], dtype=object)
    return value


def _navigation_csv(packet: dict, path: Path) -> None:
    fields = [
        ("position_sar_m", ("x_sar_m", "y_sar_m", "z_sar_m")),
        ("velocity_sar_mps", ("vx_sar_mps", "vy_sar_mps", "vz_sar_mps")),
        ("euler_NB_rpy_rad", ("roll_ned_rad", "pitch_ned_rad", "yaw_ned_rad")),
        ("euler_NB_rpy_deg", ("roll_ned_deg", "pitch_ned_deg", "yaw_ned_deg")),
        ("q_NB_wxyz", ("qw_nb", "qx_nb", "qy_nb", "qz_nb")),
        ("truth_omega_B_radps", ("truth_p_frd_radps", "truth_q_frd_radps", "truth_r_frd_radps")),
        ("truth_acceleration_sar_mps2", ("truth_ax_sar_mps2", "truth_ay_sar_mps2", "truth_az_sar_mps2")),
        ("apc_position_sar_m", ("apc_x_sar_m", "apc_y_sar_m", "apc_z_sar_m")),
        ("position_error_sar_m", ("position_error_x_m", "position_error_y_m", "position_error_z_m")),
        ("velocity_error_sar_mps", ("velocity_error_x_mps", "velocity_error_y_mps", "velocity_error_z_mps")),
        (
            "attitude_error_rotvec_B_rad",
            ("attitude_error_rx_rad", "attitude_error_ry_rad", "attitude_error_rz_rad"),
        ),
    ]
    arrays = [packet["time_s"][:, None]]
    names = ["time_s"]
    for field, labels in fields:
        names.extend(labels)
        arrays.append(packet[field])
    np.savetxt(
        path,
        np.column_stack(arrays),
        delimiter=",",
        header=",".join(names),
        comments="",
        fmt="%.16g",
        encoding="utf-8",
    )


def export_experiment(result: dict, output_dir: str | Path) -> Path:
    """MAT v5（<2 GiB/变量）+ CSV；输出目录必须为空，避免覆盖已有成果。"""
    root = Path(output_dir)
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"Refusing to overwrite nonempty output directory: {root}")
    root.mkdir(parents=True, exist_ok=True)
    cfg = Config.model_validate(result["config"])
    save_config(cfg, root / "config_snapshot.yaml")
    write_json(root / "metadata.json", result["metadata"])
    write_json(root / "validation.json", result["validation"])
    for pass_name, data in result.items():
        if not pass_name.startswith("pass"):
            continue
        if cfg.output.save_mat:
            savemat(
                root / f"uav_motion_{pass_name}.mat",
                {"pass_result": _mat_safe(data)},
                do_compression=True,
                long_field_names=True,
                oned_as="column",
            )
            savemat(
                root / f"navigation_{pass_name}.mat",
                {
                    "navigation": _mat_safe(data["navigation"]),
                    "navigation_truth": _mat_safe(data["navigation_truth"]),
                    "metadata": _mat_safe(data["metadata"]),
                },
                do_compression=True,
                long_field_names=True,
                oned_as="column",
            )
        if cfg.output.save_csv:
            export_motion_segment(
                from_simulated_pass(data, cfg), root, pass_name,
                result.get("validation", {}).get(pass_name, {}).get("route_quality"),
                {"base_seed": cfg.base_seed, "pass_seed": data["metadata"]["pass_seed"],
                 "config": cfg.model_dump(mode="json")},
            )
            truth = data["truth_sar"]
            fields = [
                ("position_sar_m", ("x_m", "y_m", "z_m")),
                ("velocity_sar_mps", ("vx_mps", "vy_mps", "vz_mps")),
                ("acceleration_sar_mps2", ("ax_mps2", "ay_mps2", "az_mps2")),
                ("q_NB_wxyz", ("qw", "qx", "qy", "qz")),
                ("euler_NB_rpy_rad", ("roll_rad", "pitch_rad", "yaw_rad")),
                ("omega_B_radps", ("p_radps", "q_radps", "r_radps")),
                ("apc_position_sar_m", ("apc_x_m", "apc_y_m", "apc_z_m")),
                ("apc_velocity_sar_mps", ("apc_vx_mps", "apc_vy_mps", "apc_vz_mps")),
                ("beam_sar_unit", ("beam_x", "beam_y", "beam_z")),
            ]
            labels = ["time_s"]
            arrays = [truth["time_s"][:, None]]
            for field, names in fields:
                labels.extend(names)
                arrays.append(truth[field])
            labels.extend(["nav_x_m", "nav_y_m", "nav_z_m"])
            arrays.append(data["navigation_sar"]["position_sar_m"])
            np.savetxt(
                root / f"uav_motion_{pass_name}.csv",
                np.column_stack(arrays),
                delimiter=",",
                header=",".join(labels),
                comments="",
                fmt="%.16g",
                encoding="utf-8",
            )
            _navigation_csv(data["navigation"], root / f"navigation_{pass_name}.csv")
            _navigation_csv(data["navigation_truth"], root / f"navigation_truth_{pass_name}.csv")
    if cfg.output.save_mat:
        summary = {key: result[key] for key in ["config", "metadata", "validation"]}
        savemat(
            root / "uav_motion_summary.mat",
            {"result": _mat_safe(summary)},
            do_compression=True,
            long_field_names=True,
            oned_as="column",
        )
    return root


def load_exported_pass(path: str | Path) -> dict:
    return loadmat(path, simplify_cells=True)["pass_result"]
