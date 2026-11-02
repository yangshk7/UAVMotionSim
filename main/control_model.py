"""上位机参数映射；界面只更新明确字段，保留未显示的完整模型参数。"""

import math
import re
from copy import deepcopy
from dataclasses import dataclass

from pydantic import ValidationError

from uavsim.config.validator import Config


@dataclass(frozen=True)
class Field:
    path: str
    label: str
    kind: str = "float"
    choices: tuple[str, ...] = ()
    nullable: bool = False


def get_value(data: dict, path: str):
    value = data
    for key in path.split("."):
        value = value[key]
    return value


def set_value(data: dict, path: str, value) -> None:
    node = data
    *parents, key = path.split(".")
    for parent in parents:
        node = node[parent]
    node[key] = value


def parse_value(field: Field, value):
    """标量按原类型解析；标准差单位由标签说明；空值只允许可选字段。"""
    if value is None or (isinstance(value, str) and not value.strip()):
        if field.nullable:
            return None
        raise ValueError(f"{field.label}不能为空")
    if field.kind == "bool":
        if isinstance(value, bool):
            return value
        if str(value).lower() not in ("true", "false", "1", "0"):
            raise ValueError(f"{field.label}必须为开/关")
        return str(value).lower() in ("true", "1")
    if field.kind in ("choice", "str"):
        result = str(value)
        if field.choices and result not in field.choices:
            raise ValueError(f"{field.label}选项无效")
        return result
    try:
        if field.kind in ("vector", "seeds"):
            parts = (
                value
                if isinstance(value, (list, tuple))
                else re.split(r"[,，\s]+", str(value).strip().strip("[]()").strip())
            )
            result = [int(v) if field.kind == "seeds" else float(v) for v in parts]
            if field.kind == "vector" and len(result) != 3:
                raise ValueError("Three components required")
        else:
            result = int(str(value)) if field.kind == "int" else float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field.label}数值格式不正确") from error
    numbers = result if isinstance(result, list) else [result]
    if not all(math.isfinite(v) for v in numbers):
        raise ValueError(f"{field.label}必须为有限数值")
    return result


def apply_fields(base: Config, fields: list[Field], values: dict) -> Config:
    """先完整校验再返回新配置，不更改原对象、不自动放宽物理约束。"""
    data = deepcopy(base.model_dump())
    for field in fields:
        set_value(data, field.path, parse_value(field, values[field.path]))
    try:
        return Config.model_validate(data)
    except ValidationError as error:
        labels = {field.path: field.label for field in fields}
        details = []
        for item in error.errors(include_url=False):
            path = ".".join(str(x) for x in item["loc"])
            name = labels.get(path, path or "组合参数")
            message = item["msg"].removeprefix("Value error, ")
            translations = {
                "All update rates must divide dynamics_rate_hz": "积分频率必须是各输出/控制频率的整数倍",
                "Durations must lie on all sample grids": "时长必须同时落在积分、SAR、导航和扰动采样网格上",
                "Common disturbance rate must be >= integration rate": "公共扰动频率不能低于积分频率",
                "Reference speed exceeds controller limit": "参考速度超过飞控速度上限",
                "Dynamics inputs require independently specified N or N*m amplitude": "动力学层必须重新明确填写 N 或 N*m 幅度",
                "Pass seeds must be nonnegative and distinct": "各航次随机种子必须非负且互不相同",
            }
            details.append(f"{name}：{translations.get(message, message)}")
        raise ValueError("\n".join(details)) from error


NAVIGATION_GROUPS = [
    ("位置估计误差 · SAR X/Y/Z", "navigation_position_error", "bias_std_sar_m", "noise_std_sar_m", "m"),
    ("速度估计误差 · SAR X/Y/Z", "navigation_velocity_error", "bias_std_sar_mps", "noise_std_sar_mps", "m/s"),
    (
        "姿态估计误差 · FRD 旋转向量",
        "navigation_attitude_error",
        "bias_std_body_deg",
        "noise_std_body_deg",
        "deg",
    ),
]

FLIGHT_FIELDS = [
    Field("trajectory.speed_mps", "名义速度 (m/s)"),
    Field("trajectory.route_length_m", "目标有效航线长度 (m)"),
    Field("trajectory.speed_tolerance_fraction", "有效航段速度容差比例（0.1=±10%）"),
    Field("simulation.duration_mode", "时长计算方式", "choice", ("route_length", "fixed")),
    Field("simulation.duration_margin_s", "额外飞行裕量 (s)"),
    Field("trajectory.apc_height_m", "APC 名义高度 (m)"),
    Field("trajectory.yaw_deg", "名义偏航 (deg)"),
    Field("simulation.acquisition_duration_s", "固定模式航迹时长 (s)"),
    Field("simulation.pre_stabilization_s", "预稳定时长 (s)"),
    Field("simulation.dynamics_rate_hz", "积分频率 (Hz)", "int"),
    Field("simulation.sar_rate_hz", "SAR 输出频率 (Hz)", "int"),
    Field("simulation.navigation_rate_hz", "导航频率 (Hz)", "int"),
    Field("disturbances.input_rate_hz", "公共扰动频率 (Hz)", "int"),
    Field("antenna.look_angle_deg", "波束下视角 (deg)"),
    Field("antenna.squint_angle_deg", "波束斜视角 (deg)"),
    Field("vehicle.mass_kg", "机体质量 (kg)"),
    Field("vehicle.motor_time_constant_s", "电机时间常数 (s)"),
    Field("vehicle.max_rotor_thrust_N", "单旋翼推力上限 (N)"),
]

LEVER_ARM_FIELDS = [Field("antenna.phase_center_frd_m", "质心 → APC 杆臂 (m)", "vector")]

RESOURCE_FIELDS = [
    Field("runtime.backend", "动力学后端", "choice", ("rotorpy", "cpu", "auto")),
    Field("base_seed", "基础随机种子", "int"),
    Field("pass_seeds", "航次种子（逗号分隔）", "seeds"),
    Field("nominal_baseline_sar_m", "名义双航次基线 SAR (m)", "vector"),
    Field("runtime.max_workers", "并行进程上限（0=自动）", "int"),
    Field("runtime.memory_fraction", "可用内存使用比例 (0–0.9)"),
    Field("runtime.reserve_memory_gb", "保留内存 (GiB)"),
    Field("runtime.blas_threads_per_worker", "每进程数值库线程", "int"),
]
