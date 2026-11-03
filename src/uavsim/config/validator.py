"""配置集中校验；SI 单位由字段名或显式 unit 表明。未知字段视为错误。"""

import math
from functools import reduce
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

Vector3 = tuple[float, float, float]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, validate_default=True)


class VehicleConfig(StrictModel):
    model: Literal["HexX12"] = "HexX12"
    mass_kg: float = Field(12.0, gt=0)
    inertia_kg_m2: tuple[Vector3, Vector3, Vector3] = ((0.55, 0.0, 0.0), (0.0, 0.60, 0.0), (0.0, 0.0, 1.0))
    arm_length_m: float = Field(0.55, gt=0)
    max_rotor_thrust_N: float = Field(42.0, gt=0)
    kf: float = Field(8e-5, gt=0, description="N/(rad/s)^2")
    km: float = Field(2.4e-6, ge=0, description="N m/(rad/s)^2")
    motor_time_constant_s: float = Field(0.1, gt=0)
    gravity_mps2: float = Field(9.81, gt=0)
    drag_enabled: bool = False
    air_density_kg_m3: float = Field(1.225, gt=0)
    drag_cd_area_m2: Vector3 = (0.0, 0.0, 0.0)

    @model_validator(mode="after")
    def physical(self):
        j = np.array(self.inertia_kg_m2)
        if not np.allclose(j, j.T, atol=1e-12) or np.min(np.linalg.eigvalsh(j)) <= 0:
            raise ValueError("Inertia must be symmetric positive definite")
        moments = np.linalg.eigvalsh(j)
        if 2 * moments[-1] > sum(moments) + 1e-12:
            raise ValueError("Principal moments violate rigid-body triangle inequality")
        if self.mass_kg * self.gravity_mps2 >= 6 * self.max_rotor_thrust_N:
            raise ValueError("Insufficient thrust for hover")
        if min(self.drag_cd_area_m2) < 0:
            raise ValueError("Drag Cd*A must be nonnegative")
        return self


class SimulationConfig(StrictModel):
    dynamics_rate_hz: int = Field(3200, gt=0)
    sar_rate_hz: int = Field(800, gt=0)
    navigation_rate_hz: int = Field(100, gt=0)
    pre_stabilization_s: float = Field(5.0, ge=0)
    acquisition_duration_s: float = Field(20.48, gt=0)
    duration_mode: Literal["fixed", "route_length"] = "fixed"
    duration_margin_s: float = Field(10.0, ge=0)
    integrator: Literal["RK4"] = "RK4"
    initial_position_error_sar_m: Vector3 = (0.0, 0.0, 0.0)
    initial_velocity_error_sar_mps: Vector3 = (0.0, 0.0, 0.0)
    initial_attitude_error_rpy_deg: Vector3 = (0.0, 0.0, 0.0)
    initial_body_rate_radps: Vector3 = (0.0, 0.0, 0.0)


class ControllerConfig(StrictModel):
    position_rate_hz: int = Field(20, gt=0)
    velocity_rate_hz: int = Field(50, gt=0)
    attitude_rate_hz: int = Field(200, gt=0)
    angular_rate_hz: int = Field(200, gt=0)
    kp_position_per_s: Vector3 = (0.6, 0.6, 0.8)
    kp_velocity_per_s: Vector3 = (1.5, 1.5, 2.0)
    ki_velocity_per_s2: Vector3 = (0.12, 0.12, 0.25)
    velocity_integral_limit_m: Vector3 = (2.0, 2.0, 2.0)
    kp_attitude_per_s: Vector3 = (4.0, 4.0, 3.0)
    kp_rate_per_s: Vector3 = (12.0, 12.0, 10.0)
    ki_rate_per_s2: Vector3 = (1.0, 1.0, 0.5)
    rate_integral_limit_rad: Vector3 = (0.4, 0.4, 0.4)
    max_speed_mps: float = Field(12.0, gt=0)
    max_horizontal_acceleration_mps2: float = Field(2.5, gt=0)
    max_vertical_acceleration_mps2: float = Field(2.0, gt=0)
    max_tilt_deg: float = Field(25.0, gt=0, lt=85)
    max_body_rate_radps: float = Field(1.5, gt=0)
    max_torque_Nm: Vector3 = (8.0, 8.0, 2.0)
    allocation_weights: tuple[float, float, float, float] = (1.0, 2.0, 2.0, 4.0)

    @model_validator(mode="after")
    def nonnegative(self):
        for name in type(self).model_fields:
            value = getattr(self, name)
            if isinstance(value, tuple) and min(value) < 0:
                raise ValueError(f"{name} must be nonnegative")
        if min(self.allocation_weights) <= 0:
            raise ValueError("Allocation weights must be positive")
        return self


class TrajectoryConfig(StrictModel):
    type: Literal["straight_constant_velocity", "waypoint", "time_series", "segmented_velocity"] = (
        "straight_constant_velocity"
    )
    speed_mps: float = Field(10.0, ge=0)
    route_length_m: float = Field(204.8, gt=0)
    speed_tolerance_fraction: float = Field(0.1, gt=0, lt=1)
    apc_height_m: float = Field(300.0, gt=0)
    yaw_deg: float = 0.0
    center_aperture: bool = True
    times_s: tuple[float, ...] = ()
    positions_sar_m: tuple[Vector3, ...] = ()
    velocities_sar_mps: tuple[Vector3, ...] = ()
    yaw_series_deg: tuple[float, ...] = ()

    @model_validator(mode="after")
    def check_table(self):
        if self.type != "straight_constant_velocity":
            if len(self.times_s) < 2 or np.any(np.diff(self.times_s) <= 0):
                raise ValueError("Strictly increasing trajectory times_s required")
            rows = self.velocities_sar_mps if self.type == "segmented_velocity" else self.positions_sar_m
            if len(rows) != len(self.times_s):
                raise ValueError("Trajectory table length mismatch")
            if self.velocities_sar_mps and len(self.velocities_sar_mps) != len(self.times_s):
                raise ValueError("Velocity table length mismatch")
            if self.yaw_series_deg and len(self.yaw_series_deg) != len(self.times_s):
                raise ValueError("Yaw table length mismatch")
        return self


class AntennaConfig(StrictModel):
    phase_center_frd_m: Vector3 = (0.0, 0.1, 0.2)
    look_angle_deg: float = Field(50.0, gt=0, lt=90)
    right_looking: Literal[True] = True
    squint_angle_deg: float = Field(0.0, gt=-90, lt=90)


class FactorConfig(StrictModel):
    enabled: bool = False
    injection_mode: Literal["command", "dynamics"] = "command"
    level: Literal["small", "medium", "large"] = "medium"
    profile: Literal["ou", "constant"] = "ou"
    start_time_s: float = 0.0
    end_time_s: float | None = None
    ramp_s: float = Field(0.5, gt=0)
    amplitude: float | None = Field(
        None, ge=0, description="OU stationary std, or constant magnitude; see unit"
    )
    unit: Literal["auto", "m", "m/s", "deg", "N", "N*m"] = "auto"
    random_seed: int | None = Field(None, ge=0)

    @model_validator(mode="after")
    def bounds(self):
        if self.end_time_s is not None and self.end_time_s <= self.start_time_s:
            raise ValueError("Disturbance end must exceed start")
        if self.injection_mode == "dynamics" and (self.amplitude is None or self.unit not in ("N", "N*m")):
            raise ValueError("Dynamics inputs require independently specified N or N*m amplitude")
        return self


class NavigationConfig(StrictModel):
    enabled: bool = False
    level: Literal["small", "medium", "large"] = "medium"
    correlation_time_s: float = Field(8.0, gt=0)
    bias_std_sar_m: Vector3 | None = None
    noise_std_sar_m: Vector3 = (0.01, 0.01, 0.02)
    random_seed: int | None = Field(None, ge=0)

    @model_validator(mode="after")
    def positive_std(self):
        if min(self.noise_std_sar_m) < 0 or (
            self.bias_std_sar_m is not None and min(self.bias_std_sar_m) < 0
        ):
            raise ValueError("Navigation standard deviation must be nonnegative")
        return self


class VelocityErrorConfig(StrictModel):
    enabled: bool = False
    correlation_time_s: float = Field(3.0, gt=0)
    bias_std_sar_mps: Vector3 = (0.02, 0.02, 0.04)
    noise_std_sar_mps: Vector3 = (0.0, 0.0, 0.0)
    random_seed: int | None = Field(None, ge=0)

    @model_validator(mode="after")
    def nonnegative_std(self):
        if min(self.bias_std_sar_mps) < 0 or min(self.noise_std_sar_mps) < 0:
            raise ValueError("Velocity error standard deviations must be nonnegative")
        return self


class AttitudeErrorConfig(StrictModel):
    enabled: bool = False
    correlation_time_s: float = Field(5.0, gt=0)
    bias_std_body_deg: Vector3 = (0.05, 0.05, 0.10)
    noise_std_body_deg: Vector3 = (0.0, 0.0, 0.0)
    random_seed: int | None = Field(None, ge=0)

    @model_validator(mode="after")
    def nonnegative_std(self):
        if min(self.bias_std_body_deg) < 0 or min(self.noise_std_body_deg) < 0:
            raise ValueError("Attitude error standard deviations must be nonnegative")
        return self


class ImuGnssConfig(StrictModel):
    """局部导航线性误差状态模型；随机游走单位按手册每√小时定义。"""

    enabled: bool = False
    accelerometer_bias_std_mps2: Vector3 = (0.000686, 0.000686, 0.000686)
    accelerometer_vrw_mps_sqrt_hour: Vector3 = (0.03, 0.03, 0.03)
    gyro_bias_std_degph: Vector3 = (3.0, 3.0, 3.0)
    gyro_arw_deg_sqrt_hour: Vector3 = (0.2, 0.2, 0.2)
    bias_correlation_time_s: float = Field(100.0, gt=0)
    gnss_enabled: bool = True
    gnss_rate_hz: float = Field(5.0, gt=0)
    gnss_position_std_ned_m: Vector3 = (0.007778, 0.007778, 0.016)
    gnss_velocity_std_ned_mps: Vector3 = (0.021213, 0.021213, 0.03)
    heading_enabled: bool = True
    heading_std_deg: float = Field(0.08, gt=0)
    initial_position_std_m: float = Field(0.02, ge=0)
    initial_velocity_std_mps: float = Field(0.03, ge=0)
    initial_tilt_std_deg: float = Field(0.018, ge=0)
    initial_heading_std_deg: float = Field(0.084, ge=0)
    random_seed: int = Field(0, ge=0)

    @model_validator(mode="after")
    def valid_noise(self):
        for name in ("accelerometer_bias_std_mps2", "accelerometer_vrw_mps_sqrt_hour",
                     "gyro_bias_std_degph", "gyro_arw_deg_sqrt_hour"):
            if min(getattr(self, name)) < 0:
                raise ValueError("IMU noise must be nonnegative")
        for name in ("gnss_position_std_ned_m", "gnss_velocity_std_ned_mps"):
            if min(getattr(self, name)) <= 0:
                raise ValueError("GNSS observation standard deviations must be positive")
        return self


class DisturbanceConfig(StrictModel):
    imu_gnss: ImuGnssConfig = Field(default_factory=ImuGnssConfig)
    all_enabled: bool = False
    tau_slow_s: float = Field(2.5, gt=0)
    tau_fast_s: float = Field(0.35, gt=0)
    slow_variance_fraction: float = Field(0.7, ge=0, le=1)
    input_rate_hz: int = Field(3200, gt=0)
    command_filter_cutoff_hz: float = Field(
        1.0, gt=0, description="Third-order Butterworth before command derivatives"
    )
    translation: dict[str, FactorConfig] = Field(
        default_factory=lambda: {k: FactorConfig() for k in ("x", "y", "z")}
    )
    attitude: dict[str, FactorConfig] = Field(
        default_factory=lambda: {k: FactorConfig() for k in ("roll", "pitch", "yaw")}
    )
    navigation_position_error: NavigationConfig = Field(default_factory=NavigationConfig)
    navigation_velocity_error: VelocityErrorConfig = Field(default_factory=VelocityErrorConfig)
    navigation_attitude_error: AttitudeErrorConfig = Field(default_factory=AttitudeErrorConfig)

    @model_validator(mode="after")
    def channels(self):
        if self.command_filter_cutoff_hz >= self.input_rate_hz / 2:
            raise ValueError("Command filter cutoff must be below input Nyquist")
        for mapping, keys, command_units, dynamic_unit in [
            (self.translation, ("x", "y", "z"), ("m", "m", "m"), "N"),
            (self.attitude, ("roll", "pitch", "yaw"), ("deg", "deg", "deg"), "N*m"),
        ]:
            if set(mapping) != set(keys):
                raise ValueError(f"Expected exactly channels {keys}")
            for key, unit in zip(keys, command_units):
                factor = mapping[key]
                expected = unit if factor.injection_mode == "command" else dynamic_unit
                if factor.unit not in ("auto", expected):
                    raise ValueError(f"{key} requires unit {expected}")
        return self


class GeometryConfig(StrictModel):
    translation: Literal["actual", "ideal"] = "actual"
    attitude: Literal["actual", "ideal"] = "actual"


class RuntimeConfig(StrictModel):
    max_workers: int = Field(0, ge=0, description="0 selects automatic safe concurrency")
    memory_fraction: float = Field(0.6, gt=0, le=0.9)
    reserve_memory_gb: float = Field(2.0, ge=0)
    backend: Literal["auto", "cpu", "rotorpy"] = "auto"
    blas_threads_per_worker: int = Field(1, ge=1)


class OutputConfig(StrictModel):
    save_mat: bool = True
    save_csv: bool = True
    save_plots: bool = True


class Config(StrictModel):
    base_seed: int = Field(20260604, ge=0)
    pass_seeds: tuple[int, ...] = (20260604, 20260605)
    nominal_baseline_sar_m: Vector3 = (0.0, 0.0, 0.0)
    vehicle: VehicleConfig = Field(default_factory=VehicleConfig)
    simulation: SimulationConfig = Field(default_factory=SimulationConfig)
    controller: ControllerConfig = Field(default_factory=ControllerConfig)
    trajectory: TrajectoryConfig = Field(default_factory=TrajectoryConfig)
    antenna: AntennaConfig = Field(default_factory=AntennaConfig)
    disturbances: DisturbanceConfig = Field(default_factory=DisturbanceConfig)
    geometry: GeometryConfig = Field(default_factory=GeometryConfig)
    runtime: RuntimeConfig = Field(default_factory=RuntimeConfig)
    output: OutputConfig = Field(default_factory=OutputConfig)

    @model_validator(mode="after")
    def scheduler(self):
        s = self.simulation
        if s.duration_mode == "route_length":
            if self.trajectory.type != "straight_constant_velocity":
                raise ValueError("Route-length duration requires a straight constant-velocity reference")
            if self.trajectory.speed_mps <= 0:
                raise ValueError("Route-length duration requires positive reference speed")
            grid_rate = reduce(math.gcd, (
                s.dynamics_rate_hz, s.sar_rate_hz, s.navigation_rate_hz,
                self.disturbances.input_rate_hz,
            ))
            required_s = self.trajectory.route_length_m / self.trajectory.speed_mps + s.duration_margin_s
            s.acquisition_duration_s = math.ceil(required_s * grid_rate - 1e-9) / grid_rate
        rates = [
            s.sar_rate_hz,
            s.navigation_rate_hz,
            self.controller.position_rate_hz,
            self.controller.velocity_rate_hz,
            self.controller.attitude_rate_hz,
            self.controller.angular_rate_hz,
        ]
        if any(s.dynamics_rate_hz % rate for rate in rates):
            raise ValueError("All update rates must divide dynamics_rate_hz")
        if self.disturbances.input_rate_hz < s.dynamics_rate_hz:
            raise ValueError("Common disturbance rate must be >= integration rate")
        for duration in [s.acquisition_duration_s, s.pre_stabilization_s]:
            for rate in [
                s.dynamics_rate_hz,
                s.sar_rate_hz,
                s.navigation_rate_hz,
                self.disturbances.input_rate_hz,
            ]:
                if abs(duration * rate - round(duration * rate)) > 1e-8:
                    raise ValueError("Durations must lie on all sample grids")
        if self.trajectory.speed_mps > self.controller.max_speed_mps:
            raise ValueError("Reference speed exceeds controller limit")
        if s.dynamics_rate_hz * self.vehicle.motor_time_constant_s < 2:
            raise ValueError("Motor time constant too short for fixed RK4 step")
        if any(seed < 0 for seed in self.pass_seeds) or len(set(self.pass_seeds)) != len(self.pass_seeds):
            raise ValueError("Pass seeds must be nonnegative and distinct")
        return self
