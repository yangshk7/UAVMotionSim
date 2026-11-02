"""按物理扰动状态寻找目标长度航段；速度带始终保留为观测统计。"""

import numpy as np

from uavsim.sar_adapter.pulse_sync import sample_indices


def evaluate_route_quality(pass_result: dict, config) -> dict:
    """检查原生真值的速度和沿 X 前进距离；受扰动时允许速度波动，单位 s、m、m/s。"""
    trajectory = config.trajectory
    indices = sample_indices(config)["acquisition"]
    truth = pass_result["truth_full"]
    x = np.asarray(truth["position_sar_m"])[indices, 0]
    velocity = np.asarray(truth["velocity_N_mps"])[indices]
    speed = np.linalg.norm(velocity, axis=1)
    target = trajectory.speed_mps
    tolerance = target * trajectory.speed_tolerance_fraction
    speed_in_band = np.isfinite(speed) & (speed >= target - tolerance) & (speed <= target + tolerance)
    physical_disturbance_active = config.disturbances.all_enabled or any(
        factor.enabled and factor.amplitude != 0
        for group in (config.disturbances.translation, config.disturbances.attitude)
        for factor in group.values()
    )
    speed_band_enforced = not physical_disturbance_active
    valid = speed_in_band if speed_band_enforced else np.isfinite(speed)
    time = np.arange(len(indices), dtype=np.float64) / config.simulation.dynamics_rate_hz
    navigation_stride = config.simulation.dynamics_rate_hz // config.simulation.navigation_rate_hz
    longest_distance = 0.0
    longest_speed_band_distance = 0.0
    selected = None
    start = None

    def finish(end: int) -> None:
        nonlocal longest_distance, selected
        if start is None or end <= start:
            return
        distance = float(x[end] - x[start])
        longest_distance = max(longest_distance, distance)
        if selected is None and distance >= trajectory.route_length_m:
            route_length = trajectory.route_length_m
            first_common = ((start + navigation_stride - 1) // navigation_stride) * navigation_stride
            last_common = (end // navigation_stride) * navigation_stride
            numerical_tolerance_m = 1e-9
            if last_common <= first_common or x[last_common] - x[first_common] < route_length - numerical_tolerance_m:
                return
            common = np.arange(first_common, last_common + 1, navigation_stride)
            centered_x = (x[first_common] + x[last_common] - route_length) / 2
            candidate = int(np.searchsorted(x[common], centered_x))
            latest_start = int(np.searchsorted(x[common], x[last_common] - route_length + numerical_tolerance_m,
                                               side="right")) - 1
            upper = min(max(candidate, 0), latest_start)
            lower = max(upper - 1, 0)
            crop_start = int(common[min((lower, upper), key=lambda j: abs(x[common[j]] - centered_x))])
            stop_index = int(np.searchsorted(x[common], x[crop_start] + route_length - numerical_tolerance_m))
            stop = int(common[stop_index])
            selected = {
                "start_time_s": float(time[crop_start]),
                "end_time_s": float(time[stop]),
                "start_x_sar_m": float(x[crop_start]),
                "end_x_sar_m": float(x[stop]),
                "forward_distance_m": float(x[stop] - x[crop_start]),
                "speed_min_mps": float(speed[crop_start : stop + 1].min()),
                "speed_max_mps": float(speed[crop_start : stop + 1].max()),
            }

    speed_band_start = None
    for index in range(len(speed)):
        if not speed_in_band[index] or (index and x[index] <= x[index - 1]):
            if speed_band_start is not None:
                longest_speed_band_distance = max(
                    longest_speed_band_distance, float(x[index - 1] - x[speed_band_start])
                )
            speed_band_start = None
        if speed_in_band[index] and speed_band_start is None:
            speed_band_start = index
        if not valid[index]:
            finish(index - 1)
            start = None
        elif start is None:
            start = index
        elif x[index] <= x[index - 1]:
            finish(index - 1)
            start = index
    finish(len(speed) - 1)
    if speed_band_start is not None:
        longest_speed_band_distance = max(
            longest_speed_band_distance, float(x[-1] - x[speed_band_start])
        )
    return {
        "passed": selected is not None,
        "assessment_mode": "disturbed_physical_coverage" if physical_disturbance_active else "nominal_speed_band",
        "speed_band_enforced": speed_band_enforced,
        "target_route_length_m": trajectory.route_length_m,
        "target_speed_mps": target,
        "speed_tolerance_fraction": trajectory.speed_tolerance_fraction,
        "speed_allowed_min_mps": target - tolerance,
        "speed_allowed_max_mps": target + tolerance,
        "output_duration_s": config.simulation.acquisition_duration_s,
        "margin_requested_s": config.simulation.duration_margin_s,
        "full_speed_min_mps": float(speed.min()),
        "full_speed_max_mps": float(speed.max()),
        "full_forward_distance_m": float(x[-1] - x[0]),
        "fraction_within_speed_band": float(np.mean(speed_in_band)),
        "longest_contiguous_valid_distance_m": longest_distance,
        "longest_contiguous_speed_band_distance_m": longest_speed_band_distance,
        "crop_window": selected,
    }
