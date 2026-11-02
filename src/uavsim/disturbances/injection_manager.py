import numpy as np
from scipy.interpolate import CubicSpline
from scipy.signal import butter, sosfilt, sosfilt_zi

from uavsim.frames.coordinate import polar_to_sar
from uavsim.trajectory.smooth_transition import quintic_blend

from .attitude import attitude_command
from .ou_process import dual_ou
from .presets import factor_scale
from .translation import translation_command


class DisturbanceInputs:
    """预生成公共网格随机输入；C2 样条供不同积分频率读取同一连续实现。"""

    def __init__(self, config, pass_seed: int):
        self.config = config
        d = config.disturbances
        s = config.simulation
        rate = d.input_rate_hz
        count = round((s.pre_stabilization_s + s.acquisition_duration_s) * rate) + 1
        self.time_s = np.arange(count) / rate - s.pre_stabilization_s
        self.commands = {"translation": [None] * 3, "attitude": [None] * 3}
        self.force = [None] * 3
        self.torque = [None] * 3
        self.channels = {}
        for group, names in [("translation", ("x", "y", "z")), ("attitude", ("roll", "pitch", "yaw"))]:
            for axis, name in enumerate(names):
                factor = getattr(d, group)[name]
                if not (factor.enabled or d.all_enabled):
                    continue
                scale, unit = factor_scale(factor, group, axis)
                seed = np.random.SeedSequence(
                    [
                        pass_seed,
                        config.base_seed,
                        1 if group == "translation" else 2,
                        axis,
                        0 if factor.random_seed is None else factor.random_seed,
                    ]
                )
                raw = (
                    dual_ou(count, 1 / rate, d.tau_slow_s, d.tau_fast_s, d.slow_variance_fraction, seed)
                    if factor.profile == "ou"
                    else np.ones(count)
                )
                if factor.injection_mode == "command":
                    sos = butter(3, d.command_filter_cutoff_hz, fs=rate, output="sos")
                    raw = sosfilt(sos, raw, zi=sosfilt_zi(sos) * raw[0])[0]
                end = s.acquisition_duration_s if factor.end_time_s is None else factor.end_time_s
                # 五次窗避免开启/关闭时位置和姿态跳变；端点处一二阶导数连续。
                window = quintic_blend((self.time_s - factor.start_time_s) / factor.ramp_s)
                window *= quintic_blend((end - self.time_s) / factor.ramp_s)
                values = scale * raw * window
                if unit == "deg":
                    values = np.deg2rad(values)
                    unit = "rad"
                spline = CubicSpline(self.time_s, values)
                target = (
                    self.commands[group]
                    if factor.injection_mode == "command"
                    else self.force
                    if group == "translation"
                    else self.torque
                )
                target[axis] = spline
                self.channels[name] = {"unit": unit, "mode": factor.injection_mode, "values": values}

    def command_arrays(self, time_s: np.ndarray) -> dict:
        p, v, a = translation_command(self.commands["translation"], time_s)
        attitude, rates = attitude_command(self.commands["attitude"], time_s)
        return {
            "position_offset_N_m": polar_to_sar(p),
            "velocity_offset_N_mps": polar_to_sar(v),
            "acceleration_offset_N_mps2": polar_to_sar(a),
            "attitude_offset_rad": attitude,
            "rate_offset_radps": rates,
        }

    def dynamics_arrays(self, time_s: np.ndarray) -> tuple:
        def evaluate(splines):
            return np.column_stack(
                [np.zeros(len(time_s)) if spline is None else spline(time_s) for spline in splines]
            )

        return evaluate(self.force), evaluate(self.torque)

    def export(self, time_s: np.ndarray) -> dict:
        output = {"time_s": time_s}
        for name, data in self.channels.items():
            group = "translation" if name in self.config.disturbances.translation else "attitude"
            axis = (("x", "y", "z") if group == "translation" else ("roll", "pitch", "yaw")).index(name)
            target = (
                self.commands[group]
                if data["mode"] == "command"
                else self.force
                if group == "translation"
                else self.torque
            )
            output[name] = {"unit": data["unit"], "mode": data["mode"], "values": target[axis](time_s)}
        force, torque = self.dynamics_arrays(time_s)
        output["force_B_N"] = force
        output["torque_B_Nm"] = torque
        return output
