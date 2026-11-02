import numpy as np


def motor_derivative(
    speed_radps: np.ndarray, command_radps: np.ndarray, tau_s: float, maximum_radps: float
) -> np.ndarray:
    """一阶转速动态；状态积分由统一 RK4 完成，命令限幅 [0,max]。"""
    return (np.clip(command_radps, 0, maximum_radps) - speed_radps) / tau_s
