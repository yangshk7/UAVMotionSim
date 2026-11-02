import numpy as np

from .equations import RigidBodyDynamics


def rk4_step(
    dynamics: RigidBodyDynamics,
    state: np.ndarray,
    command: np.ndarray,
    dt_s: float,
    force_samples_b: tuple[np.ndarray, np.ndarray, np.ndarray],
    torque_samples_b: tuple[np.ndarray, np.ndarray, np.ndarray],
) -> np.ndarray:
    """固定步长 RK4；外部确定性输入分别在起点/中点/终点读取。命令零阶保持。"""
    rhs = dynamics.derivative
    command = np.clip(command, 0, dynamics.vehicle.max_speed_radps)
    f0, fm, f1 = force_samples_b
    t0, tm, t1 = torque_samples_b
    k1 = rhs(state, command, f0, t0)
    k2 = rhs(state + 0.5 * dt_s * k1, command, fm, tm)
    k3 = rhs(state + 0.5 * dt_s * k2, command, fm, tm)
    k4 = rhs(state + dt_s * k3, command, f1, t1)
    result = state + (dt_s / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    result[6:10] /= np.linalg.norm(result[6:10])
    result[13:] = np.clip(result[13:], 0, dynamics.vehicle.max_speed_radps)
    if not np.all(np.isfinite(result)):
        raise FloatingPointError("Nonfinite integrated state")
    return result
