"""局部NED、15维线性化GNSS/INS误差状态仿真，不复现厂商固件。

状态为估计减真值的[p,v,导航系小姿态角,加计残余偏置,陀螺残余偏置]。
忽略地球自转/曲率；适用短距离、小姿态误差。GNSS与航向观测使用独立随机流。
"""

import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def transition(f, dt):
    """三阶状态转移；步长需远小于偏置相关时间及运动变化时间。"""
    a = f * dt
    return np.eye(15) + a + a @ a / 2 + a @ a @ a / 6


def fuse(error, covariance, h, variance, measurement_noise):
    """合成误差状态Kalman校正；Joseph形式保持协方差半正定。"""
    gain = np.linalg.solve(h @ covariance @ h.T + np.diag(variance), h @ covariance).T
    error = error - gain @ (h @ error - measurement_noise)
    residual = np.eye(15) - gain @ h
    covariance = residual @ covariance @ residual.T + (gain * variance) @ gain.T
    return error, (covariance + covariance.T) / 2


def simulate_errors(config, truth, rotations, pass_seed):
    """返回导航率误差和可审计诊断。噪声密度由随机游走/60换算为每√秒。

    连续白噪声按积分协方差离散，不把导航输出RMS直接叠加到结果。
    GNSS时刻量化到最近的导航采样点；不插入虚构高频GNSS更新。
    """
    cfg = config.disturbances.imu_gnss
    t = truth['time_s']
    if len(t) < 2:
        raise ValueError('GNSS/INS requires at least two navigation samples')
    dt = float(t[1] - t[0])
    if dt / cfg.bias_correlation_time_s > 0.05:
        raise ValueError('Navigation step must be at most 5% of bias correlation time')
    if cfg.gnss_rate_hz > 1 / dt:
        raise ValueError('GNSS rate cannot exceed navigation rate')
    seeds = np.random.SeedSequence([config.base_seed, pass_seed, 91, cfg.random_seed]).spawn(3)
    process_rng, observation_rng, initial_rng = [np.random.default_rng(s) for s in seeds]
    ba = np.asarray(cfg.accelerometer_bias_std_mps2)
    bg = np.deg2rad(cfg.gyro_bias_std_degph) / 3600
    qa = (np.asarray(cfg.accelerometer_vrw_mps_sqrt_hour) / 60)**2
    qg = (np.deg2rad(cfg.gyro_arw_deg_sqrt_hour) / 60)**2
    tau = cfg.bias_correlation_time_s
    initial = np.r_[np.full(3, cfg.initial_position_std_m),
                    np.full(3, cfg.initial_velocity_std_mps),
                    np.deg2rad([cfg.initial_tilt_std_deg]*2 + [cfg.initial_heading_std_deg]), ba, bg]
    covariance = np.diag(initial**2)
    error = initial_rng.normal(size=15) * initial
    errors = np.empty((len(t), 15))
    sigmas = np.empty_like(errors)
    updates = np.zeros(len(t), dtype=bool)
    h = np.zeros((7 if cfg.heading_enabled else 6, 15))
    h[:6, :6] = np.eye(6)
    variance = np.r_[np.asarray(cfg.gnss_position_std_ned_m)**2,
                     np.asarray(cfg.gnss_velocity_std_ned_mps)**2]
    if cfg.heading_enabled:
        h[6, 8] = 1
        variance = np.r_[variance, np.deg2rad(cfg.heading_std_deg)**2]
    update_ids = set(np.rint(np.arange(0, t[-1] - t[0] + dt/2, 1/cfg.gnss_rate_hz)/dt).astype(int))
    gravity = np.array([0., 0., config.vehicle.gravity_mps2])
    # 三点Gauss积分离散Q，包含位置/速度/姿态的过程噪声交叉项。
    nodes, weights = np.polynomial.legendre.leggauss(3)
    for k in range(len(t)):
        if k:
            r = rotations[k-1]
            specific_force_n = truth['acceleration_N_mps2'][k-1] - gravity
            f = np.zeros((15, 15))
            f[:3, 3:6] = np.eye(3)
            f[3:6, 6:9] = -skew(specific_force_n)
            f[3:6, 9:12] = r
            f[6:9, 12:15] = r
            f[9:15, 9:15] = -np.eye(6)/tau
            spectral = np.zeros((15, 15))
            spectral[3:6, 3:6] = (r * qa) @ r.T
            spectral[6:9, 6:9] = (r * qg) @ r.T
            spectral[9:15, 9:15] = np.diag(np.r_[ba**2, bg**2]*2/tau)
            phi = transition(f, dt)
            q = np.zeros((15, 15))
            for node, weight in zip(nodes, weights):
                a = transition(f, (node + 1)*dt/2)
                q += weight*dt/2 * (a @ spectral @ a.T)
            eigenvalues, vectors = np.linalg.eigh((q + q.T)/2)
            process = vectors @ (np.sqrt(np.maximum(eigenvalues, 0))*process_rng.normal(size=15))
            error = phi @ error + process
            covariance = phi @ covariance @ phi.T + q
        if cfg.gnss_enabled and k in update_ids:
            error, covariance = fuse(error, covariance, h, variance,
                                     observation_rng.normal(size=len(variance))*np.sqrt(variance))
            updates[k] = True
        errors[k] = error
        sigmas[k] = np.sqrt(np.maximum(np.diag(covariance), 0))
    if np.max(np.linalg.norm(errors[:, 6:9], axis=1)) > np.deg2rad(5):
        raise ValueError('INS attitude error exceeds 5 degrees; linear model validity exceeded')
    # 左乘导航系误差转换为供接口使用的右乘机体系旋转向量。
    body_angle = np.einsum('nji,nj->ni', rotations, errors[:, 6:9])
    estimated_rotation = Rotation.from_rotvec(errors[:, 6:9]).as_matrix() @ rotations
    return errors, body_angle, estimated_rotation, sigmas, updates
