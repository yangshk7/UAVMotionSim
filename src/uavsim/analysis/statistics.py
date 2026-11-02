from pathlib import Path

import numpy as np
from scipy.signal import correlate, welch

from uavsim.disturbances.ou_process import dual_ou
from uavsim.visualization.chinese_style import use_chinese_font


def validate_ou_statistics(root: str | Path, seed: int = 937) -> dict:
    """20,000 s 固定种子统计验证；未用于宣称机体扰动已实测校准。"""
    import matplotlib.pyplot as plt

    use_chinese_font()

    count = 400000
    dt = 0.05
    slow, fast, fraction = 2.5, 0.35, 0.7
    signal = dual_ou(count, dt, slow, fast, fraction, np.random.SeedSequence(seed))
    other = dual_ou(count, dt, slow, fast, fraction, np.random.SeedSequence(seed + 1))
    centered = signal - signal.mean()
    lags = np.arange(201)
    covariance = correlate(centered, centered, mode="full", method="fft")[count - 1 : count + 200] / (
        count - lags
    )
    acf = covariance / covariance[0]
    expected = fraction * np.exp(-lags * dt / slow) + (1 - fraction) * np.exp(-lags * dt / fast)
    freq, psd = welch(signal, fs=1 / dt, nperseg=8192)
    # 精确离散 OU 的单边 PSD，避免把连续 Lorentzian 高频尾部当作离散参考。
    theory = np.zeros_like(freq)
    for weight, tau in [(fraction, slow), (1 - fraction, fast)]:
        a = np.exp(-dt / tau)
        theory += 2 * dt * weight * (1 - a * a) / (1 + a * a - 2 * a * np.cos(2 * np.pi * freq * dt))
    theory[[0, -1]] *= 0.5
    report = {
        "seed": seed,
        "samples": count,
        "duration_s": count * dt,
        "dt_s": dt,
        "tau_slow_s": slow,
        "tau_fast_s": fast,
        "slow_variance_fraction": fraction,
        "mean": float(signal.mean()),
        "variance": float(signal.var()),
        "acf_max_error_0_to_10s": float(np.max(np.abs(acf - expected))),
        "independent_stream_correlation": float(np.corrcoef(signal, other)[0, 1]),
        "psd_integrated_variance": float(np.trapezoid(psd, freq)),
        "limits": {
            "absolute_mean": 0.05,
            "variance_absolute_error": 0.08,
            "acf_max_error": 0.06,
            "absolute_cross_correlation": 0.08,
        },
    }
    report["passed"] = bool(
        abs(report["mean"]) < 0.05
        and abs(report["variance"] - 1) < 0.08
        and report["acf_max_error_0_to_10s"] < 0.06
        and abs(report["independent_stream_correlation"]) < 0.08
        and abs(report["psd_integrated_variance"] - 1) < 0.1
    )
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), layout="constrained")
    axes[0].plot(lags * dt, acf, label="长序列测量值")
    axes[0].plot(lags * dt, expected, "--", label="双 OU 理论自相关")
    axes[0].set(xlabel="时间间隔 (s)", ylabel="自相关")
    axes[1].loglog(freq[1:], psd[1:], label="Welch 功率谱密度")
    axes[1].loglog(freq[1:], theory[1:], "--", label="离散 OU 理论功率谱密度")
    axes[1].set(xlabel="频率 (Hz)", ylabel="功率谱密度 (单位²/Hz)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.savefig(Path(root) / "ou_statistics.png", dpi=160)
    plt.close(fig)
    return report
