# 实施计划

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.0.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

源规范：`AirborneSAR_UAVMotionSim_Codex_Technical_Roadmap_V1.0.md`。

工程直接建立在 `UAV_Motion_Sim` 内，不再增加重复的工程根目录。所有新增路径使用英文。

用户已确认采用规范中的独立基准：800 Hz、APC 高度 300 m、10 m/s、16385 脉冲。
原有 SAR 工程只读，禁止引用旧的 amplitudeXYZ_m 作为扰动参数。

用户后续要求将 100 Hz 导航式位姿作为主要输出，并增加速度和姿态估计误差。这覆盖原规范仅位置误差的限制。用户已确认速度/姿态候选标准差和相关时间，详见 data_interface.md；真实刚体量仍严格隔离。

按 P0 环境 → P1 坐标/几何 → P2 动力学 → P3 飞控 → P4 扰动 → P5 导航/双航次 → P6 SAR 接口 → P7 科研验证顺序实现并测试。
P8 是独立外部环境验证，不能用本项目自身测试冒充 SITL/Simscape 验证。

资源优化：检测 CPU、物理内存及可用内存、GPU 型号与显存；以可用内存和 CPU 配额计算航次进程数，限制 BLAS 线程，预分配 float64 数组，避免在 RK4 内采样噪声。19 维有反馈状态积分默认选择 CPU；仅检测到 GPU 不等于已使用 GPU。

各阶段实际测试与剩余问题记入 `docs/stage_reports.md` 和 `docs/validation_report.md`。
