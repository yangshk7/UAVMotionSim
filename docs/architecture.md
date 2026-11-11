# 架构与职责

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.1.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

`config` 用 Pydantic 校验 YAML，可用相对 `extends` 合并实验覆盖。`trajectory` 生成名义 APC 及对应质心参考。`controllers` 按整数步调度位置、速度、SO(3) 姿态与角速度环。

`vehicle` 定义六旋翼几何、推力、执行器和分配；`dynamics` 用固定步长 RK4 积分 19 维刚体状态。控制器只返回电机指令，不改状态。扰动在仿真前生成，RHS 中没有随机采样或控制器更新。

OpenSource 版本的 `runtime.backend=rotorpy` 通过 `dynamics/rotorpy_hex.py` 调用 RotorPy 3.0.0 的多旋翼 RHS；本项目保留六旋翼受限分配、RK4 时间网格与导航/SAR 导出。`cpu` 和 `auto` 保留原自实现 RHS。集成范围和未部署阶段见根目录 `OPEN_SOURCE_INTEGRATION.md`。

`disturbances` 负责独立通道 OU、指令平滑及外力/力矩。`navigation` 在独立 100 Hz 时轴上构造位置、速度及 SO(3) 姿态估计误差与导航式位姿记录，不能反馈到控制器。`experiments` 负责双航次与单独标记的几何解耦实验。`sar_adapter` 计算 APC、波束、精确脉冲索引并导出数据。

`motion` 为独立交付接口：在原生积分网格导出机体与 APC 物理真值，在 100 Hz 网格导出导航位姿；两路共享精确时间原点，并支持在共有样本时刻裁剪。`analysis.route_quality` 对设定速度与目标长度做逐点硬检查，只报告合格窗口，不修饰原始数据。外部飞控的接收时钟近邻记录保留在 `deployment`；后续从 PX4 ULog 与 SITL DataFlash 构造并验证了同仿真时钟的机体轨迹。其 APC 外参仍未标定，故未构造 SAR 就绪的 `MotionSegment`。

Reference、Truth、Navigation、SAR Experiment 分开保存。改变导航误差不会改变 truth_full 的任何运动字段。不同航次由独立种子驱动，串行/并行不改变运动结果。

`runtime` 探测硬件并规划资源，`analysis` 进行运动学残差、OU 统计和数值收敛，`visualization` 绘制实际计算结果。Git 管理源代码，uv.lock 管理跨平台依赖解析；requirements.lock 记录当前部署环境版本。

公共接口：`load_config(path)`、`simulate_pass(config, pass_seed, pass_index)`、`run_experiment(config, passes)`、`export_experiment(result, directory)`。重要函数注明了坐标与单位。
