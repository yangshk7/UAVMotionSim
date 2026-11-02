# 外部飞控真值与导航的同钟对齐

本次直接复用已通过完整航线的 ArduPilot SITL 与 PX4/Gazebo 原始日志，没有修改飞行数据。对齐只处理**仿真时间**，不做 SAR 回波/成像，也不把仿真误差当成实机精度。

## PX4/Gazebo：已验证精确公共仿真时钟

PX4 v1.15.4 的 Gazebo 桥接代码从 Gazebo `/clock` 读取 `clock.sim()`，并调用 `px4_clock_settime(CLOCK_MONOTONIC, ...)` 设置 PX4 内部时钟。ULog `vehicle_local_position.timestamp_sample`、`vehicle_attitude.timestamp_sample` 是这个原始时钟的微秒值；不使用经 ROS/DDS 转换成 Unix 时间的 `timestamp_s`，也不使用主机接收时间来决定物理时刻。该机制与 [PX4 v1.15 Gazebo 时间说明](https://docs.px4.io/v1.15/en/sim_gazebo_gz/)一致，原始导航字段见 [VehicleLocalPosition 文档](https://docs.px4.io/v1.15/en/msg_docs/VehicleLocalPosition)。

对已保存的 20.48 s 航线，`deployment/extract_px4_ulog.py` 从 ULog 导出未变换时间戳，`deployment/align_px4_sim_clock.py` 把 Gazebo 1000 Hz 真值与 PX4 导航放到同一仿真时间轴。用独立的 ULog `vehicle_local_position_groundtruth` 与 Gazebo 位姿交叉验证：**1025 个共同时间戳全部精确匹配**；SAR X/Y/Z 位置最大绝对差分别为 **7.38 µm / 0.058 µm / 29.84 µm**。独立姿态真值 **1205 个共同时间戳**匹配，最大旋转角差约 **0.0000012°**。这同时检查了 ENU/FLU 到 SAR/NED/FRD 的坐标变换。

输出位于 `results/xtdrone_route/xtdrone_20261102_095933_737941/clock_sync/`：`truth_1000hz.csv` 为 **20481 个 Gazebo 原生位置/姿态样本**；速度由 1 ms 位置中心差分得到，明确标为导出计算值。`navigation_100hz.csv` 为 **2049 个同钟导航位姿样本**，由 ULog 原生位置和姿态按各自 `timestamp_sample` 插值；最大插值跨度分别 **8 ms / 4 ms**。`truth_at_navigation_100hz.csv` 与导航样本的时刻一一对应，**2049/2049 均直接落在 Gazebo 原生 1 ms 网格**，无真值时间插值。原始证据与门槛保存在 `alignment.json`、`pair_validation.json`。

## ArduPilot SITL：已验证共同启动/仿真时间

原先的 MAVLink `SIMSTATE` 消息不带自身时间戳，主机接收时间受批量发送影响，不能用于严格对时。此处改用同一次飞行的 DataFlash 日志：`SIM2.TimeUS` 保存物理位置/速度，`SIM.TimeUS` 保存真值姿态，`XKF1.TimeUS` 保存独立 EKF 位置；MAVLink 导航消息使用 `time_boot_ms`。`deployment/extract_sitl_dataflash.py` 导出这些原始时间戳，`deployment/align_sitl_sim_clock.py` 用 **1026 个采集段点**将 `XKF1` 按其 `TimeUS` 插值到 MAVLink `time_boot_ms`，三轴差 RMS 为 **0.00672 / 0.00011 / 0.000018 m**，最大差 **0.0123 / 0.00065 / 0.00010 m**。该独立状态对照支持两类日志共用 SITL 启动时间轴；差值也包含低频日志插值与估计记录差异，不能当作飞行精度。

最终输出位于 `results/frequency_probe/sitl_route/sitl_20261102_095735_133880/clock_sync_final/`。`truth_native.csv` 保留 **8198 个 SIM2 原生样本**（含 100 Hz 窗口前后各一个边界样本），中位间隔 **2.499 ms**、最大 **3.332 ms**，有效频率约 **400 Hz**。其真值姿态来自约 10 Hz `SIM` 四元数插值，**不能称为独立 400 Hz 姿态观测**。`navigation_100hz.csv` 为 **2050 个同钟样本**，由约 50 Hz MAVLink 位置/姿态插值，最大跨度均为 **21 ms**。`truth_at_navigation_100hz.csv` 给出逐点对应的真值，最多跨 **3.332 ms** 的原生真值间隔插值；其中 2 点恰好落在原生时戳。

## 配对误差与适用边界

`deployment/validate_clock_aligned.py` 在导航的 100 Hz 时刻输出对应真值和 `pair_validation.json`，拒绝超出真值覆盖、非 100 Hz 网格或过大的真值缺口。两套平台的位置差仍包括导航原点偏移、估计器内部时间滞后和模型差异；**同钟不等于导航零延迟或实机精度已标定**。PX4 原始同钟位置差 RMS 为约 **1.010 / 0.043 / 1.166 m**，SITL 为约 **0.307 / 0.016 / 0.103 m**，这些不应直接当成导航器独立误差指标。

两套输出是机体参考点。目标 12 kg Hex-X 的 CG→SAR APC 杆臂和外部导航共同原点尚无实测标定，因此此处没有伪造 APC 位置，也没有把数据标记为已通过 `MotionSegment` 的 SAR 就绪接口。用户后续若只需机体航迹和同钟导航位姿，可直接使用本次 CSV；用于天线相位中心和 SAR 脉冲级建模前，仍需给出杆臂及外参。未经实测的估计器延迟可以作为误差现象保留，不能靠移动数据时间戳掩盖。

后续已按两份既有仿真规范共同给出的 **FRD `[0,0.1,0.2] m` 工程安装值**，为这两套同钟机体轨迹另生成 APC 真值与导航估计派生 CSV；原始同钟文件和本节 `alignment.json` 保持不变。派生文件的位置、上位机参数入口和几何验证见[仿真杆臂说明](lever_arm_simulation.md)。这些派生 APC 是给定仿真安装值下的计算结果，仍不等于实机杆臂测量标定。

复算脚本都采用英文文件名和独立结果目录，拒绝覆盖已有输出。PX4 的 ULog 解析器使用已部署 WSL `px4_venv` 中的 `pyulog`；SITL 的 DataFlash 解析器使用已部署 SITL 环境中的 `pymavlink`；对齐与配对使用项目现有 `uv` 环境。`motion_source_audit_aligned.json` 汇总最新同钟状态，旧 `motion_source_audit.json` 保留对时前的审计记录。
