# 外部验证状态

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.1.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

**OpenSource 版本 2026-11-01 更新：** ArduPilot SITL、XTDrone2、ROS2、Gazebo、PX4 已部署；两个平台的短时起降和 300 m / 10 m/s / 20.48 s 采集航段分别通过，且自动落地停桨。PX4 新增 Gazebo 原始位姿采集和接收时钟近邻对照。原点偏移与处理延迟尚未校准，不据此声称导航精度或 800 Hz SAR 真值。版本、入口与原始记录见 [部署说明](../deployment/README.md)。

**2026-11-02 补充：** 同次飞行的 PX4 ULog 和 SITL DataFlash 提供原始仿真时戳，已与外部物理真值、导航共同时间轴对齐，并做独立状态交叉检查；结果见[时钟对齐报告](external_clock_alignment.md)。原点偏移和估计器内部滞后仍未标定。

MATLAB：已在本机实际加载两航次完整 MAT，核验 800 Hz 时间轴、APC、波束、导航采样，并调用现有 L 波段 `simulate_echo` 生成 4096×16385 单点回波。实际结果保存在对应运行目录的 matlab_validation.json，汇总见 validation_report.md。

MAT 文件的 Python 整数加载为 MATLAB int64；读取侧计算时间时显式转 double，避免整数除法量化。修复后 Python/MATLAB 时间轴逐点精确相同，没有放宽时间同步检查。

旧 `lband.beam_weight` 采用固定名义波束，不消费新输出的随姿态变化波束。适配器保留该字段，现有回波验证覆盖运动位置和斜距。当前没有修改外部 SAR 文件；将姿态波束真正接入照射权重仍需要针对原工程的后续适配。

ArduPilot 使用独立 Ubuntu-24.04 WSL2 中的官方 ArduCopter 4.7.2 SITL 二进制及自定义 Hex-X 参数；已经实际执行完整采集航段。其内置推力和电池模型与 RotorPy 不完全一致，当前仍不能称为逐参数同模型精度验证。Gazebo/PX4 的原始物理位姿和飞控导航解已分别保存，需完成统一原点、姿态和时间延迟检查后再比较准确度。

Simscape/实机：未进行参数匹配和交叉验证。所有物理参数与 OU 候选值仍为初始假设。
