# 公开飞行数据的适用范围

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.1.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

本机没有 12 kg Hex-X 实测日志。下面数据仅用于验证数据处理、时间同步和参数辨识方法；不同机型的质量、惯量、电机推力与导航误差不能作为本项目标定值直接写入 `configs/`。

| 数据源 | 可用内容 | 本项目可验证的步骤 | 不可直接得出的结论 |
|---|---|---|---|
| [MUN-FRL 数据集](https://mun-frl-vil-dataset.readthedocs.io/en/latest/) | DJI M600 六旋翼；硬件同步 IMU、RTK/PPK 位置真值、LiDAR 与图像。官方说明 IMU 400 Hz、PPK 位置 5 Hz。 | IMU/位置时间对齐、导航误差计算、坐标与杆臂处理。 | 12 kg Hex-X 的惯量、电机系数、800 Hz SAR 真值。完整 ROS bag 单航段为数十 GB，未下载。 |
| [UAV-FD 数据集](https://zenodo.org/records/7648996) | ArduPilot 六旋翼实飞；IMU、命令、估计和实测单电机转速，含正常与故障航次。 | 电机响应与执行器故障分析的接口、统计方法。已下载并核验 `NO_FAULT1.mat`、`NO_FAULT5.mat`。 | 本机型电机参数或高精度外部位置真值。 |
| [NTU VIRAL 数据集](https://ntu-aris.github.io/ntu_viral_dataset/) | 作者公开的 FAST-LIO2 估计轨迹与独立 Leica 棱镜位置真值。 | 位置时间配对、杆臂补偿、坐标对齐、ATE；已在 `eee_01` 实飞航段完成，见[导航对照](public_navigation_validation.md)。 | 无独立姿态真值，不是本项目机体或飞控的实机精度。 |

选择公开数据时先核对许可证、时钟、坐标、传感器外参和目标机型。若未来取得本机型实飞日志，再依据相同的处理流程拟合质量/惯量、推力曲线、电机时间常数、IMU 偏置和导航误差，并将拟合区间、残差与原始日志一起保存。当前仿真参数保持经项目配置记录的工程初值。

## UAV-FD `NO_FAULT1.mat` 实际核验

原始文件保存在 `results/public_data/uav_fd/NO_FAULT1.mat`，校验结果与逐流统计保存在同目录 `validation.json`；重跑命令为 `uv run --no-sync python deployment/validate_public_uav_fd.py results/public_data/uav_fd/NO_FAULT1.mat --output results/public_data/uav_fd/validation.json`。实测 MD5 为 `065e36e917d0e6188a3b766e17a720c7`，与 Zenodo 公布值一致。`main.m` 为作者提供的字段提取示例，也保存在同目录。

实际记录约 72.11 s；IMU 26,162 条，采样间隔中位数 2.581 ms；飞控姿态和 EKF 各 6,734 条，GPS 359 条，六路 ESC 转速各 637～647 条。所查各流 `TimeUS` 严格递增，数值有限。按 `TimeUS` 最近邻且时间差不超过 30 ms，对全部 359 条三维定位 GPS 记录配对 EKF；最大配对时间差 8.31 ms，EKF 水平速度模长减 GPS `Spd` 的均值为 0.297 m/s、RMS 为 0.667 m/s。两者均来自机载系统，属于内部一致性检查，不是相对于独立真值的速度误差。

该航次 GPS 最大地速 2.158 m/s、海拔高度范围 84.67～89.41 m，与本项目的 10 m/s、300 m 相对采集高度及目标机型均不相同。因此没有进行逐点航迹比较，也没有把其电机或导航统计量写入 12 kg Hex-X 参数。

## 六路电机转速响应辨识与独立航次检查

另外下载并核验了 `NO_FAULT5.mat`，MD5 为 Zenodo 公布的 `22488aa7ed8fee78c0568149abc8efc0`。作者 `sync_zoh_exa.m` 明确使用 `RCOU` 的 C9～C14 对应六路 ESC；该示例也保存在同一目录。`deployment/fit_public_uav_fd_motors.py` 对 `NO_FAULT1` 每路转速高于 1500 RPM、PWM 高于 1200 µs 的样本拟合 `RPM = a·PWM + b`。前 70% 活跃时间用于选择 0～0.30 s 日志对齐偏移及拟合参数，后 30% 留出；再将六组参数原样用于 `NO_FAULT5`，不重新拟合。运行结果见 `results/public_data/uav_fd/motor_fit.json`。

| 电机 | 首航次留出集 RMSE (RPM) / R² | 第二航次 RMSE (RPM) / R² |
|---|---:|---:|
| ESC 0 | 302.5 / 0.93 | 286.4 / 0.82 |
| ESC 1 | 173.3 / 0.56 | 274.7 / 0.68 |
| ESC 2 | 288.2 / 0.92 | 263.0 / 0.81 |
| ESC 3 | 213.1 / 0.79 | 237.3 / 0.62 |
| ESC 4 | 259.6 / 0.49 | 285.3 / 0.55 |
| ESC 5 | 355.8 / 0.88 | 277.8 / 0.79 |

这证明该数据支持有误差边界的本机型 PWM→RPM 经验关系，不能证明推力/扭矩模型正确。对齐偏移为 0.16～0.24 s，混合了 9.3 Hz ESC 记录、通信和电机响应，不能当作电机时间常数。第二航次的六路预测平均误差均为负，提示存在航次间偏移。不同机体、ESC 和螺旋桨不可直接共用系数；本项目参数未修改。
