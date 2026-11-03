# 主要输出：导航式位姿记录

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.0.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

> 当前默认已升级为加速度计/陀螺仪误差传播与GNSS校正，旧独立PVA误差及80%/20%拆分仅保留历史兼容；以 [GNSS/INS模型说明](gnss_ins_model.md) 为准。

用户后续明确要求输出类似导航解的位姿信息，并授权增加**速度与姿态估计误差**。这覆盖源规范中“仅位置误差”的原限制。当前输出采用局部坐标，不捏造经纬度、GPS 周或 UTC 绝对起始历元。

每航次主要文件：

| 文件 | 频率 | 内容 |
|---|---|---|
| navigation_pass1.csv / navigation_pass2.csv | 100 Hz | 含位置、速度、姿态误差的导航式解 |
| navigation_truth_pass1.csv / navigation_truth_pass2.csv | 100 Hz | 对应时刻的真实位姿，用于比较 |
| navigation_pass1.mat / navigation_pass2.mat | 100 Hz | navigation、navigation_truth 和 metadata，便于单独加载 |
| uav_motion_pass1.mat / uav_motion_pass2.mat | 3200/800/100 Hz | 完整刚体真值、导航、参考、扰动和 SAR 几何 |
| uav_motion_pass1.csv / uav_motion_pass2.csv | 800 Hz | SAR 脉冲状态 |
| uav_motion_summary.mat、config_snapshot.yaml、metadata.json | — | 配置、种子、软件版本和资源记录 |

CSV 主字段如下。所有姿态约定均写入 MAT 的说明字段；每行具有唯一 `time_s`。

| 字段 | 含义 / 单位 |
|---|---|
| time_s | 相对采集开始时刻，s |
| x_sar_m, y_sar_m, z_sar_m | 质心位置估计，X前/Y右/Z上，m |
| vx_sar_mps, vy_sar_mps, vz_sar_mps | 速度估计，m/s |
| roll_ned_rad, pitch_ned_rad, yaw_ned_rad | FRD→局部 NED 的 ZYX 欧拉角，rad |
| roll_ned_deg, pitch_ned_deg, yaw_ned_deg | 同一姿态的度表示 |
| qw_nb, qx_nb, qy_nb, qz_nb | 同一姿态的标量在前单位四元数 |
| truth_p_frd_radps, truth_q_frd_radps, truth_r_frd_radps | 体轴角速度真值，rad/s |
| truth_ax_sar_mps2, truth_ay_sar_mps2, truth_az_sar_mps2 | 质心加速度真值，m/s² |
| apc_x_sar_m, apc_y_sar_m, apc_z_sar_m | 由导航位置与导航姿态推算的 APC，m |
| position_error_*_m | 已知合成位置估计误差，m |
| velocity_error_*_mps | 已知合成速度估计误差，m/s |
| attitude_error_r*_rad | 右乘姿态误差的 FRD 旋转向量，rad，不是欧拉角差 |

导航估计模型：默认15维局部线性化GNSS/INS误差传播与Kalman校正；PVA误差关联生成。详细状态、单位及假设见[模型说明](gnss_ins_model.md)。旧独立位置/速度/姿态统计模式仅用于历史YAML兼容。

角速度与加速度目前没有建立额外传感器误差，因此使用 **truth_** 前缀，不能解释为陀螺或加速度计实测。新模型包含GNSS离散校正，更新点的估计位置导数不必等于输出速度；它是线性误差状态仿真，不是原始IMU日志的完整非线性解算器。

`truth_full` 保存预稳定和采集的 3200 Hz 真实刚体量；`truth_sar` 使用整数索引直接取出 800 Hz 状态，无低频插值或补齐。`navigation_sar` 的位置/速度来自 100 Hz 线性重采样，姿态来自同一 100 Hz 导航四元数的 SLERP，不再生成随机噪声。其 APC 同时使用重采样的导航位置与导航姿态。

`sar_geometry` 是另存的连续几何解耦实验状态，标记 physical_truth=false，不应当作导航真值或刚体真值。名义双航次基线与实际航迹差异分别记录。

MAT 中一维时间保存为列向量，向量状态 N×3、四元数 N×4、旋转矩阵 N×3×3。`dynamics_sample_index` 和 `pulse_index` 为 Python **零基**；MATLAB 数组寻址需加 1。MATLAB 在计算时间/频率时需把 Python 导出的 int64 频率显式转为 double。

```matlab
[navigation, truth, metadata] = load_navigation_output('results/<run>/navigation_pass1.mat');
plot(navigation.time_s, navigation.euler_NB_rpy_deg);
```
