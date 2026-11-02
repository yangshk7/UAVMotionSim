# 主要输出：导航式位姿记录

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

导航估计模型：位置/速度分别为真值加独立的慢变 OU 偏差与测量白噪声。姿态采用 `R_est=R_true Exp(delta_theta_B)`，再输出等价欧拉角与连续符号四元数。用户已同意的速度慢变标准差为 [0.02,0.02,0.04] m/s、相关时间 3 s；姿态误差旋转向量标准差为 [0.05°,0.05°,0.10°]、相关时间 5 s。新增两类白噪声默认零，可在 YAML 独立设置。

上述数值为早期工程候选值。当前标准实验与上位机已改为继承 `navigation_x1_5_rtk.yaml`，按北云X1-5的RTK、GNSS未中断指标配置；水平RMS换算、姿态近似与相关时间假设见 [X1参数依据](navigation_x1_reference.md)。三个误差通道具有独立开关、随机源、标准差和相关时间，不能反馈或覆盖真实动力学。`hex_x_12kg.yaml` 仍默认关闭所有误差；`experiment_two_pass.yaml` 开启三类导航误差。

角速度与加速度目前没有建立额外传感器误差，因此使用 **truth_** 前缀，不能解释为陀螺或加速度计实测。导航是独立统计误差模型，不是完整 GNSS/IMU 融合解算器；带噪位置的数值导数不必等于独立速度估计。

`truth_full` 保存预稳定和采集的 3200 Hz 真实刚体量；`truth_sar` 使用整数索引直接取出 800 Hz 状态，无低频插值或补齐。`navigation_sar` 的位置/速度来自 100 Hz 线性重采样，姿态来自同一 100 Hz 导航四元数的 SLERP，不再生成随机噪声。其 APC 同时使用重采样的导航位置与导航姿态。

`sar_geometry` 是另存的连续几何解耦实验状态，标记 physical_truth=false，不应当作导航真值或刚体真值。名义双航次基线与实际航迹差异分别记录。

MAT 中一维时间保存为列向量，向量状态 N×3、四元数 N×4、旋转矩阵 N×3×3。`dynamics_sample_index` 和 `pulse_index` 为 Python **零基**；MATLAB 数组寻址需加 1。MATLAB 在计算时间/频率时需把 Python 导出的 int64 频率显式转为 double。

```matlab
[navigation, truth, metadata] = load_navigation_output('results/<run>/navigation_pass1.mat');
plot(navigation.time_s, navigation.euler_NB_rpy_deg);
```
