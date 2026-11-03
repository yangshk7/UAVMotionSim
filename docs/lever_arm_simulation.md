# 仿真 CG→SAR APC 杆臂设置与几何验证

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.0.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

本项目使用**仿真安装值**，不声称完成实机测量标定。两份原始资料一致：`UAV_Motion_Sim/configs/hex_x_12kg.yaml` 与 `无人机航迹仿真/functions/config/default_uav_config.m` 均给出质心到 SAR 天线相位中心的机体 FRD 杆臂 `l_B=[0,+0.10,+0.20] m`。后者的技术指导书还明确给出外部 SAR 坐标偏移 `[0,+0.10,−0.20] m`。FRD 的三轴是**前 +X、右 +Y、下 +Z**；外部 SAR 的 Z 向上，所以第三分量符号反转。

上位机“飞行与采样 → 天线相位中心安装 · 仿真杆臂”有三个独立输入框，可修改这组值。“检查参数”与状态栏显示实际使用的 FRD 向量。配置保存在 `main/motion_config.yaml` 的 `antenna.phase_center_frd_m`；另存配置会保存用户当前选择，运行时快照和航段 JSON 也保留该参数。改变杆臂会改变 APC 真值、导航 APC 估计、名义质心参考和天线几何；不会凭空改变所保存的机体质心物理轨迹。

对任一机体位姿，计算公式为

`p_APC,SAR = p_CG,SAR + diag(1,1,−1) R_NB l_FRD`。

其中 `R_NB` 是机体 FRD 到导航 NED 的主动旋转，`l_FRD` 单位 m，`p_CG,SAR` 与 `p_APC,SAR` 为 X 前、Y 右、Z 上的米制位置。真值使用真实姿态，导航估计使用估计姿态，避免把真值姿态偷用于导航 APC。零姿态时，若 APC 为 `(0,0,300) m`，质心应为 `(0,−0.1,300.2) m`。

外部已对时航迹也可在上位机“结果查看”点击“外部同钟航迹应用杆臂”，选择包含 `alignment.json` 和 `pair_validation.json` 的目录。程序读取当前窗口的杆臂值，在新建英文子目录输出 `apc_truth.csv`、`apc_navigation.csv` 和 `lever_arm_manifest.json`，不覆盖原始真值或导航。命令行等价入口为：

```powershell
uv run --no-sync python scripts/apply_simulated_lever_arm.py --config main/motion_config.yaml --source <clock_sync_dir> --output <new_output_dir>
```

已对 PX4/Gazebo 的 **20481 真值 + 2049 导航**样本及 SITL 的 **8198 真值 + 2050 导航**样本运行。结果分别位于 `results/xtdrone_route/xtdrone_20261102_095933_737941/clock_sync/lever_arm_reference/` 和 `results/frequency_probe/sitl_route/sitl_20261102_095735_133880/clock_sync_final/lever_arm_reference/`。所有样本的 APC–质心距离与理论长度 `sqrt(0.1²+0.2²)=0.22360679775 m` 相符，最大绝对偏差小于 `8.2e−14 m`。解析的零姿态与 90° 偏航用例也通过自动测试。

这些检查证明**给定杆臂被正确应用**，并不证明安装值与某架真实无人机一致。将来如需实测标定，应提供同一时钟下独立测得的 CG 位姿与 APC 位置（或 CAD/测量尺寸及其坐标定义），才能估计或核验真实杆臂。外部飞控仿真模型没有 SAR 天线 APC 独立观测，因此不能仅从当前飞行日志反求该长度。
