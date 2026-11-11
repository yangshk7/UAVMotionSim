# 分阶段实施记录

> 历史记录/原始设计：以下参数、测试数量、路径和结论仅对应文中版本，不是最终版现状。最终版统一以 [v1.1.0发布说明](FINAL_RELEASE.md) 和其引用的最终验收记录为准。原始结果未追溯修改。

## P0：环境部署与初始化

1. 完成：项目专属 Python 3.11.15、uv 虚拟环境、依赖安装和锁定、测试框架、RotorPy 源码阅读、既有 MATLAB 配置只读核查。
2. 文件：pyproject.toml、requirements.lock、src/uavsim/__init__.py、tests/test_environment.py、docs/rotorpy_review.md、docs/implementation_plan.md。
3. 原理：独立环境不改变现有 SAR 项目。用户确认规范参数；既有 default_config.m 也包含相同的 800 Hz/300 m/10 m/s/16385 基准。
4. 测试：pytest 环境及 P1 测试。
5. 结果：4 passed（含 P1）。
6. 问题：未将物理初值视为实测参数；外部飞控尚待独立验证。
7. 下一步：P1。

## P1：坐标与天线几何

1. 完成：极向量/轴向量反射、wxyz 四元数、ZYX 姿态、APC 杆臂、含旋转项的 APC 速度/加速度。
2. 文件：src/uavsim/frames/*、sar_adapter/antenna_geometry.py、tests/test_frames.py。
3. 原理：内部 NED/FRD；外部 C R C。轴向角速度反射为 det(C)C，禁止对四元数单分量翻转。
4. 测试：坐标往返、姿态正交性、四元数微分、右视 50°、APC 安装。
5. 结果：4 passed（含环境）。
6. 问题：无本阶段已知失败项。
7. 下一步：P2。

## P2：六旋翼物理模型

1. 完成：Hex-X、转速平方推力、反扭矩、电机滞后、受限分配、19 状态、确定性 RK4。
2. 文件：config/*、vehicle/*、dynamics/state.py、equations.py、integrator.py、tests/test_dynamics.py。
3. 原理：旋翼力矩由 r×F 重建；可行分配走预计算伪逆，饱和时求解有界最小二乘；控制器和噪声不进入 RHS。
4. 测试：悬停、自由落体、电机指数解析解、恒偏航力矩、饱和、非法惯量/频率/种子。
5. 结果：累计 13 passed。
6. 问题：机体参数为规范推荐初值，尚未做实机辨识。
7. 下一步：P3 飞控。

## P3：串级飞控

1. 完成：20/50/200/200 Hz 独立串级环、零阶保持、前馈、PI 抗饱和、SO(3) 误差、参考限幅及平滑航迹。
2. 文件：controllers/*、trajectory/*、tests/test_controller.py、test_controller_acceptance.py。
3. 原理：根据 NED 推力方向构造姿态，用 SO(3) 对数生成体轴角速度，用惯量和陀螺项生成力矩；积分状态不混入刚体状态。
4. 测试：巡航/调度、速度偏差、高度恢复；另有悬停、加减速、高度变化、横移、Roll/Pitch/Yaw 恢复和小角度极点检查。
5. 结果：首批 2 passed，扩展控制器验收 11 passed；每项稳态速度门槛 0.1 m/s、高度门槛 0.05 m。
6. 问题：增益仅完成仿真首轮整定，尚未实机或 SITL 调参验证。
7. 下一步：P4。

## P4：扰动系统

1. 完成：各轴独立开关和随机源、双尺度 OU、候选三级预设、指令/动力学注入、单独几何解耦。
2. 文件：disturbances/*、experiments/geometry_decoupling.py、tests/test_disturbance.py、test_scenarios.py。
3. 原理：精确 OU 递推和固定公共输入样条；位置/姿态指令低通后微分；外力/力矩单位与参考偏移分开。
4. 测试：20,000 s 长序列均值/方差/ACF/PSD、随机复现、通道独立、单因素 X/Y/Z/Roll/Pitch/Yaw、组合输入和四种几何选择。
5. 结果：首批 4 passed，扩展场景 15 passed（含 P5 并行与基线验证）。
6. 问题：候选扰动幅值尚未实测标定，指令低通后标准差不保证等于输入候选值。
7. 下一步：P5。

## P5：导航与双航次

1. 完成：100 Hz 导航、独立航次种子、名义基线、CPU 进程并行和导航误差严格隔离。
2. 文件：navigation/*、experiments/two_pass.py、runtime/*、tests/test_navigation.py、test_runtime.py、test_navigation_pose.py。
3. 原理：导航误差在真实运动结束后独立计算；相同种子不随 worker 数改变。名义基线和实际差异分别记录。
4. 测试：逐位真值隔离、导航重采样、串行/并行逐位复现、不同航次独立、内存不足拒绝、GPU 检测与后端标记。
5. 结果：P5 初始检查 4 passed；新增完整导航位姿与导出检查累计独立 7 passed（含原导航与导出）。
6. 问题：资源预算是启动时估算，未实现 GPU 动力学后端。
7. 下一步：P6。

## 用户追加：导航式位姿与速度/姿态估计误差

1. 完成：100 Hz 主输出包括位置、速度、欧拉角 rad/deg、wxyz 四元数、真值角速度与加速度、估计 APC；独立输出对应真实位姿。
2. 文件：navigation/state_error.py、navigation/resampling.py、matlab/load_navigation_output.m、visualization/navigation_plot.py、docs/data_interface.md。
3. 原理：速度为独立 OU 偏差加可选白噪声；姿态用 R_est=R_true Exp(delta_theta_B)，保持四元数单位模与符号连续。800 Hz 姿态只由 100 Hz 导航 SLERP 得到。
4. 参数：用户明确同意速度标准差 [0.02,0.02,0.04] m/s、tau=3 s；姿态旋转向量标准差 [0.05,0.05,0.10]°、tau=5 s；新增白噪声默认零。
5. 测试：新增模型隔离性、源独立性、SO(3) 误差重建、四元数、100 Hz 独立 MAT/CSV、100→800 Hz APC/姿态对齐。
6. 问题：这是一组统计估计误差，不是完整 GNSS/IMU 融合；角速度/加速度仍明确标为真值。
7. 下一步：完整 P6/P7 回归。

## P6：SAR 与 MATLAB 接口

1. 完成：整数步脉冲同步、MAT/CSV、导航专用 MAT/CSV、配置/版本/种子/源指纹、图表、MATLAB 读取和原 L 波段运动适配器。
2. 文件：sar_adapter/*、matlab/*、cli.py、configs/*、examples/*、scripts/*、tests/test_export.py。
3. 原理：SAR 真值直接取 3200 Hz 状态索引；导航位置/速度线性、姿态 SLERP；几何实验状态独立保存。
4. 测试：16385/2049/81537 点数和首末索引、MAT/CSV 往返、禁止覆盖；MATLAB 完整两航次与现有 4096×16385 单点回波。
5. 结果：初次完整 MATLAB 验证通过，APC 重建误差 0 m、回波斜距误差约 5.68e-14 m；追加导航模型后另做最终回归，见 validation_report.md。
6. 问题：原回波程序的固定波束权重尚未接入新姿态波束；原 SAR 代码保持只读。
7. 下一步：P7 最终科学验证和性能记录。

## P7：科研级验证

1. 完成：解析解、无外力矩能量/惯性角动量守恒、动力学/旋转运动学残差、OU 统计、1600/3200/6400 Hz 公共输入收敛、控制器场景与导航隔离。
2. 文件：analysis/*、tests/*、docs/validation_report.md、结果目录中的 scientific_validation.json 和图表。
3. 原理：通过独立解析/守恒量和共同连续扰动交叉检查；不以运行成功替代指标验收。
4. 测试与实际最终数字：见 validation_report.md 和对应日志，不将历史阶段测试数量相加冒充总数。
5. 首次完整结果：OU 方差 0.9930、ACF 最大误差 0.00934；3200 对 6400 Hz 斜距 RMS 1.555e-6 m，低于 1e-3 m。
6. 问题：P8 SITL、Simscape、实机参数辨识尚未完成，见 external_validation.md。
7. 下一步：交付已验证独立工程；外部验证需另行部署和参数映射。
