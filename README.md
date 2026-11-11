# UAV Motion Sim

新机器按[复现与部署说明](docs/REPRODUCIBILITY.md)安装固定Python与外部环境；正式验收配置和精简证据随仓库保存。

顶部新增“全链路验收”：串行执行RotorPy、MATLAB、内置CPU、SITL、PX4双航次及各自适用的完整检查，生成总览、子报告和交叉指标；不包含暂缓的SAR成像端到端。详见[上位机使用说明](main/docs/CONTROL_PANEL_USER_GUIDE.md#新增全链路验收)。

> 当前版v1.1.0：日常上位机默认1000 m、10 m/s、10 s裕量，输出110 s；六轴small。`main/run.cmd`仍是固定20.48 s标准接口验收。最终参数和结果以[发布说明](docs/FINAL_RELEASE.md)为准。

上位机“仿真链路”现可统一选择`rotorpy`、`matlab`、`sitl`、`px4`，保留`cpu/auto`兼容选项。外部入口真实启动本机已部署的WSL平台，按上位机直线航线参数飞行、严格对时，统一导出高频真值和100 Hz导航；同时保留原始日志、飞控导航和统一误差模型导航。外部A/B扰动等尚未统一注入，完整验收范围也有所区别，使用前见[上位机详细说明](main/docs/CONTROL_PANEL_USER_GUIDE.md)。本次接入不包含全新电脑上的WSL环境自动部署验证。

**本目录是独立的 OpenSource 版本。** 默认实际调用已安装的 RotorPy 3.0.0 多旋翼动力学，保持 12 kg Hex-X。集成范围、验证和后续阶段见 [开源集成状态](OPEN_SOURCE_INTEGRATION.md)。原 `UAV_Motion_Sim` 未修改。

ArduPilot SITL 与 XTDrone2/ROS2/Gazebo/PX4 已在独立 WSL 中部署，两个平台的短时起降与自动停桨已通过。双击 `main/tools/run_external_validation.cmd` 可重新验证，详见 [外部平台部署](deployment/README.md)。

两个外部飞控还已分别完成 300 m 高度、10 m/s 指令、20.48 s 的完整采集航段与自动降落；双击 `main/tools/run_external_route.cmd` 可重新运行并生成独立交互报告。PX4/Gazebo 已保存原始物理位姿与导航记录，接收时间近邻对齐结果保留时间和原点不确定性，详见 [部署说明](deployment/README.md)。

主航迹数据源的选择指标、采集段三维管道定义与当前实算结果见 [航迹主数据源与管道评估](docs/route_source_selection.md)。

1000 m 无扰动与受扰动航线的阶段验收记录、结果位置和人工查看步骤见 [运动模块阶段验收](docs/acceptance_20261102.md)。

运行生成的 `results/`、`main/outputs/` 和 `main/gui_outputs/` 不纳入 GitHub。当前机器的历史运行数据已保存在项目旁的 `UAVMotionSim_Archive_20261102/`；两份最终上位机验收页面仍留在本机 `main/gui_outputs/`。克隆仓库后按下文命令可重新生成结果。

独立的 12 kg Hex-X 六旋翼六自由度运动仿真工程。运动物理核心可选RotorPy、MATLAB或外部飞控平台，导航误差与导出使用统一适配接口；SAR回波和成像仍在后续端到端流程中验证。所有项目文件与目录采用英文命名，说明和关键算法注释使用中文。

从配置、仿真到导航输出及验收的完整步骤见 [项目工作流程](WORKFLOW.md)。

面向独立 SAR 运动模块的标准产出是一段高频物理真值与同时间原点的 100 Hz 导航估计；字段、边界和外部平台适配条件见 [运动航段模块接口](docs/motion_module.md)。普通命令行及上位机仿真默认一段，完整科研验收继续运行双航次。
上位机可按“目标航线长度÷速度＋额外裕量”计算输出时间。无物理扰动时用目标速度 ±10% 检查连续匀速航段；开启物理扰动后，速度范围仅作观测统计，按实际连续前进距离选择可裁剪航段，保留真实波动。
已验证的 1000 m/10 m/s 示例、同步裁剪数据和外部数据时钟审计见 [非端到端模块交付记录](docs/non_e2e_completion.md)。
两套外部飞控的机体真值和导航现已基于原始 PX4 ULog / ArduPilot DataFlash 完成同钟对齐，频率、插值与验证边界见[外部时钟对齐报告](docs/external_clock_alignment.md)。
上位机可配置仿真 CG→APC 杆臂，并将当前参数应用到外部同钟航迹；默认数值来源及验证见[仿真杆臂说明](docs/lever_arm_simulation.md)。

已实现：旋翼推力与反扭矩、受限六旋翼分配、电机一阶动态、19 状态 RK4、多速率串级飞控、可复现 A/B/E 扰动、双航次、固定天线几何、导航式位姿输出和 MATLAB 适配。

## 安装与运行

**人工验收入口：**打开 `main` 文件夹，双击 `run.cmd`。它会运行完整双航次、测试、统计、收敛和 MATLAB 接口检查，最后打开带时间滑块与关键图表的离线验收页。详见 [main 使用说明](main/README.md)。

**桌面上位机：**双击 `main/open_control_panel.cmd`，直接在窗口内编辑关键参数、保存配置、启动仿真/完整验收，查看日志、图表和导航位姿。详见 [上位机说明](main/docs/CONTROL_PANEL.md)。

在本工程根目录打开 PowerShell。包管理统一使用 **uv**；Python、缓存和虚拟环境位于项目内。

```powershell
.\scripts\setup.ps1
uv run --no-sync python -m uavsim.cli hardware
uv run --no-sync python -m uavsim.cli run --config configs/rotorpy_smoke.yaml --passes 1
uv run --no-sync python -m uavsim.cli run --config configs/rotorpy_hex_x12.yaml --passes 2
```

也可运行 `.\scripts\run_simulation.ps1`。`hex_x_12kg.yaml` 保留原 CPU 基准；`experiment_two_pass.yaml` 打开 A/B 轴与导航误差；`rotorpy_hex_x12.yaml` 在其基础上启用 RotorPy。Small/Medium/Large 配置位于 `configs`。

```powershell
uv run --no-sync python -m pytest
uv run --no-sync python -m uavsim.cli validate --config configs/rotorpy_hex_x12.yaml
uv run --no-sync python -m uavsim.cli plot --input results/<run_directory>
uv run --no-sync python scripts/benchmark.py
```

`validate` 运行自动化测试、20,000 s OU 统计和完整 1600/3200/6400 Hz 数值收敛试验；`--quick` 仅缩短收敛试验，不代表完整验收。每次运行创建新的英文结果目录，不覆盖已有数据。

## 输出内容

主要输出为 **100 Hz 导航式位姿记录**（位置、速度、姿态角、四元数等），以及 **800 Hz SAR 脉冲同步运动记录**。当前标准配置用加速度计/陀螺仪误差传播与GNSS校正生成相关联的位置、速度和姿态误差，并另存对应真实位姿；模型范围和参数假设见 [GNSS/INS模型](docs/gnss_ins_model.md)。角速度与加速度字段明确使用 truth_ 前缀。详见 [数据接口](docs/data_interface.md)。

标准时序：预稳定 `[-5,0)` s，采集 `[0,20.48]` s。完整动力学 81,537 点，采集动力学 65,537 点，SAR 16,385 点，导航 2,049 点。

内部为右手 NED/FRD；对外 SAR 为 X 前、Y 右、Z 上。四元数固定使用 `q_NB_wxyz`。位置、速度、加速度、角速度采用 m、m/s、m/s²、rad/s；姿态角采用 rad。APC 名义高度为 300 m；默认航迹在孔径中心 X=0，采集起点 X=-102.4 m。

输出目录包含完整 MAT 数据、SAR CSV、导航 CSV、配置快照、种子、依赖版本、资源策略、数值检查和图表。详见 [数据接口](docs/data_interface.md)。

## MATLAB 对接

```matlab
addpath('matlab');
motion = load_uav_trajectory('results/<run_directory>');
[navigation, truth] = load_navigation_output('results/<run_directory>/navigation_pass1.mat');
report = verify_sar_geometry('results/<run_directory>');
report = run_matlab_validation('results/<run_directory>', '../L波段仿真');
```

最后一条在本机现有 SAR 工程上生成完整 4096×16385 单点回波进行接口验证；不修改原 SAR 文件或雷达参数。`uav_to_lband_track.m` 提供运动数据适配。现有 `lband.beam_weight` 使用固定波束，尚未消费新平台输出的随姿态变化波束。

## 资源调度

自动检测 CPU、总/可用 RAM、GPU 和显存。按可用内存、CPU 与任务长度规划进程数，限制 BLAS 线程，预分配 float64 数组；不同航次可并行。短任务自动串行，低内存任务提前拒绝并给出所需容量。`--workers` 可指定并行上限，内存约束仍然生效。

当前动力学后端为 CPU，GPU 检测结果会保存，但不宣称 GPU 已参与积分。19 维闭环逐时积分的并行度较低；项目未添加 Torch/CUDA 等大型依赖。详见 [运行优化](docs/runtime_optimization.md)。

## 验证范围

查看 [验证报告](docs/validation_report.md)、[阶段记录](docs/stage_reports.md)、[坐标系统](docs/coordinate_system.md)、[动力学模型](docs/dynamics_model.md)、[扰动模型](docs/disturbance_model.md)。

物理参数与扰动预设均为技术规范推荐初值，未经本机型实测辨识。数值收敛达到毫米门槛不能解释为实机定位精度达到毫米。P8 ArduPilot SITL、Simscape 及实机交叉验证另见 [外部验证](docs/external_validation.md)。
