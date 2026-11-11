# Main：运行入口

顶部“全链路验收”会串行运行RotorPy、MATLAB、内置CPU、SITL、PX4的双航次及各自适用的完整检查，再生成汇总和交叉对照。需要MATLAB及已部署WSL，不含SAR成像端到端；详见[使用说明](docs/CONTROL_PANEL_USER_GUIDE.md#新增全链路验收)。

上位机已提供RotorPy、MATLAB、ArduPilot SITL和PX4/Gazebo统一启动入口。外部链路需要已部署的WSL环境，可选择统一导航误差模型或飞控自身导航；参数适用范围、输出来源与外部验收边界见[上位机详细说明](docs/CONTROL_PANEL_USER_GUIDE.md#仿真链路统一入口2026-11-11)。

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.0.0发布说明](../docs/FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

> 当前默认已升级为加速度计/陀螺仪误差传播与GNSS校正，旧独立PVA误差及80%/20%拆分仅保留历史兼容；以 [GNSS/INS模型说明](../docs/gnss_ins_model.md) 为准。

日常可直接双击 `UAVMotionSim.exe` 打开上位机，也可使用以下两个 `.cmd` 文件：

| 文件 | 用途 |
|---|---|
| `run.cmd` | 一键运行完整双航次验收，最后打开离线结果页。 |
| `open_control_panel.cmd` | 打开上位机，调整参数并运行单航次仿真或完整验收。 |

`config.yaml` 是完整验收配置，`motion_config.yaml` 是上位机默认配置，目标有效航线长度为 1000 m；三轴运动扰动默认small，同为 0.05 m 位置目标 OU。也可在上位机加载 [三轴外力配置](../configs/three_axis_force_6n.yaml)。`internal/` 存放程序实现，`tools/` 存放外部平台和 PowerShell 辅助入口，`docs/` 存放详细记录；这些目录中的脚本不是日常主入口。结果写入 `outputs/` 和 `gui_outputs/`。

## 其他运行方式

导航误差默认参考北云 X1-5、RTK、GNSS未中断指标；换算规则与适用条件见 [X1参数依据](../docs/navigation_x1_reference.md)。

OpenSource 外部平台验收可双击 **tools/run_external_validation.cmd**：依次运行 ArduPilot SITL 与 XTDrone2/PX4/Gazebo 的 Hex-X 起降并生成中文航迹图。该入口已串联运行通过；运行范围和原始记录见 [外部平台部署](../deployment/README.md)。

完整 300 m 采集航段可双击 **tools/run_external_route.cmd**，两平台分别飞越 20.48 s 航段并生成独立的交互验收页；双平台一键串联入口已实际运行通过。结果范围与指标见 [部署说明](../deployment/README.md)。

外部入口结束后还会生成并打开独立的 `index.html` 汇总页，包含验证状态、可拖动三维航迹、逐帧记录、图片和下载链接。页面位于 `results/external_review/`，最近路径保存在本目录 `outputs/latest_external_report.txt`。

双击 **open_control_panel.cmd** 可在桌面窗口中调整导航误差、飞行/采样、A/B 扰动和资源参数，并启动仿真或完整验收；窗口内提供实时日志、图表和逐帧位姿查看。操作步骤见 [上位机详细使用说明](docs/CONTROL_PANEL_USER_GUIDE.md)，实现细节见 [上位机技术说明](docs/CONTROL_PANEL.md)。

本目录是人工验收入口。默认运行**完整双航次、全长 20.48 s 采集、自动化测试、20,000 s OU 统计、1600/3200/6400 Hz 收敛、MATLAB 导航/几何/回波接口**，最后自动打开本地验收页面。沿用主工程的物理与误差模型，不另造一套演示数据。

## 一键运行

Windows 直接双击 **run.cmd**。也可在本目录运行：

```powershell
.\tools\run.ps1
```

在项目根目录可运行：

```powershell
uv run --no-sync python main/internal/run.py
```

首次使用先在项目根目录执行 `scripts/setup.ps1`。当前机器已完成 uv 环境部署。验收程序根据可用 CPU/RAM 选择 worker 数，记录 GPU/显存；当前动力学为 CPU 后端。

## 结果位置

每次创建 `main/outputs/acceptance_<Shanghai_timestamp>/`，不覆盖历史结果：

```text
acceptance_<timestamp>/
  index.html                  # 人工验收总页，双击即可离线查看
  acceptance.json             # 本次每个检查的真实状态和资源信息
  resolved_config.yaml        # 本次完整配置
  run.log                     # 全流程日志
  logs/matlab_output.txt       # MATLAB 输出
  validate_<timestamp>/       # 原始 MAT/CSV、测试、收敛和图表
```

`outputs/latest_report.txt` 记录最近生成的页面路径。MAT/CSV 保留原始精度；页面逐帧数据仅为显示舍入。页面不依赖外网或本地服务器。

## 人工查看顺序

1. 查看验收状态、脉冲数量、导航数量、测试与数值收敛。未执行项显示“未执行”，不会伪装为通过。
2. 在时间滑块选择任一 100 Hz 导航采样时刻，对照位置、速度、Roll/Pitch/Yaw、wxyz 四元数与 APC；可切换两个航次、播放记录。三维 APC 航迹可拖动旋转、滚轮缩放和切换真实比例，蓝色真值与橙色导航估计均来自本次验收数据。
3. 查看位置/速度/姿态真值与估计逐轴图、导航误差、APC 轨迹、运动状态、天线杆臂和波束、旋翼边界、四元数模、扰动、OU 和收敛图，共 13 张（含IMU/GNSS诊断图）。
4. 打开主要导航 MAT/CSV，核对字段、单位与坐标。主要位置是**质心**；APC 单独输出。角速度与加速度的 truth_ 字段仍为真值。
5. 根据实验目的确认动力学参数、扰动幅度和导航误差初值合理，再将产出用于后续 SAR 实验。

## 调参和运行选项

编辑本目录的 `config.yaml`；OpenSource 版本继承 `configs/rotorpy_hex_x12.yaml`，默认实际调用 RotorPy 后端。为保证人工可检查，入口强制保留 MAT、CSV 和图表，不改动原配置文件。

```powershell
# 指定进程上限，内存约束仍然生效
.\tools\run.ps1 -Workers 1

# 只生成报告，不自动弹出浏览器
.\tools\run.ps1 -NoOpen

# 没有 MATLAB 时允许先检查 Python 结果，整体状态显示 partial
.\tools\run.ps1 -Matlab skip

# 要求 MATLAB 必须存在且通过
.\tools\run.ps1 -Matlab required
```

Python 入口还支持 `--matlab-exe <path>`、`--sar-project <directory>`、`--config <yaml>`。

默认自动检测 MATLAB，并读取相邻既有 L 波段工程进行完整单点回波接口验证。原 SAR 工程不写入；若无 MATLAB、无原回波工程或显式跳过，整体只能标为部分完成。任何数值检查失败均保留日志并停止后续接口验收。

**验收边界（2026-11-11更新）**：本入口现覆盖 RotorPy/Python、MATLAB接口、早期MATLAB UAV物理核心对照，以及已部署SITL/PX4的同工况飞行与原生时钟对照。每次运行状态和差异见报告新增表格，跳过或失败会进入总状态。外部飞控对照采用独立300 m、10 m/s、20.48 s基准。执行环境、额外耗时和参数解释见上位机使用说明。Simscape、实机标定和SAR成像端到端实验仍属于后续工作。
