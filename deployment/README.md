# 外部开源平台部署

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.1.0发布说明](../docs/FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

部署日期：2026-10-31。运行环境为独立 WSL2 `Ubuntu-24.04`；源码和构建位于 Linux `/opt/uav_motion/`。项目原有 RotorPy 链路保留，外部飞控验证使用独立入口。所有验证只连接本次启动的本地仿真实例。

| 组件 | 已部署版本 | 实际验证情况 |
|---|---|---|
| ArduPilot SITL | ArduCopter 4.7.2，官方静态二进制，哈希校验 | 12 kg Hex-X 解锁、约 10 m 起飞、降落停桨通过 |
| pymavlink | 2.4.50，uv 独立环境 | 已接收位置/姿态记录并控制实际 SITL |
| ROS2 | Jazzy，Ubuntu 24.04 官方 ROS 软件源 | ROS2 节点与话题工作 |
| Gazebo | Harmonic / gz-sim 8.15.0 | 生成并实际运行 12 kg Hex-X，而非用四旋翼替代 |
| PX4 | v1.15.4，`99c40407` | 已编译、实际启动；自定义 Hex-X 解锁、起飞、落地检测与自动停桨通过 |
| XTDrone2 | `15b2ef6e`，px4_msgs `18cbcced` | 8 个包构建成功；实际导航话题、状态话题、Offboard/ARM/LAND 服务命令已验证 |
| Micro XRCE-DDS Agent | v2.4.2 | 已构建并桥接 PX4 与 ROS2；本地兼容补丁保存在 `patches/` |

精确提交号、软件包版本、二进制哈希和上游本地改动保存在 `environment_snapshot.json`。安装完成不代表所有飞行验收通过。

`evidence/` 保存本次通过记录，以及修复前 Hex-X、原生 x500 对照失败的原始检查 JSON 副本；完整遥测与日志仍在各自结果目录。

## 一键验证

在本项目根目录的 PowerShell 中运行：

```powershell
.\deployment\run_sitl_validation.ps1
.\deployment\run_xtdrone_validation.ps1
```

也可直接双击 `main/tools/run_external_validation.cmd`，依次验证两个外部平台并生成中文航迹图。该入口独立于 `main/run.cmd` 的 RotorPy/SAR 完整数值验收。

## 300 m 完整采集航段

双击 `main/tools/run_external_route.cmd` 可依次运行 ArduPilot SITL 与 PX4/Gazebo/XTDrone2 的独立航线模式。流程为爬升至约 300 m、在 X=-150 m 附近预定位并助跑、以约 10 m/s 穿越 X=-102.4 m 后开始 20.48 s 采集，最后自动降落。每次结果写入 `results/sitl_route/`、`results/xtdrone_route/` 和 `results/external_route_review/`；页面路径保存在 `main/outputs/latest_external_route_report.txt`。`--no-pause` 用于不自动打开浏览器的批量运行。

当前默认真值请求频率固定为 SITL **300 Hz**、PX4/Gazebo **1000 Hz**。SITL 固件实际按约 333 Hz 输出，请求频率与实测频率分别保存在结果中；详见 [高频真值实测](../docs/high_rate_truth.md)。

航段验收检查实际采样数、前向位移至少 180 m、平均导航高度 290～310 m、横向 RMS 不超过 10 m，以及最终停桨；指标不达标会返回失败，原始 CSV/日志保留。ArduPilot 实际通过记录为 `results/sitl_route/sitl_20261101_103551_887153/`，20.480 s 前向位移约 204.874 m、平均导航高度约 299.997 m、横向 RMS 约 0.020 m。PX4 实际通过记录为 `results/xtdrone_route/xtdrone_20261101_103747_208723/`，20.472 s 前向位移约 207.550 m、平均导航高度约 300.208 m、横向 RMS 约 0.193 m。前两次 ArduPilot 失败记录仍保留，原因是从静止或反向飞行状态开始采集，速度建立时间使前向位移低于门槛；正式模式已在采集前增加助跑。

这些为飞控导航解的航段指标，不是物理真值误差或 800 Hz 脉冲真值。PX4 真值采集重跑记录为 `results/xtdrone_route/xtdrone_20261101_104435_998294/`，航段前向位移约 197.665 m，保存 21,633 条 Gazebo 原始 ENU 位姿及独立导航接收时间。`navigation_truth_comparison.json` 用两进程共享的单调接收时钟对 1,944 个采集段导航点做最近邻配对，最大接收时间差约 8.5 ms；其位置差包含未消除的局部原点偏移和处理延迟，不能当作已标定导航精度。比较工具为 `deployment/compare_gazebo_nav.py`，只读原始 CSV。模型参数仍是工程初值，需实测数据才能标定。

本机无本机型实测日志；已核查可用于处理流程验证的公开六旋翼数据源，见 [公开飞行数据](../docs/public_flight_data.md)。
两套平台提高物理真值输出频率的实际边界、完整航线结果与运行命令见 [高频真值实测](../docs/high_rate_truth.md)。
现已从同次飞行的 PX4 ULog 和 ArduPilot DataFlash 原始日志对齐物理真值与导航仿真时钟，分别导出 100 Hz 导航及原生高频真值；严格时间证据、插值跨度与数据文件见[外部时钟对齐报告](../docs/external_clock_alignment.md)。原先按主机接收时间近邻配对的文件保留，不能代替新的同钟数据。
UAV-FD 的两段无故障实飞记录已下载并核验；六路 PWM→RPM 经验拟合、时间留出及独立航次检查见该文档和 `results/public_data/uav_fd/motor_fit.json`。这些系数未写入本项目 12 kg Hex-X 模型。
公开实飞导航误差评估方法已用 NTU VIRAL 的独立 Leica 位置真值和官方 FAST-LIO2 示例轨迹运行，见[导航对照报告](../docs/public_navigation_validation.md)及 `results/public_data/ntu_viral_eee_01/evaluation.json`。这不等于本项目机体的实机精度。

外部平台可分别运行可配置的风扰/传感器场景，命令和参数含义见 [场景说明](../docs/external_scenarios.md)。目标 12 kg Hex-X 的实测辨识输入、顺序和当前限制见 [标定准备](../docs/calibration_readiness.md)。
2026-11-01 的双平台风扰航线实际通过，比较结果保存在 `results/scenario_review/crosswind_sensor_comparison.json`，含可拖动三维航迹的验收页位于 `results/external_route_review/review_20261101_122512_511446/index.html`。

双平台一键入口在 Windows 代码页 936 下完整重跑通过，返回码 0；本次报告在 `results/external_route_review/review_20261101_105435_286735/index.html`，`report_summary.json` 记录两平台状态和返回码均通过。ArduPilot 航段前向位移约 205.063 m；PX4 约 196.116 m，另有 21,129 条 Gazebo 原始真值和 1,933 条时间近邻对照。页面、三维交互与本地文件链接已检查。第一次未达标的 ArduPilot 航段结果仍原样保留。

PX4 同一次运行另导出 `route_aligned_sar.csv`，逐行保留飞控导航位置、Gazebo ENU 映射位置、最近的速度/位置指令、两路原始时间及接收时间差。它只取实际消息并在 20 ms 内配对，不插值；对照中的位置差仍含原点与延迟不确定性。

2026-11-01 的补充运行：PX4 `results/xtdrone_route/xtdrone_20261101_111850_435438/` 完整航段通过，采集 20.482 s、前向位移 198.816 m、平均导航高度 298.788 m，并记录 Gazebo 真值。PX4 航段按接收时间配对 1,963 个采样，原始导航减真值位置 RMS 为 1.183/0.206/1.026 m；用起飞前静止段的中位差消去基线后为 0.897/0.028/0.312 m，Gazebo ENU/FLU 转 PX4 NED/FRD 后姿态旋转角 RMS 为 1.354°。基线中也含估计器偏差，且处理延迟未校正，这些均不能当作实机导航精度。ArduPilot `results/sitl_route/sitl_20261101_112556_975042/` 通过，采集 20.480 s、前向位移 204.873 m，同时记录 MAVLink `SIMSTATE(164)` 高精度经纬度以及 `SIM_STATE(108)` 高度、速度、姿态。`deployment/compare_sitl_truth.py` 将独立仿真状态与导航按接收单调时钟近邻配对，生成 `route_truth_aligned_sar.csv`；本次水平航段 1,025 条全部配对，最大接收时差 0.143 ms。相对位移 X/Y/Z 的 RMS 差为 0.245/0.013/0.004 m，速度差为 0.019/0.013/0.001 m/s，姿态旋转角 RMS 为 0.650°。相对位移以采集前第一对记录为共同零点，结果仍包含估计器/通信延迟，不能当作实机导航精度。该 ArduPilot 构建的 `SIM_STATE` 浮点经纬度实测呈 degE7 数值，与 MAVLink 文档的 degree 单位不一致；该两列仅作为原始值保存，对照使用 `SIMSTATE(164)` 的 int32 degE7。上述旧运行的 CSV 标题 `lat_deg`/`lon_deg` 不反映实际数值单位，详见对照 JSON 的说明；后续新运行保存为 `lat_raw_float`/`lon_raw_float`。

最新验收页位于 `results/external_route_review/review_20261101_114450_692624/index.html`。“仅水平采集段”同时显示真实米制比例和偏差放大两幅可拖动三维图，均取 `route_navigation_sar.csv` 的实际采集窗口，不含爬升、助跑或降落；偏差图明确显示 Y/Z 相对 X 的放大倍数与各轴原始米数。两平台的指令都是水平直线飞行，因此真实比例图接近直线：SITL 前进 204.873 m，Y/Z 仅变化 0.059/0.009 m；PX4 前进 198.816 m，Y/Z 变化 0.416/1.835 m。放大图只改变显示比例，不补造轨迹扰动。完整航迹默认真实米制比例。旧图各轴独立缩放会把约 0.3 m 的 PX4 上升横向摆动与约 300 m 的高度变化放大到相近画幅；本次上升至 290 m 前导航 X/Y 范围分别约 -0.168～0.173 m、-0.122～0.218 m，Gazebo 物理真值有相近量级的横向运动，故视觉上的剧烈抖动主要来自比例放大，并非数十米的实际横摆。页面是两次独立通过运行的最新记录汇总，标题明确标识，并非同一次一键入口的结果。

真值消息和姿态坐标定义参见 [MAVLink SIM_STATE](https://mavlink.io/en/messages/common.html#SIM_STATE)、[MAVLink SIMSTATE](https://mavlink.io/en/messages/ardupilotmega.html#SIMSTATE)、[PX4 VehicleAttitude v1.15](https://docs.px4.io/v1.15/en/msg_docs/VehicleAttitude) 和 [PX4 坐标系说明](https://docs.px4.io/v1.15/en/ros/external_position_estimation)。

## 离线 index.html 汇总页

一键入口现在会在 `results/external_review/review_<timestamp>/index.html` 生成本次结果页，交互运行结束后自动打开。`main/outputs/latest_external_report.txt` 记录最近页面的完整路径。`--no-pause` 自动化模式生成页面但不打开浏览器。

页面提供平台切换、可拖动旋转/缩放的三维航迹、真实/独立比例切换、时间滑块、播放、位置/速度/最近姿态记录、静态图、原始 CSV 和日志链接。它直接读取实际记录，姿态显示最近消息的时间差，没有生成插值真值。

入口启动前保存已有目录清单，报告只选择随后本次新生成的结果；某平台未执行或失败时保留对应状态，不会拿历史通过记录补齐。若只想查看此前已生成的数据，可单独运行 `uv run --no-sync python deployment/create_report.py`，它会生成明确标为“已保存结果汇总”的页面。页面可直接离线双击打开，不依赖 CDN 或本地服务器。

两个入口都创建新结果目录，保存控制命令、仿真日志、导航 CSV 和 `validation.json`。只终止本次创建的子进程组。SITL 使用独立实例 20（TCP 5960）；XTDrone2 使用隔离 Gazebo 分区 `uav_motion_hex_x12` 与本地 UDP 8888。

Windows PowerShell 5.1 在中文代码页下可能把捕获的 `wslpath` UTF-8 输出错误解码，造成中文目录变成乱码。入口现直接使用 `wsl --cd <Windows项目路径> --exec ...`，由 WSL 原生处理工作目录，Linux 侧使用相对路径。已用 `tests/windows_wsl_path_smoke.ps1` 在代码页 936 和 65001 下验证实际文件访问，无需改名中文父目录或修改系统编码。

该修复还通过了 `cmd.exe` 代码页 936 下的完整双平台入口重跑，返回 0，两个起降检查与中文图表均通过；证据见 `evidence/windows_path_validation.json`。

入口只有在全部起降检查通过时才返回 0，失败时日志和数据照常保留。Python 测试套件本次 68 项通过，包括新增的 Gazebo 六旋翼总质量、惯量及推力上限独立复算；飞行验证通过由实际 PX4/ArduPilot 状态单独确认。

SITL 已通过的实际记录为 `results/sitl/sitl_20261031_154519_105217/`：最高相对高度约 10.013 m，位置 2,671 条、姿态 4,630 条。`flight_summary.png` 为实际遥测中文图表。

XTDrone2 修复后通过记录为 `results/xtdrone/xtdrone_20261031_162625_964802/`：最高相对高度约 4.647 m，导航位置 2,022 条、姿态 2,023 条、XTDrone2 状态 387 条；PX4 日志含 `Landing detected` 与 `Disarmed by landing`，Offboard 发送状态最终为 `DISABLED`。目标高度 5 m，本烟雾测试在达到 4 m 后请求降落，不用于声称高度跟踪误差或 SAR 航线精度。

随后从 `main/tools/run_external_validation.cmd --no-pause` 串联重跑两个平台，整体返回 0。最新 XTDrone2 结果为 `results/xtdrone/xtdrone_20261031_162949_483003/`（最高约 4.622 m，位置 3,088 条、姿态 3,089 条），SITL 结果为 `results/sitl/sitl_20261031_162926_289887/`。两者均包含自动生成的中文 `flight_summary.png`。`validation_summary.json` 汇总这次验证，失败历史没有修改为通过。

此前失败记录保留。原生 x500 在相同命令流程中也复现降落异常，排除了仅由 Hex-X 参数导致的解释。根本原因是 XTDrone2 `land()` 没有停止 Offboard 定时器，旧位置目标与 PX4 自主降落轨迹交替写入同一控制话题。`xtdrone-stop-offboard-on-land.patch` 在 LAND 前清空旧目标并停止其发送，随后 Hex-X 自动起降通过。没有关闭保护、强制解锁/停桨或放宽落地门槛。

## 数据含义

- MAVLink SITL 的 `time_boot_s` 为飞控开机时间。PX4 DDS 时间戳经过时钟同步，新记录使用 `timestamp_s`，不称为开机时间。早期 XTDrone 诊断记录的 `time_boot_s` 列实际上也是同步时间戳，只能按相邻时间差/相对时间解释；原诊断文件保留。
- 位置由局部 NED 转为 X 前、Y 右、Z 上，Z 为相对局部原点高度。不是 300 m APC 位置，也不是 800 Hz SAR 物理真值；位置和姿态按各自实际消息时间分别保存。
- PX4 本次记录在落地时仍有导航 Z 偏移，原始图表会直接显示负值；它不是机体穿地或已验证的离地高度。尚未完成导航精度与传感器参数标定，不能根据本次短时起降声称定位误差达标。
- 这里的导航解来自 ArduPilot/PX4 的实际估计器，不是原项目独立注入误差的统计模型；采样率应根据记录时间计算，不能假定等于指令请求频率。

## Hex-X 参数与模型差异

ArduPilot JSON 设置质量 12 kg、直径 1.1 m、惯量 `[0.55,0.60,1.0] kg·m²` 和六旋翼。其内置推力/电池模型与原 RotorPy 电机模型不同，`hoverThrOut` 是工程映射，尚未逐项辨识。

Gazebo 生成器设置六个 0.55 m 旋翼杆臂、总质量 12 kg、总惯量同上、推力系数 `8e-5`、反扭矩/推力比 `0.03 m`、电机时间常数 0.1 s、名义单桨推力上限 42 N。旋翼实体质量及惯量已从机体中扣除，测试独立复算总量；传感器和接触结构来自 PX4 上游 x500_base，外形为简化示意。PX4 ESC 接口为整数转速，最大指令 724 rad/s；解锁后怠速为 150 rad/s。生成器中的推力曲线和悬停量依据该速度映射设置，并不代表已调定的实机飞控。

## 部署与兼容记录

1. 现有 Ubuntu-26.04 存在 22.04/26.04 混合软件源，因此新增 Ubuntu-24.04 用于正式运行；没有修改旧发行版的软件源。早期源码下载与初次 SITL 测试保留在旧发行版用户目录 `~/uav_motion_open_source/`。
2. ROS2/Gazebo 来自 ROS 官方 Noble 软件源，XTDrone2 的安装要求来自其 `docs/InstallationTutorial.md`。PX4 构建使用独立 uv 环境，Python 依赖锁在 `px4/requirements.lock`。
3. DDS Agent 2.4.2 引用的 `2.12.x` 分支已失效，固定为 Fast-DDS `v2.12.2`；该旧版生成文件缺少 `<cstdint>`，添加显式头文件。补丁均可审阅，未修改算法门槛。
4. XTDrone2 的 LAND/Offboard 指令冲突已用独立补丁修复；验证入口启动前检查补丁是否已应用。ROS2 节点收到正常的退出信号时，上游代码可能打印 `ExternalShutdownException`，不代表已完成的飞行检查失败，检查状态以 `validation.json` 和飞控落地日志为准。
5. `sitl/setup.sh` 可校验并重建 SITL 环境。官方下载地址为 stable，脚本强制校验本次 4.7.2 的 SHA256；若 stable 更新会明确失败，不能静默切换版本。PX4/XTDrone2 已安装工作区可由上述验证入口直接复用。

来源：[ArduPilot WSL 部署](https://ardupilot.org/dev/docs/building-setup-windows11.html)、[官方 SITL 二进制](https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/)、[XTDrone2](https://github.com/andy-zhuo-02/XTDrone2)、[PX4 v1.15.4](https://github.com/PX4/PX4-Autopilot/tree/v1.15.4)、[DDS Agent v2.4.2](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.2)。PX4 传感器/接触定义沿用上游 BSD-3-Clause 许可，XTDrone2 为 MIT，ArduPilot 为 GPLv3；这些依赖的源代码与许可证保留在对应 Linux 仓库中。
