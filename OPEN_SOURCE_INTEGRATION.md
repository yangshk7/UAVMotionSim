# 开源集成版本状态

本目录是原 `UAV_Motion_Sim` 的独立 Git 版本，保留 12 kg Hex-X、800 Hz SAR、100 Hz 导航和 APC 高度 300 m 的基准。原项目未修改。

## 第一阶段：已实际集成 RotorPy

- 使用 uv 安装并锁定 **RotorPy 3.0.0**。`src/uavsim/dynamics/rotorpy_hex.py` 每个 RK4 子步调用 RotorPy 的 `Multirotor._s_dot_fn`，由它计算六旋翼推力/力矩、刚体运动、电机一阶响应和四元数导数。NumPy/SciPy、Pydantic/PyYAML、Matplotlib/Pillow 等开源依赖仍负责航迹、分配、配置、导航、导出和图表。
- RotorPy 3.0.0 的构造器对 4×N 分配矩阵求方阵逆，不能直接初始化 N=6。适配层先完成合法四旋翼初始化，再设置六旋翼几何；实际运行采用六个**直接电机转速指令**，不调用 RotorPy 的四旋翼分配逆矩阵。项目原有 4×6 有界分配器仍生成六个指令。未修改 RotorPy 安装包。
- RotorPy 使用 Z-up 和 `xyzw`；本项目使用 NED/FRD 和 `wxyz`。适配层用绕 X 轴 180° 的坐标变换处理位置、速度、姿态、角速度、旋翼位置及反扭矩符号，按 FRD 叠加外力/外力矩，再变回原 19 维状态。为保持基准，RotorPy 风场、气动项和电机随机噪声默认关闭；导航估计误差仍独立生成。
- 适配依赖 RotorPy 的私有 `_s_dot_fn`。升级 RotorPy 前必须重新验证坐标、动力学和 SAR 接口；依赖版本已精确固定在 `pyproject.toml`/`uv.lock`。

默认配置 `configs/rotorpy_hex_x12.yaml` 使用 `runtime.backend: rotorpy`。`cpu` / `auto` 可回退原自实现动力学；两者和 RotorPy 后端当前均为 float64 CPU，检测到 GPU 不表示已用于积分。上位机“执行与资源”可切换后端。

## 已运行验证

| 检查 | 实际结果 |
|---|---|
| `tests/test_rotorpy_adapter.py` | 六旋翼几何及有姿态、角速度、非对称旋翼转速、外力/外力矩时的导数与独立方程对照，最大允许差 `1e-10`；通过 |
| `uv run --no-sync pytest -q` | 67 项通过 |
| 短时单航次 `configs/rotorpy_smoke.yaml` | RotorPy 后端运行、导出与状态检查通过 |
| 完整双航次 `configs/rotorpy_hex_x12.yaml` | 每航次 SAR 16,385 点、导航 2,049 点，状态检查通过；本机本次仿真约 52.8 s |
| MATLAB 读取完整双航次 | 时间、APC、波束、导航与 4096×16385 单点回波接口通过；斜距复核最大差 `5.684e-14 m` |
| `main/run.py` 完整验收 | 双航次、67 项测试、20,000 s OU 统计、1,600/3,200/6,400 Hz 收敛、12 张图和 MATLAB 接口全部通过；本次总用时约 220.2 s |

完整双航次证据位于 `results/run_20261031_104632_687039/` 的 `validation.json`、`metadata.json` 和 `matlab_validation.json`。上述对照证明实现与接口在这些条件下一致，**不代表参数已实机标定**。旧 SAR 的固定波束权重仍未消费姿态变化波束。

完整验收报告位于 `main/outputs/acceptance_20261031_111655_817734/index.html`；`acceptance.json` 记录所有阶段实际为 `passed`，3,200 对 6,400 Hz 斜距差 RMS 为 `0.001555 mm`（门槛 1 mm）。上位机短时集成自检也通过，包含后台任务、导航位姿、可拖动三维航迹和任务取消。

## 使用

在本目录运行：

```powershell
uv sync --locked --extra dev
uv run --no-sync python -m uavsim.cli run --config configs/rotorpy_hex_x12.yaml --passes 2
uv run --no-sync pytest -q
```

双击 `main/open_control_panel.cmd` 可通过上位机调参；`main/run.cmd` 为完整人工验收入口。完整验收入口已在本版本实际运行通过。每次运行创建新时间戳目录，不覆盖已有结果。

## 后续阶段：部署更新（2026-10-31）

1. **ArduPilot SITL：已部署并运行。** Ubuntu-24.04 独立环境中安装 ArduCopter 4.7.2、pymavlink 2.4.50，质量/惯量/六旋翼几何已映射；10 m 起飞、降落与停桨实际通过。推力、电池和电机模型尚不等同于 RotorPy，不能称为完整同模型交叉验证。
2. **XTDrone2 / ROS2 / Gazebo / PX4：已部署并完成短时起降。** ROS2 Jazzy、Gazebo Harmonic 8.15、PX4 v1.15.4、DDS Agent v2.4.2 与 XTDrone2 8 个包已构建并实际互通。修复上游 LAND 后残留的 Offboard 指令流后，自定义 12 kg Hex-X 解锁、起飞、落地检测与自动停桨通过。运行入口、补丁、锁定版本及修复前后证据见 [部署说明](deployment/README.md)。
3. **开源 SAR 链路：**目前仍由用户已有 MATLAB SAR 工程作接口验证。若要求完全脱离 MATLAB，需单独实现并校验 Python 回波与成像链路。

MathWorks Simscape 示例仅是建模参考，Simscape 软件本身不是开源运行依赖，也尚未做交叉验证。

## 2026-11-01 完整采集航段补充

两个外部平台已分别飞完 300 m 高度、10 m/s 指令和 20.48 s 采集航段并自动降落。`main/run_external_route.cmd --no-pause` 已在代码页 936 下整条重跑，返回 0；本次 ArduPilot 实际航段前向位移约 205.063 m，PX4/Gazebo 约 196.116 m，离线汇总为 `passed`。结果和原始日志见 [部署说明](deployment/README.md)。

PX4 新增 Gazebo ENU 原始机体位姿、导航接收时间和采集段指令 CSV。单调接收时钟近邻对齐的 1944 条采集段样本最大时间差约 8.5 ms；原始位置差仍包含局部原点与通信/估计延迟，不当作已标定导航精度。尚未生成 800 Hz 外部物理真值或完成相位误差、成像质量交叉验证。参数实测辨识与 Simscape 对照仍未完成。
