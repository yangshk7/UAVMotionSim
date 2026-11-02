# 机载 SAR 六旋翼无人机运动仿真平台
## 本地部署与工程实现技术路线（Codex 实施版）

**文档版本：** V1.0  
**编制日期：** 2026-10-30  
**项目定位：** 科研级六旋翼无人机六自由度运动仿真平台  
**核心语言：** Python  
**集成环境：** MATLAB  
**目标平台：** Windows 11  
**实施工具：** OpenAI Codex  
**目标机型：** 12 kg Hex-X 六旋翼无人机

---

# 0. 给 Codex 的项目说明

本文件是项目的总体技术规范和实施依据。

需要开发一套独立的六旋翼无人机运动仿真系统，能够模拟无人机在名义航迹、飞行控制器和外部扰动作用下产生的真实六自由度运动，并将运动状态输出给现有机载 SAR 仿真平台。

本项目不是简单的三维轨迹生成程序，也不是通过正弦函数叠加随机噪声来制造不规则轨迹。

必须建立：

1. 六旋翼刚体动力学模型。
2. 六旋翼旋翼推力与力矩模型。
3. 电机动态模型。
4. 多级串级飞行控制器。
5. 可独立配置的平移和姿态扰动系统。
6. 导航位置估计误差模型。
7. 双航次运动仿真系统。
8. SAR 脉冲同步状态输出系统。
9. 完整的自动化测试与数值验证系统。

最终目标是生成物理一致、时间同步、参数可控、结果可复现的无人机运动状态，供 SAR 回波生成、运动补偿和 DInSAR 仿真使用。

## 0.1 总体技术路线

采用以下技术组合：

| 项目 | 技术选择 | 用途 |
|---|---|---|
| 核心开发语言 | Python 3.11 | 动力学和仿真核心 |
| 数值计算 | NumPy、SciPy | 矩阵运算、积分、四元数 |
| 配置管理 | YAML、Pydantic | 参数配置和合法性检查 |
| 仿真架构参考 | RotorPy | 多旋翼动力学架构 |
| 物理建模参考 | MATLAB Simscape | 机体、旋翼、电机、控制器建模 |
| 独立飞控验证 | ArduPilot SITL | 六旋翼飞控行为交叉验证 |
| 系统架构参考 | XTDrone | 多旋翼仿真系统组织方式 |
| 数据输出 | MAT、CSV | 与 MATLAB SAR 平台对接 |
| 测试 | Pytest | 自动化测试 |
| 可视化 | Matplotlib | 三维轨迹及运动状态分析 |

**重要原则：**

Python 是运动仿真核心，MATLAB 继续负责已有 SAR 回波与成像流程。

不需要把整个 SAR 平台迁移到 Python，也不需要在第一阶段引入 ROS、Gazebo 或 Isaac Sim。

# 1. 项目总体架构

## 1.1 系统架构

整体采用分层、模块化结构。

```text
                用户实验配置
                     |
                     v
              配置管理模块
                     |
                     v
              参考航迹生成模块
                     |
                     v
              六旋翼飞控模块
                     |
                     v
             六自由度动力学模块
                     |
                     v
              3200 Hz 状态积分
                     |
             +-------+-------+
             |               |
             v               v
         A/B 扰动模块     电机动态模块
             |               |
             +-------+-------+
                     |
                     v
               真实运动状态
                     |
           +---------+---------+
           |                   |
           v                   v
      800 Hz SAR 输出       100 Hz 导航误差
           |                   |
           v                   v
      天线几何计算          导航位置估计
           |                   |
           +---------+---------+
                     |
                     v
               数据输出模块
                     |
                     v
               MAT / CSV
                     |
                     v
             MATLAB SAR 仿真平台
                     |
                     v
               回波与成像分析
```

另设独立的验证通道：

```text
六旋翼运动仿真核心
          |
          +---- RotorPy 模型对照
          |
          +---- Simscape 模型对照
          |
          +---- ArduPilot SITL 飞控对照
```

这些工具用于参考和验证，不应成为核心程序运行的强制依赖。

## 1.2 模块之间的基本原则

必须严格区分以下四类数据：

- Reference：参考航迹。
- Truth：真实刚体运动状态。
- Navigation：导航位置估计状态。
- SAR Experiment：SAR 几何解耦后的实验运动状态。

它们不能混合存储。

尤其不能让导航位置估计误差直接改变真实动力学状态。

也不能将几何解耦后的非物理运动状态误认为真实飞行状态。

# 2. 外部项目研究与复用方案

## 2.1 RotorPy：核心动力学参考

GitHub：

https://github.com/spencerfolk/rotorpy

论文：

https://arxiv.org/abs/2306.04485

RotorPy 是基于 Python 的多旋翼仿真系统，具有六自由度动力学、气动力矩、执行器动态、执行器饱和以及随机环境模型等功能。

本项目重点参考：

1. 多旋翼刚体动力学组织方式。
2. 旋翼推力与机体力矩计算。
3. 状态积分和姿态更新。
4. 执行器动态。
5. 参考航迹和控制器接口。
6. 仿真结果及状态记录结构。

本项目不直接使用 RotorPy 默认四旋翼作为最终模型。

需要建立自己的 `HexX12` 机型模型。

**特别注意：**

本项目当前不实现随机风场、湍流、电机随机故障等功能。RotorPy 中相关模块只作为研究资料，不应全部移植。

建议 Codex 首先阅读 RotorPy 的：

- `vehicles/`
- `controllers/`
- `trajectories/`
- `world/`
- `simulate.py`
- `examples/`

具体目录以实际检出的仓库版本为准。

不允许未经审查就把 RotorPy 内部类直接作为项目公共 API。应建立自己的稳定接口，并记录参考仓库的版本或 commit。

## 2.2 MATLAB Simscape：物理建模参考

官方项目：

https://github.com/mathworks/Quadcopter-Drone-Model-Simscape

官方文档：

https://www.mathworks.com/help/simscape-electrical/ug/quadcopter-drone.html

重点研究：

- 机体和旋翼机械模型。
- 电机动态。
- 电机推力与机体力矩。
- 级联 PID 控制器。
- 轨迹跟踪。
- 阵风条件下的控制器响应。

本项目只借鉴物理建模与控制器结构，不要求将 Simscape 作为运行时依赖。

如果本地具有 MATLAB 和相应工具箱，可以将其作为独立的对照验证环境。

## 2.3 ArduPilot SITL：飞控验证

官方文档：

https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html

官方支持 Copter 的 `hexa` 仿真机型。

可采用：

```bash
sim_vehicle.py -v ArduCopter -f hexa --console --map
```

用于对比：

- 六旋翼飞行控制响应。
- 航迹跟踪。
- 姿态控制。
- 速度和高度保持。
- 飞控饱和与日志输出。

注意：ArduPilot 内置 SITL 的 Hexa 机型与本项目自定义的 12 kg Hex-X 参数并不天然一致，必须进行参数映射。

ArduPilot 官方 Gazebo 示例主要提供 Iris 和 Zephyr 等模型。后续如果要接入 Gazebo，需要自行建立对应的 Hex-X 模型，不能假设安装后就有完全匹配的六旋翼模型。

## 2.4 XTDrone：系统架构参考

GitHub：

https://github.com/robin-shaun/XTDrone

论文：

https://arxiv.org/abs/2003.09700

主要参考：

- 多旋翼系统模块化设计。
- 飞控与仿真器接口。
- 状态数据通信。
- 多机仿真组织。
- 任务航线和飞行控制接口。

XTDrone 不作为第一阶段必须部署的软件。

# 3. 本地部署环境

## 3.1 推荐开发环境

核心平台：

- Windows 11 64-bit。
- Python 3.11 64-bit。
- Git。
- Visual Studio Code。
- Codex 本地开发环境。

建议优先使用 Windows 原生 Python 环境。

ArduPilot SITL 可以后续使用独立的 Linux/WSL2 环境部署，不应影响核心 Python 项目的运行。

## 3.2 Python 依赖

核心依赖：

```text
numpy
scipy
pandas
pyyaml
pydantic
matplotlib
h5py
```

开发依赖：

```text
pytest
pytest-cov
ruff
```

建议通过 `pyproject.toml` 管理依赖，并生成锁定文件。

不要在第一阶段安装 PyTorch、ROS、Gazebo、Isaac Sim 等大型依赖。

只有在后续功能确实需要时再引入。

## 3.3 环境初始化

Codex 首先检查本地 Python 环境。

Windows PowerShell：

```powershell
py -3.11 --version

py -3.11 -m venv .venv

.\.venv\Scripts\Activate.ps1

python -m pip install --upgrade pip

pip install -e ".[dev]"
```

如果当前终端使用的是其他虚拟环境，应先检查，不要直接覆盖用户已有环境。

部署完成后执行：

```powershell
python --version

python -m pytest
```

必须输出依赖版本、环境信息和测试结果。

# 4. 项目目录结构

建立独立工程目录：

```text
AirborneSAR_UAVMotionSim/
│
├── pyproject.toml
├── README.md
├── CHANGELOG.md
├── .gitignore
│
├── configs/
│   ├── hex_x_12kg.yaml
│   ├── disturbance_small.yaml
│   ├── disturbance_medium.yaml
│   ├── disturbance_large.yaml
│   └── experiment_two_pass.yaml
│
├── src/
│   └── uavsim/
│       │
│       ├── __init__.py
│       ├── cli.py
│       │
│       ├── config/
│       │   ├── loader.py
│       │   └── validator.py
│       │
│       ├── frames/
│       │   ├── coordinate.py
│       │   ├── rotation.py
│       │   └── quaternion.py
│       │
│       ├── vehicle/
│       │   ├── hex_x12.py
│       │   ├── rotor.py
│       │   ├── motor.py
│       │   └── control_allocator.py
│       │
│       ├── dynamics/
│       │   ├── state.py
│       │   ├── equations.py
│       │   ├── integrator.py
│       │   └── simulator.py
│       │
│       ├── controllers/
│       │   ├── position_controller.py
│       │   ├── velocity_controller.py
│       │   ├── attitude_controller.py
│       │   └── rate_controller.py
│       │
│       ├── trajectory/
│       │   ├── straight_line.py
│       │   ├── waypoint.py
│       │   ├── smooth_transition.py
│       │   └── reference_manager.py
│       │
│       ├── disturbances/
│       │   ├── translation.py
│       │   ├── attitude.py
│       │   ├── ou_process.py
│       │   ├── presets.py
│       │   └── injection_manager.py
│       │
│       ├── navigation/
│       │   ├── position_error.py
│       │   └── resampling.py
│       │
│       ├── experiments/
│       │   ├── geometry_decoupling.py
│       │   └── two_pass.py
│       │
│       ├── sar_adapter/
│       │   ├── antenna_geometry.py
│       │   ├── pulse_sync.py
│       │   └── matlab_export.py
│       │
│       ├── analysis/
│       │   ├── metrics.py
│       │   ├── convergence.py
│       │   └── statistics.py
│       │
│       └── visualization/
│           ├── trajectory_3d.py
│           ├── state_plot.py
│           └── disturbance_plot.py
│
├── tests/
│   ├── test_frames.py
│   ├── test_quaternion.py
│   ├── test_vehicle.py
│   ├── test_dynamics.py
│   ├── test_controller.py
│   ├── test_disturbance.py
│   ├── test_navigation.py
│   ├── test_sar_sync.py
│   └── test_reproducibility.py
│
├── examples/
│   ├── 01_hover.py
│   ├── 02_cruise.py
│   ├── 03_translation_disturbance.py
│   ├── 04_attitude_disturbance.py
│   ├── 05_navigation_error.py
│   └── 06_two_pass_sar.py
│
├── matlab/
│   ├── load_uav_trajectory.m
│   └── verify_sar_geometry.m
│
├── docs/
│   ├── architecture.md
│   ├── coordinate_system.md
│   ├── dynamics_model.md
│   ├── disturbance_model.md
│   ├── data_interface.md
│   └── validation_report.md
│
└── results/
    └── .gitkeep
```

要求：

1. 每个模块职责明确。
2. 不允许所有代码堆积在一个 Python 文件中。
3. 不允许控制器直接修改动力学状态。
4. 不允许扰动模块直接修改积分器内部状态。
5. 所有模块都必须具备独立测试能力。

# 5. 坐标系统与天线几何

这是本项目最重要的基础部分之一。

## 5.1 SAR 外部坐标系

严格采用已经确定的用户坐标：

- X：沿名义航迹向前。
- Y：航迹右侧。
- Z：竖直向上。

这是左手坐标系。

## 5.2 内部动力学坐标系

内部统一采用右手坐标系。

导航坐标 N：

- X：沿名义航迹向前。
- Y：航迹右侧。
- Z：向下。

机体坐标 B：

- X：机头向前。
- Y：机体右侧。
- Z：机体向下。

内部姿态使用：

```text
R_NB
```

表示机体系向导航系的旋转矩阵。

内部四元数使用标量在前的格式：

```text
q = [qw, qx, qy, qz]
```

角速度使用机体系 FRD 坐标，单位 rad/s。

## 5.3 坐标转换

定义：

```python
C = diag([1, 1, -1])
```

位置转换：

```text
p_SAR = C @ p_N
```

速度转换：

```text
v_SAR = C @ v_N
```

加速度转换：

```text
a_SAR = C @ a_N
```

姿态转换必须通过旋转矩阵完成。

当导航系和机体系均使用上述相同的 Z 轴反射变换时：

```text
R_SAR = C @ R_NB @ C
```

再根据明确的旋转约定转换成对外姿态表示。

禁止直接对四元数某一个分量进行 Z 轴符号翻转。

## 5.4 天线安装

固定安装参数：

对外坐标：

```text
APC = [0, +0.10, -0.20] m
```

内部 FRD 坐标：

```text
APC = [0, +0.10, +0.20] m
```

主波束：

- 右视。
- 向下俯角 50°。
- 零斜视角。
- 天线相对于机体固定。
- 不考虑安装机构随机振动。

内部波束单位向量：

```text
b0_B = [
    0,
    cos(50°),
    sin(50°)
]
```

天线相位中心：

```text
p_APC_N = p_CG_N + R_NB @ l_APC_B
```

波束方向：

```text
b_N = R_NB @ b0_B
```

必须输出：

- 机体质心位置。
- 天线相位中心位置。
- 天线主波束方向。
- 天线相位中心速度。
- 必要时计算相位中心加速度。

SAR 高度 300 m 以天线相位中心为准，不是机体质心高度。

初始化时应使用：

```text
p_CG_ref = p_APC_ref - R_ref @ l_APC_B
```

不能简单地把机体质心设置成与天线相位中心完全相同的位置。

# 6. 六旋翼 Hex-X 物理模型

## 6.1 机型参数

基准机型为 12 kg Hex-X 六旋翼。

以下参数分为两类：

- 12 kg、六旋翼、X 布局：项目基准。
- 其他物理参数：采用上一版技术指导书中的初始推荐值，必须保留配置修改能力，不能当作实测数据。

| 参数 | 初始值 |
|---|---:|
| 质量 | 12 kg |
| 旋翼数量 | 6 |
| 机臂长度 | 0.55 m |
| Jxx | 0.55 kg·m² |
| Jyy | 0.60 kg·m² |
| Jzz | 1.00 kg·m² |
| 重力加速度 | 9.81 m/s² |
| 单旋翼最大推力 | 42 N |
| 最大总推力 | 252 N |
| 电机时间常数 | 0.10 s |
| 最大参考速度 | 12 m/s |
| 最大参考水平加速度 | 2.5 m/s² |
| 最大倾角 | 25° |

惯量初始矩阵：

```text
J = diag([0.55, 0.60, 1.00])
```

上述参数均可在 YAML 中修改。

## 6.2 六旋翼布局

六个旋翼均匀分布：

```text
alpha_i = 30° + (i-1)*60°
```

位置：

```text
r_i = arm_length * [
    cos(alpha_i),
    sin(alpha_i),
    0
]
```

反扭矩符号：

```text
s = [+1, -1, +1, -1, +1, -1]
```

这里的符号表示机体 +Z 轴方向的反扭矩约定。

必须确保旋翼编号、旋转方向和控制分配矩阵一致。

## 6.3 推力模型

每个旋翼：

```text
T_i = kf * Omega_i^2
```

反扭矩：

```text
Q_i = km * Omega_i^2
```

上一版推荐的初始系数：

```text
kf = 8.0e-5
km = 2.4e-6
```

因此：

```text
km / kf = 0.03 m
```

六个旋翼的最大推力为：

```text
T_total_max = 6 * 42 = 252 N
```

悬停时：

```text
T_hover = 12 * 9.81 = 117.72 N
```

每个旋翼平均承担：

```text
T_hover_each = 19.62 N
```

## 6.4 电机动态

采用一阶电机模型：

```text
tau_m * dOmega_i/dt = Omega_cmd_i - Omega_i
```

必须实现：

- 电机转速上下限。
- 推力上下限。
- 一阶动态响应。
- 六旋翼控制分配。

注意：

不实现电机随机故障，不代表删除正常电机动态。

正常电机响应滞后是六自由度物理模型的一部分，必须保留。

## 6.5 气动阻力

不实现随机风场。

可以保留确定性的空气阻力模型：

```text
F_D = -0.5 * rho * [
    CDx * Ax * |u| * u,
    CDy * Ay * |v| * v,
    CDz * Az * |w| * w
]
```

阻力参数必须可配置。

第一版默认关闭或使用经过单独验证的初始参数。

必须保留无阻力解析验证模式。

# 7. 六自由度动力学

## 7.1 状态向量

内部状态：

```text
X = [
    position_N(3),
    velocity_N(3),
    quaternion_NB(4),
    angular_velocity_B(3),
    rotor_speed(6)
]
```

共 19 个状态量。

控制器积分状态单独保存。

## 7.2 平移动力学

```text
dp_N/dt = v_N
```

```text
m * dv_N/dt =
    m*g*e3
    + R_NB @ (
        F_rotor_B
        + F_drag_B
        + F_external_B
    )
```

其中：

```text
F_rotor_B = [0, 0, -sum(T_i)]
```

## 7.3 旋转动力学

```text
J * domega/dt =
    tau_rotor
    + tau_external
    - cross(omega, J @ omega)
```

姿态矩阵：

```text
dR_NB/dt = R_NB @ skew(omega_B)
```

四元数使用对应的刚体姿态运动学方程进行积分。

要求：

- 四元数始终归一化。
- 姿态矩阵始终满足正交性。
- 旋转矩阵行列式接近 +1。
- 不混用主动旋转和被动旋转约定。

## 7.4 数值积分

首选固定步长 RK4。

动力学积分频率：

```text
3200 Hz
```

时间步长：

```text
dt = 1 / 3200 = 0.0003125 s
```

所有随机过程应预先生成，或者按照固定时间网格更新。

禁止在 RK4 微分函数的每次调用中重新抽取随机噪声。

# 8. 串级飞行控制系统

采用简化级联控制：

```text
位置控制器
    |
    v
速度控制器
    |
    v
加速度/总推力计算
    |
    v
期望姿态
    |
    v
姿态控制器
    |
    v
角速度控制器
    |
    v
控制分配器
    |
    v
六个电机
```

建议初始调度频率：

| 控制环 | 频率 |
|---|---:|
| 位置 | 20 Hz |
| 速度 | 50 Hz |
| 姿态 | 200 Hz |
| 角速度 | 200 Hz |

以上作为工程初始配置，可调整。

## 8.1 位置控制

```text
v_cmd = v_ref + Kp_pos * (p_ref - p)
```

## 8.2 速度控制

```text
a_cmd =
    a_ref
    + Kp_vel * velocity_error
    + Ki_vel * integral_velocity_error
```

必须实现：

- 前馈。
- 速度和加速度限幅。
- 积分抗饱和。
- 离散时间控制。
- 零阶保持。

## 8.3 姿态控制

根据期望加速度和期望偏航角计算期望姿态。

姿态误差使用四元数或 SO(3) 旋转误差。

禁止直接将欧拉角误差作为三个互不相关的控制量。

## 8.4 角速度控制

角速度控制器产生机体力矩指令。

再由六旋翼控制分配矩阵转换为六个旋翼推力指令。

控制分配需要支持推力饱和约束。

PID 增益不允许直接使用未经验证的随意数值。

Codex 必须通过悬停、阶跃响应、巡航和抗扰动实验完成首轮整定。

# 9. 参考航迹生成

本项目需要支持不同类型的参考航迹。

第一阶段至少支持：

1. 恒速直线。
2. 分段速度变化。
3. 平滑加速和减速。
4. 航点连接。
5. 平滑转弯。
6. 用户指定时间序列。

不将简单正弦曲线作为主要航迹生成方法。

## 9.1 默认 SAR 航迹

初始名义参数：

| 参数 | 默认值 |
|---|---:|
| 航迹方向 | X 正方向 |
| 飞行速度 | 10 m/s |
| APC 高度 | 300 m |
| 斜视角 | 0° |
| 主波束方向 | 右视 |
| 波束俯角 | 50° |
| PRF | 800 Hz |
| 脉冲数量 | 16,385 |

有效采集时间：

```text
T = (16385 - 1) / 800
  = 20.48 s
```

航迹长度：

```text
L = 10 * 20.48
  = 204.8 m
```

这些参数应优先从现有 SAR 配置文件读取，而不是在 Python 程序中重复硬编码。

不能修改现有雷达参数。

不能引用旧配置中用户明确要求不参考的：

```text
cfg.motion.amplitudeXYZ_m = [0.04, 0.12, 0.10]
```

不得把该字段映射到新的 A/B 扰动参数。

# 10. 扰动系统

这是本项目的核心功能之一。

## 10.1 扰动类别

只保留以下三类：

### A 类：平移运动扰动

包括：

- X 航向运动。
- Y 横航向运动。
- Z 高度方向运动。

### B 类：姿态及角速度扰动

包括：

- Roll。
- Pitch。
- Yaw。
- 三轴角速度响应。

### E 类：导航位置估计误差

仅模拟：

- X 方向位置估计误差。
- Y 方向位置估计误差。
- Z 方向位置估计误差。

不实现 C、D 类。

不实现：

- 随机风场。
- 电机随机扰动。
- 旋翼故障。
- 姿态估计误差。
- GNSS/IMU 完整传感器模型。
- 其他未选定的 SAR 误差项。

## 10.2 三种扰动注入方式

### 方式一：指令层

在飞控参考输入端注入扰动。

A 类：

- 修改参考位置。
- 修改参考速度。
- 修改参考加速度。

B 类：

- 修改期望 Roll。
- 修改期望 Pitch。
- 修改期望 Yaw。
- 修改期望角速度。

飞控对扰动产生自然响应。

### 方式二：动力学层

直接在六自由度动力学中注入：

A 类：

```text
F_external_B
```

B 类：

```text
tau_external_B
```

力和力矩必须具有明确单位。

不允许把位置幅值直接当作外力幅值。

### 方式三：SAR 几何解耦层

该模式不修改真实刚体动力学。

允许单独选择：

- 理想平移 + 理想姿态。
- 实际平移 + 理想姿态。
- 理想平移 + 实际姿态。
- 实际平移 + 实际姿态。

几何解耦结果单独保存，标记为：

```text
geometry_decoupled
```

不能覆盖原始真实运动状态。

对于独立拼接的平移和姿态序列，必须通过连续插值和微分得到相应速度、加速度及角速度。

禁止简单拼接数组后直接输出。

## 10.3 独立控制接口

每个扰动因素都必须拥有独立配置：

```yaml
enabled: false

injection_mode: command

level: medium

profile: ou

start_time_s: 0.0

end_time_s: 20.48

amplitude: null

random_seed: null
```

实际 YAML 中必须根据不同扰动通道补全具有明确单位的参数。

每个因素可以独立开启、关闭、修改幅度、时间范围和随机种子。

默认等级为 Medium，但所有扰动默认关闭。

# 11. 三级扰动预设

以下为上一版技术方案中的候选初始数值。

它们是工程初始配置，不代表实测飞行统计结果。Codex 应将其放在配置文件中，允许调整，并生成校准报告。

## 11.1 A 类平移扰动

| 扰动因素 | Small | Medium | Large |
|---|---:|---:|---:|
| X 速度变化 | 0.2 m/s | 0.5 m/s | 1.0 m/s |
| Y 参考位置偏移 | 0.05 m | 0.15 m | 0.30 m |
| Z 参考位置偏移 | 0.03 m | 0.10 m | 0.20 m |

注意：

这些数值是参考输入的扰动尺度，不是实际运动误差保证值。

真实运动由动力学和控制器共同决定。

## 11.2 B 类姿态扰动

| 扰动因素 | Small | Medium | Large |
|---|---:|---:|---:|
| Roll | 0.5° | 1.5° | 3° |
| Pitch | 0.5° | 1.5° | 3° |
| Yaw | 1° | 3° | 5° |

角度参数在配置文件中可以使用度，但进入动力学计算前必须转换成 rad。

外力和力矩的三级幅度应单独标定，不能直接套用以上角度和位置幅值。

# 12. 双时间尺度 OU 随机过程

采用双时间尺度 OU 有色随机过程。

目的不是产生完全无规律的白噪声，而是模拟具有短时和长时相关性的运动变化。

模型：

```text
d(t) = sigma * (
    sqrt(0.7) * u_s(t)
    + sqrt(0.3) * u_f(t)
)
```

其中：

- `u_s`：慢时间尺度 OU 过程。
- `u_f`：快时间尺度 OU 过程。

候选初始参数：

```text
tau_slow = 2.5 s
tau_fast = 0.35 s
```

这些参数应可配置，不得写死在动力学函数中。

离散更新：

```text
a = exp(-dt / tau)

u[k+1] =
    a*u[k]
    + sqrt(1-a*a)*normal_random
```

上述形式对应单位稳态方差 OU 过程。

必须保证：

1. 每个轴有独立随机源。
2. 平移和姿态扰动不共用同一随机序列。
3. 两个航次使用独立随机种子。
4. 相同种子和配置可以重复生成相同结果。
5. OU 过程具有正确的均值、方差和自相关特性。
6. 随机过程与动力学积分时间轴严格同步。

不要在每次微分函数调用时随机抽取噪声。

建议在仿真开始前，按照 3200 Hz 时间轴生成可复现的扰动序列，再由不同模块读取。

# 13. 导航位置估计误差

导航误差仅作用于导航位置估计。

模型：

```text
p_nav = p_true + e_nav
```

采用慢变偏差和测量噪声组成的误差模型。

上一版方案的候选初始配置：

| 级别 | 水平慢变标准差 | 垂直慢变标准差 |
|---|---:|---:|
| Small | 0.01 m | 0.02 m |
| Medium | 0.05 m | 0.10 m |
| Large | 0.20 m | 0.40 m |

候选慢变相关时间：

```text
tau_nav = 8.0 s
```

候选 Medium 测量噪声：

```text
sigma_noise = [0.01, 0.01, 0.02] m
```

以上参数均为可配置的初始假设。

导航位置更新频率：

```text
100 Hz
```

导航误差不得反馈到真实动力学控制器。

必须验证：

```text
改变导航误差
    |
    +---- 真实位置不变
    |
    +---- 真实速度不变
    |
    +---- 真实姿态不变
    |
    +---- 真实角速度不变
    |
    +---- 仅导航估计位置变化
```

800 Hz SAR 时间轴上的导航估计位置由 100 Hz 导航数据重采样获得。

不能通过在 800 Hz 上重新生成独立噪声来伪造 800 Hz 导航数据。

# 14. 多速率时间同步

这是强制要求。

## 14.1 频率配置

| 模块 | 更新频率 |
|---|---:|
| 六自由度积分 | 3200 Hz |
| 电机动态 | 3200 Hz |
| 角速度控制 | 200 Hz |
| 姿态控制 | 200 Hz |
| 速度控制 | 50 Hz |
| 位置控制 | 20 Hz |
| 导航位置估计 | 100 Hz |
| SAR 运动输出 | 800 Hz |

## 14.2 整数步长关系

以 3200 Hz 为基础：

```text
3200 / 800 = 4
3200 / 100 = 32
3200 / 200 = 16
3200 / 50  = 64
3200 / 20  = 160
```

所有频率必须通过整数时间步索引调度。

不允许依靠浮点数取模判断控制器更新时间。

控制器输出使用零阶保持，直到下一次控制器更新。

## 14.3 预稳定与采集时间

预稳定：

```text
5 s
```

有效 SAR 采集：

```text
20.48 s
```

时间定义：

```text
预稳定：[-5, 0)

SAR 采集：[0, 20.48]
```

SAR 时间轴：

```text
t[k] = k / 800
```

其中：

```text
k = 0, 1, ..., 16384
```

因此：

```text
N_SAR = 16385
```

## 14.4 数据数量

| 数据 | 频率 | 有效采集样本数 |
|---|---:|---:|
| 动力学状态 | 3200 Hz | 65,537 |
| SAR 状态 | 800 Hz | 16,385 |
| 导航位置估计 | 100 Hz | 2,049 |

包含预稳定时间的完整动力学状态：

```text
N_full = 81,537
```

首末采样点必须严格对齐。

不能因为数组长度相同或时间戳接近就认为已经完成同步。

# 15. 双航次仿真

系统必须支持独立的两次航过。

默认配置：

```text
Pass 1:
    nominal trajectory = common reference
    random seed = seed_1

Pass 2:
    nominal trajectory = common reference
    random seed = seed_2
```

要求：

- 两次航过使用相同名义参考航迹。
- 两次航过使用相同采样频率。
- 两次航过具有独立扰动随机序列。
- 每个航次分别输出完整真实运动状态。
- 每个航次分别输出导航估计位置。
- 分别计算天线相位中心位置。
- 支持设置名义双航次基线。
- 默认名义基线为零。

必须区分：

```text
Nominal Baseline
```

与：

```text
Actual Trajectory Difference
```

不能把无人机两次航过实际运动差异直接等同于人为设置的名义基线。

# 16. 数据输出与 MATLAB 接口

## 16.1 Python 输出结构

建议：

```python
result = {
    "pass1": {},
    "pass2": {},
    "config": {},
    "metadata": {},
    "validation": {}
}
```

每个航次：

```python
pass_result = {
    "truth_full": {},
    "truth_sar": {},
    "navigation": {},
    "navigation_sar": {},
    "sar_geometry": {},
    "reference": {},
    "disturbance_inputs": {}
}
```

## 16.2 必须输出的运动状态

800 Hz SAR 时间轴：

| 字段 | 维度 | 单位 |
|---|---:|---|
| 时间 | N×1 | s |
| 位置 | N×3 | m |
| 速度 | N×3 | m/s |
| 加速度 | N×3 | m/s² |
| 四元数 | N×4 | 无量纲 |
| 欧拉角 | N×3 | rad |
| 角速度 | N×3 | rad/s |
| APC 位置 | N×3 | m |
| 波束方向 | N×3 | 无量纲 |

必须明确标注坐标体系。

建议同时保存内部 NED/FRD 真值与 SAR 外部坐标数据，避免下游误用。

四元数统一采用内部 `q_NB_wxyz`，不能在没有明确标注的情况下把它当成外部欧拉角。

## 16.3 数据格式

优先支持：

- MAT：与 MATLAB SAR 平台直接对接。
- CSV：用于检查和数据分析。

MAT 文件保存：

```text
uav_motion_pass1.mat
uav_motion_pass2.mat
uav_motion_summary.mat
```

建议保留：

- 参数快照。
- 随机种子。
- 软件版本。
- 时间戳。
- 坐标约定。
- 采样频率。
- 扰动模式。
- 真实轨迹。
- 导航估计轨迹。
- SAR 几何状态。
- 验证结果。

不能只保存最终轨迹而不保存生成轨迹所使用的配置。

## 16.4 MATLAB 集成

建立：

```text
matlab/load_uav_trajectory.m
```

读取 Python 输出。

建立：

```text
matlab/verify_sar_geometry.m
```

验证：

1. 800 Hz 时间轴。
2. 位置和速度。
3. 天线相位中心。
4. 天线波束方向。
5. 导航估计轨迹。
6. 两航次状态。

原 SAR 回波生成程序只增加一个运动数据适配器。

不重写原有 SAR 距离压缩、方位压缩、BP、RDA 或 WKA 等处理模块。

不修改原有雷达参数。

# 17. 统一配置文件

所有参数均从 YAML 读取。

示例：

```yaml
project:
  name: AirborneSAR_UAVMotionSim
  version: 1.0

random:
  base_seed: 20260604

vehicle:
  model: HexX12

  mass_kg: 12.0

  inertia_kg_m2:
    - [0.55, 0.0, 0.0]
    - [0.0, 0.60, 0.0]
    - [0.0, 0.0, 1.00]

  arm_length_m: 0.55

  max_rotor_thrust_N: 42.0

  kf: 8.0e-5
  km: 2.4e-6

  motor_time_constant_s: 0.10

simulation:
  dynamics_rate_hz: 3200
  sar_rate_hz: 800
  navigation_rate_hz: 100

  pre_stabilization_s: 5.0
  acquisition_duration_s: 20.48

  integrator: RK4

controller:
  position_rate_hz: 20
  velocity_rate_hz: 50
  attitude_rate_hz: 200
  angular_rate_hz: 200

trajectory:
  type: straight_constant_velocity
  speed_mps: 10.0
  apc_height_m: 300.0

antenna:
  phase_center_frd_m: [0.0, 0.10, 0.20]
  look_angle_deg: 50.0
  right_looking: true
  squint_angle_deg: 0.0

disturbances:
  default_level: medium
  all_enabled: false

  wind_enabled: false
  motor_random_disturbance_enabled: false

  translation:
    enabled: false

  attitude:
    enabled: false

  navigation_position_error:
    enabled: false

output:
  save_mat: true
  save_csv: true
  save_plots: true
```

注意：

这是配置结构示例，不是完整最终配置。

Codex 必须建立完整的字段定义、默认值、单位说明和合法性检查。

配置文件中不得出现无法解释的魔法数字。

# 18. 命令行运行接口

项目最终必须支持命令行运行。

建议：

```powershell
python -m uavsim.cli run `
    --config configs/hex_x_12kg.yaml `
    --passes 2 `
    --output results/
```

单航次：

```powershell
python -m uavsim.cli run `
    --config configs/hex_x_12kg.yaml `
    --passes 1
```

运行测试：

```powershell
python -m pytest
```

生成验证报告：

```powershell
python -m uavsim.cli validate `
    --config configs/hex_x_12kg.yaml
```

绘制三维轨迹：

```powershell
python -m uavsim.cli plot `
    --input results/
```

要求命令执行后能够明确显示：

- 仿真配置。
- 动力学积分频率。
- SAR 输出频率。
- 导航更新频率。
- 运行时间。
- 扰动配置。
- 随机种子。
- 数据输出路径。
- 测试通过情况。

# 19. 科研级测试与验收

不能以程序能够运行作为完成标准。

## 19.1 动力学基础测试

必须完成：

1. 六旋翼悬停。
2. 恒速直线飞行。
3. 速度阶跃。
4. 加减速。
5. 高度变化。
6. 横向运动。
7. Roll 控制。
8. Pitch 控制。
9. Yaw 控制。
10. 电机推力饱和。

初始研发验收目标：

- 稳态速度误差小于 0.1 m/s。
- 稳态高度误差小于 0.05 m。
- 四元数单位模误差满足数值精度要求。
- 不出现 NaN 或 Inf。
- 不出现持续振荡。
- 控制分配结果不违反电机上下限。

这些是仿真验收指标，不代表真实无人机实测性能。

## 19.2 状态一致性

检查：

```text
dp/dt = velocity
```

```text
dv/dt = acceleration
```

```text
dR/dt = R * skew(angular_velocity)
```

使用数值差分和动力学残差交叉验证。

## 19.3 扰动独立性

必须实现单因素测试：

| 测试 | 开启项目 |
|---|---|
| T0 | 所有扰动关闭 |
| T1 | 仅 X 平移 |
| T2 | 仅 Y 平移 |
| T3 | 仅 Z 平移 |
| T4 | 仅 Roll |
| T5 | 仅 Pitch |
| T6 | 仅 Yaw |
| T7 | 仅导航位置误差 |
| T8 | A+B 联合扰动 |
| T9 | 同随机种子重复 |
| T10 | 两航次独立随机种子 |
| T11 | SAR 几何解耦 |

特别是 T7：

改变导航位置估计误差时，真实运动状态必须完全保持不变。

## 19.4 随机过程验证

验证：

- OU 均值。
- OU 方差。
- 自相关函数。
- 功率谱密度。
- 随机种子复现性。
- 不同随机通道独立性。

统计验证必须使用足够长的模拟序列或多次 Monte Carlo 实验。

不能仅凭一次 20.48 秒仿真就认定所有 OU 参数均已准确标定。

## 19.5 数值收敛验证

分别测试：

- 1600 Hz。
- 3200 Hz。
- 6400 Hz。

三种频率必须使用相同的连续扰动输入或同一公共高精度扰动序列。

比较：

- 位置。
- 速度。
- 加速度。
- 姿态。
- 角速度。
- 天线相位中心。
- SAR 目标斜距。
- 双程传播相位。

初始验收目标：

3200 Hz 积分结果相对于更高精度参考结果，天线斜距差 RMS 不超过 1 mm。

该指标用于检查数值积分精度，不代表实际无人机运动精度达到 1 mm。

## 19.6 SAR 时间同步

必须满足：

```text
SAR samples = 16385
```

每一个雷达脉冲必须对应唯一的运动状态。

不得通过低频插值、重复数据或数组补齐来伪造 800 Hz 状态。

## 19.7 天线几何验证

必须验证：

1. APC 杆臂正确。
2. 右视方向正确。
3. 俯角 50° 正确。
4. 机体姿态变化后的 APC 位置正确。
5. 机体姿态变化后的波束方向正确。
6. 无扰动 APC 航迹与现有 SAR 理想航迹一致。

## 19.8 MATLAB 兼容性

至少进行一次完整测试：

```text
Python 仿真
    |
    v
MAT 文件输出
    |
    v
MATLAB 加载
    |
    v
800 Hz 时间轴检查
    |
    v
APC 几何检查
    |
    v
SAR 回波程序读取
```

只有全部通过，才认为接口开发完成。

# 20. 分阶段开发计划

Codex 不允许一次性生成所有代码。

必须按以下阶段实现。

## P0：环境部署和项目初始化

任务：

- 检查 Python。
- 建立虚拟环境。
- 建立目录。
- 创建 pyproject.toml。
- 安装基础依赖。
- 创建测试框架。
- 阅读 RotorPy 代码和相关论文。
- 检查现有 MATLAB SAR 配置。

交付：

- README。
- 环境检查报告。
- 项目结构。
- 依赖清单。

验收：新环境可以正常安装，测试框架可以运行。

## P1：坐标和几何模块

任务：

- 内外坐标转换。
- 旋转矩阵。
- 四元数。
- 机体坐标。
- 天线 APC。
- 波束方向。

交付：

- 坐标转换模块。
- 姿态工具。
- 天线几何模块。
- 单元测试。

验收：全部坐标和几何测试通过。

## P2：六旋翼物理模型

任务：

- Hex-X 布局。
- 旋翼推力。
- 反扭矩。
- 电机动态。
- 控制分配。
- 六自由度动力学。
- RK4 积分器。

交付：可运行的 12 kg 六旋翼物理模型。

验收：悬停与基础动力学测试通过。

## P3：串级飞控

任务：

- 位置控制。
- 速度控制。
- 姿态控制。
- 角速度控制。
- 控制器多速率调度。
- PID 参数整定。

交付：可稳定跟踪参考航迹的六旋翼仿真器。

验收：直线巡航和基础轨迹跟踪测试通过。

## P4：扰动系统

任务：

- A 类平移扰动。
- B 类姿态扰动。
- 三种注入模式。
- 双时间尺度 OU。
- Small/Medium/Large。
- 独立开关和随机种子。

交付：完整扰动系统。

验收：单因素和组合测试通过。

## P5：导航与双航次

任务：

- 100 Hz 导航位置估计。
- 导航位置误差。
- 误差重采样。
- 双航次运动生成。
- 独立随机种子。

交付：双航次运动状态。

验收：导航误差与真实运动严格隔离。

## P6：SAR 接口

任务：

- 800 Hz 时间同步。
- 天线 APC。
- 波束方向。
- MAT 导出。
- CSV 导出。
- MATLAB 读取程序。

交付：完整 SAR 运动数据接口。

验收：16385 个脉冲均有正确运动状态。

## P7：科研级验证

任务：

- 动力学残差。
- 随机过程统计。
- 数值收敛。
- 运动状态对比。
- 双航次差异。
- SAR 几何验证。

交付：自动化科研验证报告。

验收：所有强制测试通过。

## P8：独立飞控交叉验证

任务：

- 部署 ArduPilot SITL。
- 运行 Hexa 机型。
- 建立参考航迹。
- 导出飞行日志。
- 对比轨迹和控制响应。

交付：独立飞控验证报告。

这一阶段不影响 Python 核心系统的独立运行。

# 21. 最终交付要求

Codex 完成后必须交付：

1. 完整 Python 工程。
2. 完整的 12 kg Hex-X 参数配置。
3. 六自由度动力学模块。
4. 六旋翼控制分配模块。
5. 电机动态模块。
6. 串级飞控模块。
7. A/B/E 扰动系统。
8. 双时间尺度 OU 模型。
9. 导航位置误差模型。
10. 双航次仿真程序。
11. 3200 Hz 运动真值输出。
12. 800 Hz SAR 状态输出。
13. 100 Hz 导航位置估计。
14. 天线相位中心计算模块。
15. MATLAB 数据读取接口。
16. 三维轨迹可视化。
17. 运动状态分析图。
18. 随机过程统计分析。
19. 自动化测试套件。
20. 数值收敛报告。
21. README 和技术文档。
22. 一键运行示例。

所有参数必须有单位。

所有状态必须有坐标系定义。

所有输出必须有时间戳。

所有随机过程必须可复现。

# 22. 给 Codex 的具体执行指令

以下要求具有最高优先级。

### 22.1 开发原则

- 先阅读本文件，再开始编码。
- 先检查现有本地项目，不允许覆盖原有 SAR 工程。
- 先建立最小可运行架构，再逐步扩展。
- 每完成一个阶段，必须运行对应测试。
- 每个模块必须提供独立接口。
- 每个参数必须能够配置。
- 不允许使用未经验证的物理参数冒充实测参数。
- 不允许为了通过测试而删除物理约束。
- 不允许用简单随机曲线替代六自由度动力学。
- 不允许把导航估计误差混入真实状态。
- 不允许擅自修改原 SAR 雷达参数。

### 22.2 关于参考项目

必须先阅读 RotorPy 的源代码，形成简短的代码研究报告。

报告至少回答：

1. RotorPy 的状态变量如何定义？
2. 它如何实现六自由度运动方程？
3. 如何处理旋翼推力和力矩？
4. 如何处理电机动态？
5. 如何实现控制器？
6. 哪些代码结构可以借鉴？
7. 哪些模块不符合本项目要求？
8. 如何建立 Hex-X 六旋翼模型？

不要简单复制一个四旋翼模型并把旋翼数量从 4 改为 6。

必须重新检查控制分配矩阵、旋翼编号和力矩方向。

### 22.3 关于代码质量

- 使用类型注解。
- 使用明确的数据结构。
- 使用 Pydantic 或等效配置验证。
- 使用统一异常处理。
- 使用独立随机数生成器。
- 使用自动化测试。
- 使用 Git 管理版本。
- 关键算法必须有中文注释。
- 关键接口必须有输入输出说明。
- 不允许大型函数承担多个不相关职责。

### 22.4 关于每个阶段的报告

每个阶段结束后，必须输出：

```text
阶段名称：

1. 已完成的模块
2. 新增和修改的文件
3. 核心算法说明
4. 已运行的测试
5. 测试结果
6. 尚未解决的问题
7. 下一阶段计划
```

如果某个测试未通过，必须先定位并解决问题。

不能跳过失败测试进入下一阶段。

### 22.5 关于最终验收

Codex 不得只报告：

```text
程序运行成功。
```

必须提供：

- 实际运行截图或图表。
- 三维轨迹。
- 速度变化图。
- 加速度变化图。
- Roll/Pitch/Yaw 图。
- p/q/r 角速度图。
- A/B 扰动输入图。
- 导航位置估计误差图。
- OU 自相关分析。
- 双航次轨迹对比。
- SAR 天线相位中心轨迹。
- 800 Hz 时间同步检查。
- 数值收敛测试。
- 完整自动化测试结果。

必须能够从固定配置和随机种子重新生成相同的仿真结果。

---

# 23. 项目最终完成定义

当以下条件全部满足时，项目才算完成：

**第一：物理模型**

具有完整六自由度刚体动力学、六旋翼控制分配、电机动态和串级飞控。

**第二：扰动系统**

A/B/E 三类扰动具有独立控制接口，支持三种注入方式和双时间尺度 OU 随机过程。

**第三：运动状态**

能够输出完整的位置、速度、加速度、姿态和角速度。

**第四：时间系统**

实现 3200 Hz 动力学积分、800 Hz SAR 运动状态和 100 Hz 导航位置估计。

**第五：SAR 几何**

能够正确计算固定天线安装位置、天线相位中心和右视波束方向。

**第六：双航次**

能够生成具有共同名义航迹、独立运动扰动的两次飞行数据。

**第七：数据接口**

能够将运动数据导出为 MATLAB 可以直接读取的 MAT 文件。

**第八：科研验证**

完成动力学、控制器、扰动、随机统计、数值收敛和 SAR 时间同步验证。

最终系统应成为一个独立的、可复用的机载 SAR 无人机运动仿真子平台，而不是一次性的航迹生成脚本。

---

# 24. 参考资料

1. RotorPy：  
   https://github.com/spencerfolk/rotorpy

2. RotorPy 论文：  
   https://arxiv.org/abs/2306.04485

3. MathWorks Quadcopter Drone Model：  
   https://github.com/mathworks/Quadcopter-Drone-Model-Simscape

4. MathWorks Simscape 官方示例：  
   https://www.mathworks.com/help/simscape-electrical/ug/quadcopter-drone.html

5. ArduPilot SITL：  
   https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html

6. ArduPilot Gazebo：  
   https://ardupilot.org/dev/docs/sitl-with-gazebo.html

7. XTDrone：  
   https://github.com/robin-shaun/XTDrone

8. XTDrone 论文：  
   https://arxiv.org/abs/2003.09700

---

**文档结束**

**版本：V1.0 — Codex 本地部署与实现规范**

**实施原则：先建立正确的物理模型，再增加扰动，最后连接 SAR 仿真平台。**
