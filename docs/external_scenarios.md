# 外部平台风扰与传感器场景

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.1.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

`deployment/scenarios/crosswind_sensor.json` 是可调的工程试验场景。它只改变 ArduPilot SITL 和 PX4/Gazebo 的外部仿真；原始基线配置及已有结果目录不会被覆盖。场景文件与当次模型、验证报告一起归档。

| 条目 | ArduPilot SITL | PX4/Gazebo |
|---|---|---|
| 水平风 | 2 m/s，从东吹来，`SIM_WIND_SPD=2`、`SIM_WIND_DIR=90` | ENU 风速 `[-2, 0, 0]` m/s，`WindEffects` 对机体施力 |
| 风变化 | `SIM_WIND_TURB=0.2` | 幅值为平均风速 10% 的 8 s 正弦变化 |
| 气动系数 | 本次模型副本 `mdrag_coef=0.1` | 风力近似比例 `0.1` |
| IMU | 三路 `SIM_ACC{1,2,3}_RND=0.05` m/s² | 继承 PX4 模型的 IMU 噪声与动态偏置标准差乘 2 |

这些值尚无本机型实测依据，两个平台的风力实现和气动模型也不同；同名场景不等于逐时刻相同的外力。ArduPilot 4.7.2 的加速度参数采用 `ACC1_RND` 等分通道名称，不能使用旧文档的 `SIM_ACC_RND`。其启动器没有暴露固定随机种子；JSON 的 `seed` 在 SITL 报告中仅作场景标识。Gazebo 使用该种子启动，但多进程时序未验证为逐点确定性。

运行：

```powershell
.\deployment\run_sitl_route.ps1 -Scenario deployment/scenarios/crosswind_sensor.json
.\deployment\run_xtdrone_route.ps1 -Scenario deployment/scenarios/crosswind_sensor.json
uv run --no-sync python deployment/create_report.py --route
```

每次运行在 `results/sitl_route/` 或 `results/xtdrone_route/` 下创建新目录。检查 `validation.json` 的 `passed`、`scenario`、`route_*` 和 `navigation_truth_comparison.json`；ArduPilot 还记录实际读回的 `scenario_parameters_confirmed`。PX4 归档 `scenario_world.sdf`、`scenario_model.sdf`，以便复核实际生效的风与噪声定义。报告页仍以导航航迹为主，真值对照属于仿真内部比较，不能当作实机导航精度。

## 2026-11-01 实际闭环对照

原始对照和各运行目录保存在 `results/scenario_review/crosswind_sensor_comparison.json`。两平台扰动航线、落地和停桨检查均通过；验收页为 `results/external_route_review/review_20261101_122512_511446/index.html`。

| 平台与指标 | 无扰动基线 | 本场景 |
|---|---:|---:|
| SITL 采集段导航横向 RMS | 0.0206 m | 0.0202 m |
| SITL 采集段仿真真值横向 RMS | 0.0318 m | 0.0295 m |
| PX4 采集段导航横向 RMS | 0.1935 m | 0.7579 m |
| PX4 采集段 Gazebo 真值横向 RMS | 0.1241 m | 0.8945 m |
| PX4 起飞前偏移校正后的导航/真值 Z 差 RMS | 0.3125 m | 0.5703 m |

上述结果来自两次独立运行，不是同一随机过程的配对实验。场景同时改变风、阻力和传感器噪声，不能将数值变化单独归因于某一个参数。SITL 在这一组条件下的横向指标变化很小；PX4 的真值与导航横向偏移都明显增大，符合受风后物理航迹改变的现象。PX4 时间同步发生短时回跳，真值对照从本次原始流中排除了 5 个位置、7 个姿态时间戳离群样本；采集段 1951 个导航点中 1950 个在 20 ms 内成功匹配 Gazebo 真值。未校正估计器与通信延迟。
SITL 本次日志显示建议的 `EK3_DRAG_MCOEF=0.062`，基线为 `0.000`，说明本次模型副本的动量阻力参数参与了动力学计算；横向误差仍受闭环控制约束。

参考实现：[ArduPilot SITL 参数](https://ardupilot.org/dev/docs/SITL_simulation_parameters.html)、[ArduPilot 机型阻力实现](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_Frame.cpp)、[Gazebo WindEffects](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WindEffects.html)、[Gazebo 风场示例](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/wind.sdf)。
