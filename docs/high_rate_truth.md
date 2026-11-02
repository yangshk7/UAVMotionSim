# 外部仿真物理真值输出频率实测

2026-11-02 在本机已部署的 12 kg Hex-X ArduPilot SITL 与 PX4/Gazebo 上分别测试独立物理真值记录。每档新建运行目录，不覆盖已有航线。下表频率由**记录中的独立真值点和仿真时长**计算，不把重采样或导航估计当作物理真值。

| 平台 | 请求频率 | 实测真值频率 | 结果 |
|---|---:|---:|---|
| SITL | 200 Hz | 约 200 Hz | 短时起降通过 |
| SITL | 300 / 333 Hz | 约 333 Hz | 两档短时起降通过；333 Hz 完整采集航线通过 |
| SITL | 334 / 350 / 400 / 800 Hz | 0 | 短时飞行仍可进行，但 `SIMSTATE` 真值流为空；按真值测试失败处理 |
| PX4/Gazebo | 200 Hz | 200 Hz | 短时起降通过，5 ms 间隔 |
| PX4/Gazebo | 800 Hz | 500 Hz | 短时起降通过，实际 2 ms 间隔；不是 800 Hz |
| PX4/Gazebo | 1000 Hz | 1000 Hz | 短时起降与完整采集航线通过，1 ms 间隔 |

SITL 的最高**已验证有效请求**是 333 Hz。完整 20.50 s 水平采集段按导航接收时间定位后，数到 6833 个真值消息，约 333.3 Hz；航线前向位移 205.097 m，降落停桨检查通过。真值消息只有接收时间，没有独立的仿真时间戳，因此这里的采集段频率是用导航时钟定位的平均值；不能证明每个真值点严格相隔 3 ms。请求 334 Hz 时真值流为空，说明当前 MAVLink 输出通道存在约 3 ms 的实际边界，尚未确认是协议实现还是本次固件的调度限制。

补充：上述限制特指 **MAVLink `SIMSTATE` 消息流**。同一次 SITL 飞行的 DataFlash `SIM2` 日志另有 `TimeUS`，采集段原生位置/速度记录约 400 Hz，并已与导航共同时间轴验证；详见[外部时钟对齐报告](external_clock_alignment.md)。`SIM` 真值姿态频率更低，不能据位置频率推断姿态也为 400 Hz。

PX4/Gazebo 的世界物理步长为 1 ms。原 `/world/default/dynamic_pose/info` 场景广播约 56 Hz；新增 Gazebo 官方 `PosePublisher` 插件，只发布本机模型位姿到 `/uav_motion/truth_pose`，保留仿真时间戳。完整 20.495 s 水平采集段内记录 20496 个真值点，相邻时间戳均为 1 ms，平均 1000 Hz；前向位移 208.050 m，降落停桨检查通过。1000 Hz 是**当前 1 ms 物理步长与本次已测试配置**的上限，未测试缩小物理步长。PX4/ROS 时间同步日志有回跳警告，Gazebo 自身时间戳连续并不等于导航/真值跨系统时间对齐已校准。

Gazebo 插件的 `update_frequency` 参数定义见 [官方 PosePublisher 文档](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1PosePublisher.html)。

以后默认运行完整航线，即请求 SITL 300 Hz、PX4/Gazebo 1000 Hz：

```powershell
.\deployment\run_sitl_route.ps1
.\deployment\run_xtdrone_route.ps1
```

`main/run_external_route.cmd` 和短时验收入口也使用这两个默认请求值。SITL 请求 300 Hz 在当前固件中实际得到约 333 Hz；不把请求值写成实测频率。需要复现实测上限时可显式传入 `-TruthRateHz 333`。更高频率增加真值文件体积和记录负载。导航估计消息仍按原飞控链路输出；上表不是导航解频率。1000 Hz Gazebo 真值可以在 SAR 800 Hz 脉冲时刻做时间重采样，但重采样点需标为插值，不是原生的 800 Hz Gazebo 输出。

证据：SITL 完整航线 `results/frequency_probe/sitl_route/sitl_20261102_095735_133880/`；PX4/Gazebo 完整航线 `results/xtdrone_route/xtdrone_20261102_095933_737941/`。两目录均保存 `validation.json` 和原始真值 CSV；PX4 另存 `model_snapshot.sdf`、`world_snapshot.sdf`。短时档位结果分别在 `results/frequency_probe/sitl/` 和 `results/xtdrone/`。
