# 发布说明 v1.1.0（2026-11-11）

当前功能冻结版本为 **v1.1.0**，以[最新正式验收记录](FINAL_VALIDATION.md)为准。

## 入口与能力

- `main/UAVMotionSim.exe`或`main/open_control_panel.cmd`：上位机。
- “运行仿真”：单套链路参数试验；“完整验收”：原所选链路流程；“全链路验收”：RotorPy、MATLAB、内置CPU、SITL、PX4固定双航次与适用检查，汇总报告及交叉指标。
- `main/run.cmd`：保留20.48 s标准接口验收。日常上位机默认1000 m、10 m/s、10 s裕量，即110 s采集。
- 详细操作见[上位机说明](../main/docs/CONTROL_PANEL_USER_GUIDE.md)，新机器安装见[复现说明](REPRODUCIBILITY.md)。

新增MATLAB集成物理核心、两套外部飞控上位机适配、原生时钟对齐、统一导航输出与全链路验收。修正MATLAB姿态扰动期间冻结水平反馈的问题。内置/RotorPy/MATLAB默认真值3200 Hz；SITL标准导出300 Hz、PX4/Gazebo1000 Hz，保留原生来源和插值说明；导航100 Hz。

上位机EXE是项目内启动器，依赖完整源码和uv环境，不是单文件免安装软件。MATLAB需用户许可，WSL平台独立安装；没有把它们伪装成已包含在EXE里。外部A/B扰动及Python控制器参数未映射，适用范围在界面、配置和报告中明确记录。SAR成像端到端、实机标定不在本版完成范围。

## 发布与结果管理

GitHub保存源代码、uv锁、精确外部源码提交/兼容补丁、部署/检查脚本、验收配置和精简JSON/XML证据。源码ZIP由版本标签生成，不包含虚拟环境、缓存和大体积运行数据。完整原始数据在本机保留；报告包只含离线页面、图表和检查文件。

项目内保留最新正式验收；其余历史产出移至项目旁 `UAVMotionSim_Archive_v1_1_0_20261111/`，路径映射见其中archive_manifest.json。未删除原始实验数据或用户研究配置。

历史v1.0.0发布说明见[归档](history/RELEASE_v1_0_0.md)。
