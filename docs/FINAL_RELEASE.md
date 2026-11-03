# Final release 2026-11-03

本版作为用户验收后的功能冻结版本，发布标签为 `v1.0.0`。

## 使用入口

- Windows上位机：`main/UAVMotionSim.exe`，或`main/open_control_panel.cmd`。
- 完整标准验收：`main/run.cmd`。
- 详细操作：[上位机使用说明](../main/docs/CONTROL_PANEL_USER_GUIDE.md)。

交付压缩包包含源代码、配置、Windows EXE启动器和锁定依赖；新电脑先在项目根目录运行`scripts/setup.ps1`安装Python及uv依赖，然后启动EXE。EXE依赖完整项目与`.venv`，不属于单文件免安装版本。MATLAB、WSL飞控及Gazebo属于外部环境，未捆入源码包。

默认目标1000 m、10 m/s、10 s裕量；真值3200 Hz、导航100 Hz、SAR800 Hz。六轴扰动small，A XYZ各0.05 m，B Roll/Pitch各0.5°、Yaw1°。导航使用加计/陀螺误差传播与GNSS校正；适用范围见[模型说明](gnss_ins_model.md)。SAR成像端到端验证和实机标定仍不在本次完成范围内。

## 结果管理

交付源码包排除运行环境、缓存、历史实验和大体积MAT/CSV。最终验收报告另行打包；复现时使用报告内参数快照和版本标签。项目内保留最终正式验收及选定完整航线结果，旧运行记录集中移至项目旁`UAVMotionSim_Archive_20261103_Final`，不删除原始实验数据。历史文档中旧运行路径可在归档目录的对应相对路径下查找。

最终版后仍可正常另存配置及生成新结果；如修改源代码，建议建立后续版本并重新验收。

最终实测结论及报告位置见[最终验收记录](FINAL_VALIDATION.md)。
