# 公开实飞轨迹与独立真值对照

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.0.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

2026-11-01 使用 [NTU VIRAL 官方数据页](https://ntu-aris.github.io/ntu_viral_dataset/)的 `eee_01` 航段完成导航误差评估方法验证。输入是作者公开的 [Leica 测量棱镜真值 CSV](https://huggingface.co/datasets/Yuan-Shenghai/NTU-VIRAL-DATASET/tree/main/ground_truth/eee_01)和[官方 FAST-LIO2 示例估计轨迹](https://github.com/ntu-aris/fastlio2_sample/tree/master/result_eee_01)。原始文件保存在 `results/public_data/ntu_viral_eee_01/`，各自 SHA256 见 `evaluation.json`。本数据来自其他无人机，未用于标定本项目 12 kg Hex-X。

该航段使用独立激光跟踪位置真值，不是 RTK/PPK。NTU 官方[评估教程](https://ntu-aris.github.io/ntu_viral_dataset/evaluation_tutorial.html)指出真值位于测量棱镜，机体原点到棱镜的平移为 `[-0.293656, -0.012288, -0.273095] m`；必须用估计姿态旋转该杆臂，再与棱镜位置比较。真值 CSV 的姿态列全为单位四元数，且官方说明没有姿态真值，所以本报告只评估位置。

## 处理与结果

1. 使用 ROS 纳秒时间戳。对每个估计点仅在前后两条真值记录都相距不超过 0.1 s 时作线性插值，超出范围不外推。
2. 将 FAST-LIO2 机体位置转为棱镜位置；再以无尺度的三维刚体旋转和平移对齐两个坐标系。这是标准位置 ATE 流程，不声称全局绝对定位误差。
3. 在全航段对齐后计算误差；另用前半段确定对齐变换、仅在后半段计算误差，检查全段拟合带来的乐观偏差。

| 指标 | 结果 |
|---|---:|
| 真值 / 估计原始样本 | 6,616 / 3,984 |
| 有效配对样本与持续时间 | 3,348 / 336.31 s |
| 全段对齐后三维 ATE RMS | 0.0686 m |
| 全段对齐后 95% 分位三维误差 | 0.1136 m |
| 前半段对齐、后半段验证的三维 ATE RMS | 0.0722 m |
| 姿态误差 | 不可计算：无独立姿态真值 |

运行：

```powershell
uv run --no-sync python deployment/evaluate_public_ntu_viral.py results/public_data/ntu_viral_eee_01/ground_truth.csv results/public_data/ntu_viral_eee_01/odometry.csv --output results/public_data/ntu_viral_eee_01/evaluation.json
```

产出：`evaluation.json` 保存来源哈希、参数与统计；`aligned_positions.csv` 保存逐点时间、两路位置和差值；`trajectory_error.png` 是中文实飞航迹与误差图。时间配对没有额外估计通信/算法延迟；全段刚体对齐移除了初始坐标差，且会降低报告误差，所以同时保留后半段检查。这些是 NTU VIRAL 上 FAST-LIO2 示例轨迹相对于 Leica 真值的结果，不是 PX4/ArduPilot 或本项目 Hex-X 的实机精度。

MUN-FRL 的同步 PPK 数据仍可用于后续 RTK/PPK 特定流程复核；其完整航段较大，本次选择可独立下载、带激光跟踪真值及同航段估计轨迹的 NTU VIRAL 数据完成第三项的方法验证。
