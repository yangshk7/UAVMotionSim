# 坐标、姿态与天线

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.1.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

内部 N 为 X 前/Y 右/Z 下的 NED 型局部右手系，B 为 X 机头/Y 机右/Z 机下的 FRD。`R_NB` 是 B→N 主动旋转；`q_NB_wxyz` 标量在前。欧拉角定义 `R=Rz(yaw)Ry(pitch)Rx(roll)`，单位 rad，仅作接口和显示，不直接用于姿态误差控制。

外部 SAR 为 X 前/Y 右/Z 上，是左手系。令 `C=diag(1,1,-1)`：极向量位置/速度/加速度满足 `p_SAR=C p_N`；两端机体/导航都反射时 `R_SAR=C R_NB C`。角速度是轴向量，若需要反射表示，使用 `omega_SAR=det(C)C omega_N`。当前主要输出仍保留 FRD 体轴角速度，避免误用。

固定杆臂 `l_B=[0,0.1,0.2]` m，波束 `b_B=[0,cos(50°),sin(50°)]`。APC 与波束满足：

\[
p_A=p_G+Rl_B,\quad v_A=v_G+R(\omega_B\times l_B),\quad b_N=Rb_B.
\]

\[
a_A=a_G+R(\dot\omega_B\times l_B+\omega_B\times(\omega_B\times l_B)).
\]

默认 APC 的 SAR Z=300 m，而水平零姿态质心 SAR Z=300.2 m；质心 Y=-0.1 m。APC 高度不能与质心高度混淆。参考航迹的 CG 位置、速度和加速度均扣除相应旋转杆臂项。

外部欧拉角字段 `euler_sar_xyz_rad` 是对 `C R C` 的矩阵参数化，不应与 FRD Roll/Pitch/Yaw 混读；主要导航式姿态字段使用明确标注的内部 NED/FRD 定义。
