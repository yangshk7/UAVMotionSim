# RotorPy 源码研究

> 版本口径：最终版参数、功能边界与验收结论统一以 [v1.0.0发布说明](FINAL_RELEASE.md) 为准。本文带日期的运行数字是对应历史实验记录，不代表最终版重新测得的结果。

阅读版本：`40b09697a6a1596d404a67f459941d46a39eedd3`，检索日期 2026-10-30。
原始参考文本保留在 `docs/references`。此页记录原版的研究结论；OpenSource 版本现通过 `uavsim.dynamics.rotorpy_hex` 实际调用 RotorPy 3.0.0 的多旋翼动力学 RHS。仍未复制 RotorPy 实现到项目源码。

1. 状态：位置、速度、xyzw 四元数、机体角速度、风速和旋翼转速；本项目使用 wxyz，去掉风速状态，六旋翼共 19 维。
2. 运动方程：Newton–Euler，R 将机体系向量转至世界系，角动力学包含陀螺耦合；本项目改为 NED/FRD 重力正 Z、旋翼推力负 Z。
3. 旋翼：转速平方给出推力/反扭矩，杆臂叉乘给出滚转和俯仰力矩。不能直接照搬其推力正 Z 符号。
4. 电机：一阶转速滞后，步后限幅；源码还有电机随机噪声，本项目明确排除该噪声。
5. 控制器：SE(3) 几何姿态误差及加速度前馈。本项目使用独立多速率串级环，增益须经自身测试。
6. 借鉴：纯动力学、执行器、参考轨迹和仿真调度分层；记录输出与控制输入分开。
7. 不采用：风场、传感器、地面接触、相机、Torch 批量依赖；原控制器使用方阵求逆，不适用于 4×6 六旋翼分配。
8. Hex-X：重新建立六个 30°+60°i 杆臂；分配列为 [1,-y_i,x_i,s_i km/kf]，采用有界最小二乘并验证饱和。

论文已阅读建模和积分章节。文章中的气动效应与实机对比不能作为本项目参数已校准的证据。

来源：[动力学源码](https://github.com/spencerfolk/rotorpy/blob/40b09697a6a1596d404a67f459941d46a39eedd3/rotorpy/vehicles/multirotor.py)、[控制源码](https://github.com/spencerfolk/rotorpy/blob/40b09697a6a1596d404a67f459941d46a39eedd3/rotorpy/controllers/quadrotor_control.py)、[原论文](https://arxiv.org/html/2306.04485v1)。
