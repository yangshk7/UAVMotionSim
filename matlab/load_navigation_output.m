function [navigation,navigationTruth,metadata] = load_navigation_output(matFile)
%LOAD_NAVIGATION_OUTPUT 直接读取 100 Hz 导航式位姿，不需要加载完整动力学日志。
% 位置/速度为带独立估计误差的局部 SAR 分量；q_NB_wxyz 为含 SO(3) 误差的姿态。
% truth_omega_B_radps / truth_acceleration_sar_mps2 明确为无传感器误差的真值通道。
data=load(matFile,'navigation','navigation_truth','metadata');
navigation=data.navigation;
navigationTruth=data.navigation_truth;
metadata=data.metadata;
assert(size(navigation.q_NB_wxyz,2)==4);
assert(numel(navigation.time_s)==size(navigation.position_sar_m,1));
assert(max(abs(vecnorm(navigation.q_NB_wxyz,2,2)-1))<1e-12);
end
