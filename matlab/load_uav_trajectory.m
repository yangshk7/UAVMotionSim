function motion = load_uav_trajectory(inputDirectory)
%LOAD_UAV_TRAJECTORY 读取 Python MAT v5。时间 s，位置 m，速度 m/s。
% 输入：输出运行目录。输出：summary、pass1/pass2...；内部四元数 wxyz。
arguments
    inputDirectory (1,:) char
end
files=dir(fullfile(inputDirectory,'uav_motion_pass*.mat'));
assert(~isempty(files),'No UAV motion MAT files found.');
summary=load(fullfile(inputDirectory,'uav_motion_summary.mat'),'result');
motion.summary=summary.result;
for index=1:numel(files)
    token=regexp(files(index).name,'uav_motion_(pass\d+)\.mat','tokens','once');
    loaded=load(fullfile(files(index).folder,files(index).name),'pass_result');
    motion.(token{1})=loaded.pass_result;
end
end
