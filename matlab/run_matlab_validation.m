function report = run_matlab_validation(inputDirectory,sarProjectDirectory)
%RUN_MATLAB_VALIDATION 完整时间轴检查，并将运动适配器接入现有单点回波函数。
% 不写入外部 SAR 工程；不保存大体积原始回波。
report=verify_sar_geometry(inputDirectory);
if nargin>=2 && ~isempty(sarProjectDirectory)
    previousPath=path;
    cleanup=onCleanup(@() path(previousPath)); %#ok<NASGU>
    addpath(sarProjectDirectory);
    cfg=lband.default_config();
    motion=load_uav_trajectory(inputDirectory);
    track=uav_to_lband_track(motion.pass1,cfg,'truth');
    scene=lband.make_scene(cfg);
    [raw,truth]=lband.simulate_echo(cfg,track,scene.centerXYZ_m,1);
    expectedRange=vecnorm(track.actualXYZ_m-scene.centerXYZ_m,2,2);
    assert(max(abs(truth.range_m-expectedRange))<1e-9);
    assert(all(isfinite(raw),'all'));
    assert(any(abs(raw)>0,'all'));
    report.echo=struct('range_samples',size(raw,1),'pulses',size(raw,2),...
        'range_max_error_m',max(abs(truth.range_m-expectedRange)),'passed',true,...
        'limitation','Existing beam_weight uses fixed nominal beam; exported body beam is not consumed.');
    clear raw;
end
fid=fopen(fullfile(inputDirectory,'matlab_validation.json'),'w','n','UTF-8');
assert(fid>=0);
cleanupFile=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));
end
