function report = verify_sar_geometry(inputDirectory)
%VERIFY_SAR_GEOMETRY MATLAB 独立核验整数时间索引、杆臂、波束及导航采样。
motion=load_uav_trajectory(inputDirectory);
cfg=motion.summary.config;
names=fieldnames(motion);
for index=1:numel(names)
    name=names{index};
    if ~startsWith(name,'pass'), continue; end
    data=motion.(name);
    s=data.truth_sar;
    % MAT 中 Python int 导出为 int64；MATLAB 混合整数除法会量化时间，先转 double。
    rate=double(cfg.simulation.sar_rate_hz);
    expected=(0:round(cfg.simulation.acquisition_duration_s*rate))'/rate;
    assert(isequal(s.time_s,expected),'SAR time grid mismatch.');
    assert(isequal(double(s.pulse_index),(0:numel(expected)-1)'));
    assert(all(diff(double(s.dynamics_sample_index))==double(cfg.simulation.dynamics_rate_hz)/rate));
    dynamicsIndices=double(s.dynamics_sample_index)+1;
    assert(isequal(s.position_N_m,data.truth_full.position_N_m(dynamicsIndices,:)));
    q=s.q_NB_wxyz;
    assert(max(abs(vecnorm(q,2,2)-1))<1e-12);
    lever=cfg.antenna.phase_center_frd_m(:);
    dep=deg2rad(cfg.antenna.look_angle_deg);
    squint=deg2rad(cfg.antenna.squint_angle_deg);
    beam=[cos(dep)*sin(squint);cos(dep)*cos(squint);sin(dep)];
    apc=zeros(size(s.position_N_m));
    apcVelocity=apc;
    direction=apc;
    maxOrthogonality=0;
    for k=1:numel(expected)
        w=q(k,1); x=q(k,2); y=q(k,3); z=q(k,4);
        R=[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y);...
           2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x);...
           2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];
        apc(k,:)=s.position_N_m(k,:)+(R*lever)';
        apcVelocity(k,:)=s.velocity_N_mps(k,:)+(R*cross(s.omega_B_radps(k,:)',lever))';
        direction(k,:)=(R*beam)';
        maxOrthogonality=max(maxOrthogonality,max(abs(R'*R-eye(3)),[],'all'));
        assert(abs(det(R)-1)<1e-12);
    end
    apcSar=apc.*[1 1 -1];
    beamSar=direction.*[1 1 -1];
    assert(max(abs(apcSar-s.apc_position_sar_m),[],'all')<1e-9);
    assert(max(abs(apcVelocity-s.apc_velocity_N_mps),[],'all')<1e-9);
    assert(max(abs(beamSar-s.beam_sar_unit),[],'all')<1e-12);
    assert(max(abs(vecnorm(s.beam_sar_unit,2,2)-1))<1e-12);
    stride=rate/double(cfg.simulation.navigation_rate_hz);
    assert(stride==round(stride),'Navigation/SAR ratio must be integer for this check.');
    assert(max(abs(data.navigation_sar.position_sar_m(1:stride:end,:)-data.navigation.position_sar_m),[],'all')<1e-9);
    assert(max(abs(data.navigation_sar.apc_position_sar_m(1:stride:end,:)-data.navigation.apc_position_sar_m),[],'all')<1e-9);
    [nav,navTruth]=load_navigation_output(fullfile(inputDirectory,['navigation_' name '.mat']));
    navExpected=(0:round(cfg.simulation.acquisition_duration_s*double(cfg.simulation.navigation_rate_hz)))'/double(cfg.simulation.navigation_rate_hz);
    assert(isequal(nav.time_s,navExpected));
    assert(max(abs((nav.velocity_sar_mps-navTruth.velocity_sar_mps)-nav.velocity_error_sar_mps),[],'all')<1e-12);
    assert(max(abs(vecnorm(nav.q_NB_wxyz,2,2)-1))<1e-12);
    report.(name)=struct('samples',numel(expected),'apc_max_error_m',max(abs(apcSar-s.apc_position_sar_m),[],'all'),...
        'orthogonality_max_error',maxOrthogonality,'navigation_pose_samples',numel(nav.time_s),'passed',true);
end
report.passed=true;
fprintf('MATLAB time/APC/beam/navigation validation passed.\n');
end
