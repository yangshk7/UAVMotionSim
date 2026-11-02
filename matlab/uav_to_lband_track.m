function track = uav_to_lband_track(passResult,cfg,source)
%UAV_TO_LBAND_TRACK 只做适配，不修改原雷达参数和处理器。
% source: truth / navigation / geometry；位置为 SAR 左手 X前/Y右/Z上，m。
% 旧回波程序只使用 actualXYZ_m；姿态波束另存，旧 beam_weight 不自动消费。
arguments
    passResult struct
    cfg struct
    source (1,:) char = 'truth'
end
s=passResult.truth_sar;
t=s.time_s(:);
assert(numel(t)==cfg.acquisition.numPulses,'Pulse count differs from SAR configuration.');
assert(max(abs(diff(t)-1/cfg.radar.prf_Hz))<1e-12,'PRF differs from SAR configuration.');
assert(cfg.acquisition.pulseStride==1,'Adapter requires every radar pulse.');
track.time_s=t-(t(1)+t(end))/2;
track.originalPulseIndex=(1:numel(t))';
track.nominalXYZ_m=passResult.reference.apc_position_sar_m;
switch source
    case 'truth'
        track.actualXYZ_m=s.apc_position_sar_m;
        track.beamUnitXYZ=s.beam_sar_unit;
    case 'navigation'
        track.actualXYZ_m=passResult.navigation_sar.apc_position_sar_m;
        track.beamUnitXYZ=s.beam_sar_unit;
    case 'geometry'
        track.actualXYZ_m=passResult.sar_geometry.apc_position_sar_m;
        track.beamUnitXYZ=passResult.sar_geometry.beam_sar_unit;
    otherwise
        error('Unknown source.');
end
track.errorXYZ_m=track.actualXYZ_m-track.nominalXYZ_m;
track.referenceXYZ_m=interp1(t,track.nominalXYZ_m,(t(1)+t(end))/2,'linear');
track.passIndex=passResult.metadata.pass_index;
track.effectivePRF_Hz=cfg.radar.prf_Hz;
track.motionSource=source;
end
