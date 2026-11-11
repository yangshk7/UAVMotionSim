function raw = simulatePhysical(c,r,external,trim,m)
%SIMULATEPHYSICAL 固定 RK4 积分；输入网格 Nx6 外力(N)/力矩(N m)。
% 输出 Nx19 状态、导数及独立控制器历史；不在求导中产生随机数。
n=numel(r.time_s); h=1/c.rates.dynamics_Hz;
assert(isequal(size(external),[n 6]),'uav:ExternalSize','外力/力矩必须为 Nx6。');
states=zeros(n,19); accel=zeros(n,3); motorCmd=zeros(n,6);
angularAccel=zeros(n,3);
residual=zeros(n,4); integral=zeros(n,6); desiredQ=zeros(n,4);
wrench=zeros(n,4); saturated=false(n,1);
x=[r.position_N_m(1,:)';trim.velocity_N_mps;trim.q;zeros(3,1);trim.motor_radps];
if isfield(c,'initialState'), x=c.initialState(:); end
s=uavlegacy.controllers.initialize(trim);
for k=1:n
    s=uavlegacy.controllers.update(s,x,r,k,c,m);
    dx=uavlegacy.dynamics.derivative(x,m,s.motorCommand,external(k,:)');
    states(k,:)=x'; accel(k,:)=dx(4:6)'; angularAccel(k,:)=dx(11:13)'; motorCmd(k,:)=s.motorCommand';
    residual(k,:)=s.allocationResidual'; integral(k,:)=[s.velocityIntegral;s.rateIntegral]';
    desiredQ(k,:)=s.desiredQuaternion'; wrench(k,:)=s.wrench'; saturated(k)=s.saturated;
    if k<n, x=uavlegacy.dynamics.rk4(x,h,m,s.motorCommand,external(k,:)',external(k+1,:)',c.externalHalf(k,:)'); end
    if mod(k,max(1,floor(n/100)))==0
        fprintf('UAVSIM_MATLAB_PROGRESS %.9f\n',(k-1)/(n-1));
    end
    if mod(k,c.rates.dynamics_Hz*5)==0
        assert(all(isfinite(x)),'uav:Nonfinite','动力学状态出现 NaN/Inf。');
        if c.output.showProgress, fprintf('  physics t = %.2f s\n',r.time_s(k)); end
    end
end
raw.angularAcceleration_B_radps2=angularAccel;
raw.time_s=r.time_s; raw.state=states; raw.acceleration_N_mps2=accel;
assert(all(isfinite(states),'all') && all(isfinite(accel),'all'),'uav:Nonfinite','完整状态或加速度出现 NaN/Inf。');
raw.quality.quaternionNormMaxError=max(abs(vecnorm(states(:,7:10),2,2)-1));
raw.quality.motorSpeedMin_radps=min(states(:,14:19),[],'all');
raw.quality.motorSpeedMax_radps=max(states(:,14:19),[],'all');
raw.quality.allocationSaturationFraction=mean(saturated);
assert(raw.quality.quaternionNormMaxError<1e-10 && raw.quality.motorSpeedMin_radps>=c.vehicle.minRotorSpeed_radps-1e-8 ...
    && raw.quality.motorSpeedMax_radps<=m.maxSpeed+1e-8,'uav:PhysicalBounds','四元数或电机转速超出物理边界。');
raw.controller=struct('motorCommand_radps',motorCmd,'wrenchCommand',wrench, ...
    'allocationResidual',residual,'integralState',integral,'desiredQuaternion_NB',desiredQ,'saturated',saturated);
% 预稳定只针对 t<0 的物理状态；扰动从采集开始才启用。
pre=r.time_s<0 & r.time_s>=-min(1,c.timing.preStable_s);
if any(pre)
    speedError=vecnorm(states(pre,4:6)-r.velocity_N_mps(pre,:),2,2);
    heightError=abs(states(pre,3)-r.position_N_m(pre,3));
    raw.steady=struct('speedError_mps',max(speedError),'heightError_m',max(heightError), ...
        'angularRate_radps',max(vecnorm(states(pre,11:13),2,2)), ...
        'acceleration_mps2',max(vecnorm(accel(pre,:),2,2)));
    raw.steady.passed=raw.steady.speedError_mps<c.validation.steadySpeed_mps && ...
        raw.steady.heightError_m<c.validation.steadyHeight_m && ...
        raw.steady.angularRate_radps<c.validation.steadyRate_radps && ...
        raw.steady.acceleration_mps2<c.validation.steadyAcceleration_mps2;
    assert(raw.steady.passed,'uav:PreStable','预稳定未收敛；检查配平、增益与初始扰动。');
else
    raw.steady=struct('checked',false,'reason','No pre-stabilization interval requested');
end
end
