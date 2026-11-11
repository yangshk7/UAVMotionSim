function compare_legacy_motion(outputFolder, legacyRoot)
%COMPARE_LEGACY_MOTION Run the legacy physical solver against shared inputs.
% Inputs contain SI parameters, fixed NED/FRD conventions and full-rate forces.
% Comparison configuration is saved locally; legacy source files are read only.
addpath(fullfile(legacyRoot,'functions','setup'));
startup_uav();
source=load(fullfile(outputFolder,'shared_inputs.mat'));
p=source.parameters;
c=default_uav_config();
c.sar.radar.speed_mps=p.speed;
c.sar.radar.height_m=p.height;
c.sar.radar.prf_Hz=p.sarRate;
c.sar.acquisition.numPulses=round(p.duration*p.sarRate)+1;
c.sar.duration_s=p.duration;
c.rates.dynamics_Hz=p.rate;
c.rates.position_Hz=p.controlRates(1);
c.rates.velocity_Hz=p.controlRates(2);
c.rates.attitude_Hz=p.controlRates(3);
c.rates.rate_Hz=p.controlRates(4);
c.timing.preStable_s=p.pre;
c.vehicle.mass_kg=p.mass;
c.vehicle.gravity_mps2=p.gravity;
c.vehicle.inertia_kgm2=p.inertia;
c.vehicle.arm_m=p.arm;
c.vehicle.kf_Ns2=p.kf;
c.vehicle.km_Nms2=p.km;
c.vehicle.motorTau_s=p.motorTau;
c.vehicle.maxRotorThrust_N=p.maximum;
c.antenna.lever_B_m=p.lever(:);
c.drag.enabled=false;
c.control.dragFeedforward=false;
c.control.positionKp=p.positionKp(:);
c.control.velocityKp=p.velocityKp(:);
c.control.velocityKi=p.velocityKi(:);
c.control.velocityKd=zeros(3,1);
c.control.velocityIntegralLimit=p.velocityIntegral(:);
c.control.attitudeKp=p.attitudeKp(:);
% Python uses J*(kp*rate_error+ki*integral), legacy gains multiply torque directly.
c.control.rateKp=diag(p.inertia).*p.rateKp(:);
c.control.rateKi=diag(p.inertia).*p.rateKi(:);
c.control.rateKd=zeros(3,1);
c.control.rateIntegralLimit=p.rateIntegral(:);
c.control.allocationWeights=p.weights(:);
c.limits.speed_mps=p.speedLimit;
c.limits.horizontalAcceleration_mps2=p.horizontalAcceleration;
c.limits.verticalAcceleration_mps2=p.verticalAcceleration;
c.limits.angularRate_radps=repmat(p.rateLimit,3,1);
c.limits.tilt_rad=p.tilt*pi/180;
c.navigation.enabled=false;
c.output.showProgress=false;
c.output.makePlots=false;
for k=1:numel(c.disturbances), c.disturbances(k).enabled=false; end
validate_uav_config(c);
m=uav.vehicle.model(c);
trim=uav.reference.trim(c,m);
reference=uav.reference.nominal(c,c.passes(1),trim);
assert(max(abs(reference.time_s-source.time_s))<1e-9,'Shared time grid mismatch');
names={'baseline','force_y'};
for index=1:2
    external=zeros(numel(reference.time_s),6);
    if index==2, external(:,2)=source.force_y; end
    raw=uav.dynamics.simulatePhysical(c,reference,external,trim,m);
    states=raw.state;
    time_s=raw.time_s;
    configuration=c;
    save(fullfile(outputFolder,['matlab_' names{index} '.mat']), ...
        'time_s','states','configuration','external','-v7');
end
end
