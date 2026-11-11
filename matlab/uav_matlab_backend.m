function uav_matlab_backend(inputFile, outputFile)
%UAV_MATLAB_BACKEND Execute the vendored legacy MATLAB physical core.
% Shared inputs are SI, NED/FRD and scalar-first quaternions on an exact grid.
oldPath=path; restore=onCleanup(@() path(oldPath));
addpath(fullfile(fileparts(mfilename('fullpath')),'legacy_uav'));
source=load(inputFile);
v=source.vehicle; p=source.controller; rates=source.rates;
c.rates=struct('dynamics_Hz',rates(1),'position_Hz',rates(2), ...
    'velocity_Hz',rates(3),'attitude_Hz',rates(4),'rate_Hz',rates(5));
c.timing.preStable_s=source.pre;
c.initialState=source.initial_state;
c.externalHalf=source.external_half;
c.output.showProgress=false;
c.control.positionKp=p.kp_position_per_s(:);
c.control.velocityKp=p.kp_velocity_per_s(:);
c.control.velocityKi=p.ki_velocity_per_s2(:);
c.control.velocityKd=zeros(3,1);
c.control.velocityIntegralLimit=p.velocity_integral_limit_m(:);
c.control.attitudeKp=p.kp_attitude_per_s(:);
c.control.rateKp=diag(v.inertia_kg_m2).*p.kp_rate_per_s(:);
c.control.rateKi=diag(v.inertia_kg_m2).*p.ki_rate_per_s2(:);
c.control.rateKd=zeros(3,1);
c.control.rateIntegralLimit=p.rate_integral_limit_rad(:);
c.control.dragFeedforward=false;
c.limits.speed_mps=p.max_speed_mps;
c.limits.horizontalAcceleration_mps2=p.max_horizontal_acceleration_mps2;
c.limits.verticalAcceleration_mps2=p.max_vertical_acceleration_mps2;
c.limits.tilt_rad=p.max_tilt_deg*pi/180;
c.limits.angularRate_radps=repmat(p.max_body_rate_radps,3,1);
c.limits.torque_Nm=p.max_torque_Nm(:);
c.validation=struct('steadySpeed_mps',0.1,'steadyHeight_m',0.05, ...
    'steadyRate_radps',0.02,'steadyAcceleration_mps2',0.05);
m.A=source.allocation_matrix;
m.W=diag(p.allocation_weights(:)); m.Aw=m.W*m.A; m.pinvAw=pinv(m.Aw);
m.lower=0; m.upper=v.max_rotor_thrust_N; m.maxSpeed=sqrt(m.upper/v.kf);
c.vehicle.minRotorSpeed_radps=sqrt(m.lower/v.kf);
m.J=v.inertia_kg_m2; m.invJ=inv(m.J); m.mass=v.mass_kg;
m.gravity=v.gravity_mps2; m.kf=v.kf; m.motorTau=v.motor_time_constant_s;
m.drag=0.5*v.air_density_kg_m3*v.drag_cd_area_m2(:)*double(v.drag_enabled);
m.externalForceInSAR=false;
trim.velocity_N_mps=source.reference.velocity_N_mps(1,:)';
trim.q=source.initial_state(7:10); trim.thrust_N=m.mass*m.gravity;
trim.motor_radps=repmat(sqrt(trim.thrust_N/(6*m.kf)),6,1);
raw=uavlegacy.dynamics.simulatePhysical(c,source.reference,source.external,trim,m);
states=raw.state; time_s=raw.time_s;
acceleration=raw.acceleration_N_mps2;
angular_acceleration=raw.angularAcceleration_B_radps2;
motor_commands=raw.controller.motorCommand_radps;
allocation_residuals=raw.controller.allocationResidual;
matlab_version=version;
save(outputFile,'states','time_s','acceleration','angular_acceleration', ...
    'motor_commands','allocation_residuals','matlab_version','-v7');
fprintf('UAVSIM_MATLAB_PROGRESS 1.0\n');
end
