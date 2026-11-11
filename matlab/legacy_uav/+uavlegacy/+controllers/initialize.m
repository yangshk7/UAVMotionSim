function s = initialize(trim)
%INITIALIZE 控制器状态与 19 维物理状态分开存储。
s.velocityCommand=trim.velocity_N_mps;
s.accelerationCommand=zeros(3,1);
s.velocityIntegral=zeros(3,1); s.previousVelocityError=zeros(3,1);
s.rateIntegral=zeros(3,1); s.previousRate=zeros(3,1);
s.desiredQuaternion=trim.q; s.rateCommand=zeros(3,1);
s.totalThrust=trim.thrust_N; s.motorCommand=trim.motor_radps;
s.wrench=[trim.thrust_N;0;0;0]; s.allocationResidual=zeros(4,1);
s.saturated=false;
end
