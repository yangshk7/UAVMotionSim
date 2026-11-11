function test_matlab_closed_loop()
%TEST_MATLAB_CLOSED_LOOP 姿态扰动下水平速度PI仍应纠正正负速度误差。
% 仅触发速度环一步，不混入姿态/分配器动态；期望值为解析PI结果。
oldPath=path; restore=onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(fullfile(fileparts(mfilename('fullpath')),'legacy_uav'));
c.rates=struct('dynamics_Hz',100,'position_Hz',10,'velocity_Hz',100,'attitude_Hz',10,'rate_Hz',10);
c.control.velocityKp=ones(3,1)*1.5;
c.control.velocityKi=ones(3,1)*0.12;
c.control.velocityKd=zeros(3,1);
c.control.velocityIntegralLimit=ones(3,1)*2;
c.limits.horizontalAcceleration_mps2=2.5;
c.limits.verticalAcceleration_mps2=2;
trim=struct('velocity_N_mps',[10;0;0],'q',[1;0;0;0],...
    'thrust_N',117.72,'motor_radps',ones(6,1)*500);
r.attitudeBias_rad=repmat([0.5 0.5 0]*pi/180,2,1);
r.acceleration_N_mps2=zeros(2,3);
for axis=1:2
    for error=[-1 1]
        s=uavlegacy.controllers.initialize(trim);
        x=[zeros(3,1);trim.velocity_N_mps;trim.q;zeros(9,1)];
        x(3+axis)=x(3+axis)-error;
        s=uavlegacy.controllers.update(s,x,r,2,c,struct());
        assert(abs(s.accelerationCommand(axis)-(1.5+0.12/100)*error)<1e-12,...
            'Horizontal feedback was frozen under attitude disturbance');
        assert(abs(s.velocityIntegral(axis)-error/100)<1e-12);
    end
end
disp('MATLAB disturbed horizontal closed-loop regression passed (4 analytic cases).');
end
