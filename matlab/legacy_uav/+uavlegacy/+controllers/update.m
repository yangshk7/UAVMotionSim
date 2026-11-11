function s = update(s,x,r,k,c,m)
%UPDATE 位置P→速度PI(D)→SO(3)姿态P→角速度PID，多速率整数步调度。
% r 为完整参考数组；控制指令在更新间隔内零阶保持。
step=k-1; f=c.rates.dynamics_Hz;
p=x(1:3); v=x(4:6); q=x(7:10); w=x(11:13);
R=uavlegacy.frames.quatToRot(q);
bias=r.attitudeBias_rad(k,:)';
if mod(step,round(f/c.rates.position_Hz))==0
    cmd=r.velocity_N_mps(k,:)'+c.control.positionKp.*(r.position_N_m(k,:)'-p);
    s.velocityCommand=limitNorm(cmd,c.limits.speed_mps);
end
if mod(step,round(f/c.rates.velocity_Hz))==0
    dt=1/c.rates.velocity_Hz; e=s.velocityCommand-v;
    candidate=s.velocityIntegral+dt*e;
    candidate=min(c.control.velocityIntegralLimit,max(-c.control.velocityIntegralLimit,candidate));
    ac=r.acceleration_N_mps2(k,:)'+c.control.velocityKp.*e+c.control.velocityKi.*candidate ...
        +c.control.velocityKd.*(e-s.previousVelocityError)/dt;
    % 姿态目标扰动不关闭位置/速度闭环；飞控继续纠正其引起的航迹偏离。
    limited=ac; limited(1:2)=limitNorm(ac(1:2),c.limits.horizontalAcceleration_mps2);
    limited(3)=min(c.limits.verticalAcceleration_mps2,max(-c.limits.verticalAcceleration_mps2,ac(3)));
    % 条件积分：当误差进一步推向饱和边界时停止对应积分。
    accept=(abs(ac-limited)<1e-12 | e.*(ac-limited)<=0) & ~s.saturated;
    s.velocityIntegral(accept)=candidate(accept);
    s.accelerationCommand=limited; s.previousVelocityError=e;
end
if mod(step,round(f/c.rates.attitude_Hz))==0
    force=m.mass*(s.accelerationCommand-[0;0;m.gravity]);
    if c.control.dragFeedforward
        vb=R'*r.velocity_N_mps(k,:)'; drag=-m.drag.*abs(vb).*vb;
        force=force-R*drag;
    end
    b3=-force/norm(force);
    tilt=atan2(norm(b3(1:2)),b3(3));
    if tilt>c.limits.tilt_rad
        b3=[sin(c.limits.tilt_rad)*b3(1:2)/norm(b3(1:2));cos(c.limits.tilt_rad)];
    end
    heading=[cos(r.yaw_rad(k)+bias(3));sin(r.yaw_rad(k)+bias(3));0];
    b2=cross(b3,heading); b2=b2/norm(b2); b1=cross(b2,b3);
    Rd=[b1 b2 b3]*uavlegacy.frames.quatToRot(uavlegacy.frames.eulerToQuat([bias(1:2);0]));
    % 注入后再次限制总倾角；保留航向及倾斜的水平方向。
    tilted=atan2(norm(Rd(1:2,3)),Rd(3,3));
    if tilted>c.limits.tilt_rad
        z=[sin(c.limits.tilt_rad)*Rd(1:2,3)/norm(Rd(1:2,3));cos(c.limits.tilt_rad)];
        y=cross(z,heading); y=y/norm(y); Rd=[cross(y,z) y z];
    end
    s.desiredQuaternion=uavlegacy.frames.rotToQuat(Rd);
    qe=uavlegacy.frames.multiply([q(1);-q(2:4)],s.desiredQuaternion);
    s.rateCommand=c.control.attitudeKp.*uavlegacy.frames.rotationVector(qe);
    s.rateCommand=min(c.limits.angularRate_radps,max(-c.limits.angularRate_radps,s.rateCommand));
    s.totalThrust=max(0,dot(-force,R(:,3)));
end
if mod(step,round(f/c.rates.rate_Hz))==0
    dt=1/c.rates.rate_Hz; e=s.rateCommand-w;
    candidate=s.rateIntegral+dt*e;
    candidate=min(c.control.rateIntegralLimit,max(-c.control.rateIntegralLimit,candidate));
    torque=c.control.rateKp.*e+c.control.rateKi.*candidate-c.control.rateKd.*(w-s.previousRate)/dt+cross(w,m.J*w);
    unlimitedTorque=torque;
    torque=min(c.limits.torque_Nm,max(-c.limits.torque_Nm,torque));
    s.wrench=[s.totalThrust;torque]; [rotor,info]=uavlegacy.vehicle.allocate(s.wrench,m);
    accept=e.*info.residual(2:4)>=0 | abs(info.residual(2:4))<1e-8;
    accept=accept & abs(torque-unlimitedTorque)<1e-12;
    s.rateIntegral(accept)=candidate(accept);
    s.motorCommand=sqrt(rotor/m.kf); s.allocationResidual=info.residual;
    s.saturated=info.saturated; s.previousRate=w;
end
end
function y=limitNorm(x,maximum)
y=x*min(1,maximum/max(norm(x),eps));
end
