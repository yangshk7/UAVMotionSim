function dx = derivative(x,m,motorCommand,external)
%DERIVATIVE 19 状态 FRD/NED 刚体 ODE。external=[Fx;Fy;Fz;Mx;My;Mz]。
% p/v 使用 NED，q_NB 标量在前，omega 使用机体系，电机单位 rad/s。
% 外力作用于质心；sar模式在每次RK4子阶段按当前姿态转为机体系，力矩仍为FRD。
R=uavlegacy.frames.quatToRot(x(7:10)); w=x(11:13); motor=x(14:19);
thrust=m.kf*motor.^2; wrench=m.A*thrust;
bodyVelocity=R'*x(4:6);
drag=-m.drag.*abs(bodyVelocity).*bodyVelocity;
forceBody=external(1:3);
if m.externalForceInSAR
    forceBody=R'*(external(1:3).*[1;1;-1]);
end
accel=[0;0;m.gravity]+R*([0;0;-wrench(1)]+drag+forceBody)/m.mass;
wDot=m.invJ*(wrench(2:4)+external(4:6)-cross(w,m.J*w));
qDot=0.5*uavlegacy.frames.multiply(x(7:10),[0;w]);
dx=[x(4:6);accel;qDot;wDot;(motorCommand-motor)/m.motorTau];
end
