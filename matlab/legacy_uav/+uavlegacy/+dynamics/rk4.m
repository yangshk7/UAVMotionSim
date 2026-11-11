function next = rk4(x,h,m,command,f0,f1,fm)
%RK4 固定步长四阶积分；外力/力矩在离散输入网格间线性插值，电机指令 ZOH。
if nargin<7, fm=(f0+f1)/2; end
k1=uavlegacy.dynamics.derivative(x,m,command,f0);
k2=uavlegacy.dynamics.derivative(x+h*k1/2,m,command,fm);
k3=uavlegacy.dynamics.derivative(x+h*k2/2,m,command,fm);
k4=uavlegacy.dynamics.derivative(x+h*k3,m,command,f1);
next=x+h*(k1+2*k2+2*k3+k4)/6;
next(7:10)=next(7:10)/norm(next(7:10));
end
