function [thrust,info] = allocate(wrench,m)
%ALLOCATE 有界最小二乘控制分配，无 Optimization Toolbox 依赖。
% 输入 [T;tau_x;tau_y;tau_z]；输出 6x1 推力 N；残差保留原物理单位。
% BVLS 活跃集：可行线搜索 + KKT 释放；求解失败明确报错。
b=m.W*wrench(:); x=m.pinvAw*b;
lo=m.lower; hi=m.upper;
if all(x>=lo & x<=hi)
    thrust=x; iterations=0;
else
    x=min(hi,max(lo,x)); active=zeros(6,1);
    active(x<=lo+1e-10)=-1; active(x>=hi-1e-10)=1;
    converged=false;
    for iterations=1:100
        free=active==0; bound=~free; z=x;
        if any(free), z(free)=pinv(m.Aw(:,free))*(b-m.Aw(:,bound)*x(bound)); end
        step=1; direction=z-x;
        for j=find(free)'
            if z(j)<lo, step=min(step,(lo-x(j))/direction(j)); end
            if z(j)>hi, step=min(step,(hi-x(j))/direction(j)); end
        end
        x=x+step*direction;
        if step<1-1e-12
            hitLo=free & x<=lo+1e-9 & direction<0;
            hitHi=free & x>=hi-1e-9 & direction>0;
            active(hitLo)=-1; active(hitHi)=1;
            x=min(hi,max(lo,x)); continue;
        end
        g=m.Aw'*(m.Aw*x-b);
        violation=zeros(6,1);
        violation(active==-1)=-g(active==-1);
        violation(active==1)=g(active==1);
        [worst,j]=max(violation);
        if worst<1e-8, converged=true; break; end
        active(j)=0;
    end
    assert(converged,'uav:AllocationFailed','有界控制分配未满足 KKT 条件。');
    thrust=min(hi,max(lo,x));
end
info.residual = m.A*thrust-wrench(:);
info.saturated=any(thrust<=lo+1e-8 | thrust>=hi-1e-8);
info.iterations=iterations;
end
