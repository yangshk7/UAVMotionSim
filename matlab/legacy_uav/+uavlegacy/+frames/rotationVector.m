function v = rotationVector(q)
%ROTATIONVECTOR 单位四元数的最短弧旋转向量，rad。
q=q(:)/norm(q); if q(1)<0, q=-q; end
s=norm(q(2:4));
if s<1e-12, v=2*q(2:4); else, v=2*atan2(s,q(1))*q(2:4)/s; end
end
