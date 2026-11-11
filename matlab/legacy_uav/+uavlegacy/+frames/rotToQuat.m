function q = rotToQuat(R)
%ROTTOQUAT SO(3) 主动旋转到标量在前四元数；稳定处理接近 pi 的旋转。
t=trace(R);
if t>0
    s=2*sqrt(1+t); q=[s/4;(R(3,2)-R(2,3))/s;(R(1,3)-R(3,1))/s;(R(2,1)-R(1,2))/s];
else
    [~,i]=max(diag(R)); j=mod(i,3)+1; k=mod(j,3)+1;
    s=2*sqrt(max(0,1+R(i,i)-R(j,j)-R(k,k)));
    q=zeros(4,1); q(i+1)=s/4; q(1)=(R(k,j)-R(j,k))/s;
    q(j+1)=(R(j,i)+R(i,j))/s; q(k+1)=(R(k,i)+R(i,k))/s;
end
q=q/norm(q); if q(1)<0, q=-q; end
end
