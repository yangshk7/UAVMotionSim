function R = quatToRot(q)
%QUATTOROT 标量在前 q_NB，主动旋转：v_N=R_NB*v_B；输入 4 元素。
q=q(:)/norm(q); w=q(1); x=q(2); y=q(3); z=q(4);
R=[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y); ...
   2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x); ...
   2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];
end
