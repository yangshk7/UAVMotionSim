function q = multiply(a,b)
%MULTIPLY Hamilton 四元数乘积；旋转矩阵满足 R(a*b)=R(a)R(b)。
a=a(:); b=b(:);
q=[a(1)*b(1)-dot(a(2:4),b(2:4)); ...
    a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end
