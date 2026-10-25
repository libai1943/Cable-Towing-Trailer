function p=EulerPose(x,P)
q=x(4:5)-P.a*[cos(x(6));sin(x(6))];
p=[x(1:3);atan2(x(2)-x(5),x(1)-x(4));x(6);q];
end
