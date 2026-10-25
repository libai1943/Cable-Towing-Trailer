function V=WarehouseBody(x,P,b)
% Arbitrary convex footprint in the body frame; vertices counterclockwise.
if b==1,p=x(1:2);th=x(3);else,p=x(4:5)-P.a*[cos(x(6));sin(x(6))];th=x(6);end
R=[cos(th),-sin(th);sin(th),cos(th)];V=P.vertices{b}*R'+p';
end
