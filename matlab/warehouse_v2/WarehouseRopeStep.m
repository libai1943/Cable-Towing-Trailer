function [y,w,eta]=WarehouseRopeStep(x,s,k,P)

if isfield(P,'rigid')&&P.rigid

 y=x;y(1:2)=x(1:2)+s*[cos(x(3));sin(x(3))];y(3)=x(3)+s*k;

 y(6)=x(6)+s/(P.a+P.L)*sin(x(3)-x(6));

 y(4:5)=y(1:2)-P.L*[cos(y(6));sin(y(6))];w=0;eta=0;return;

end

% First-order explicit Warehouse on [p_x,p_y,phi,h_x,h_y,theta].

% Algebraic unilateral length closure; no RK or continuous arc integrator.

% The smaller root is unique for |s|<L and D>=dmin>|s|.

r=x(1:2)-x(4:5);d=norm(r);n=r/d;ef=[cos(x(3));sin(x(3))];

c=n'*ef;eta=d+s*c-sqrt(P.L^2-s^2*(1-c^2));w=max(0,eta);

y=x;y(1:2)=x(1:2)+s*ef;y(3)=x(3)+s*k;

y(4:5)=x(4:5)+w*n;

y(6)=x(6)+w/P.a*(n(2)*cos(x(6))-n(1)*sin(x(6)));

end

