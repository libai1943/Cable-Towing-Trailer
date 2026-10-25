function V=BodyPolygon(p,theta,g)
R=[cos(theta),-sin(theta);sin(theta),cos(theta)];
V=[g(1),g(3);-g(2),g(3);-g(2),-g(3);g(1),-g(3)]*R'+p(:)';
end
