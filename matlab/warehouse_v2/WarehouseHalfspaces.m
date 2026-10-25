function [G,g]=WarehouseHalfspaces(V)
% Counterclockwise vertices -> outward unit normals G*x <= g.
area=sum(V(:,1).*circshift(V(:,2),-1)-V(:,2).*circshift(V(:,1),-1));
assert(area>0,'Footprint vertices must be counterclockwise.');
E=circshift(V,-1)-V;G=[E(:,2),-E(:,1)];G=G./vecnorm(G,2,2);g=sum(G.*V,2);
assert(max(G*V'-g,[],'all')<1e-9,'Footprint must be convex.');
end
