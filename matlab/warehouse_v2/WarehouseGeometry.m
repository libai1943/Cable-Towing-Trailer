function [ok,wall,self,obstacle]=WarehouseGeometry(x,P)
A=WarehouseBody(x,P,1);B=WarehouseBody(x,P,2);V=[A;B];
wall=min([V(:,1)-P.left;P.right-V(:,1);V(:,2)-P.ymin;P.ymax-V(:,2)]);
self=PolygonDistance(A,B);obstacle=inf;
for j=1:size(P.obstacles,1)
 b=P.obstacles(j,:);C=[b(1),b(3);b(2),b(3);b(2),b(4);b(1),b(4)];
 obstacle=min([obstacle,PolygonDistance(A,C),PolygonDistance(B,C)]);
end
d=norm(x(1:2)-x(4:5));ok=min([wall,self,obstacle])>=P.clearance-1e-7&&d<=P.L+1e-7&&d>=.25-1e-7;
end
