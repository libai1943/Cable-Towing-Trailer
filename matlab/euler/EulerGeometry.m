function [ok,wall,self,obstacle]=EulerGeometry(x,P)
p=EulerPose(x,P);A=BodyPolygon(p(1:2)',p(3),P.front);B=BodyPolygon(p(6:7)',p(5),P.rear);
wall=min([A(:,1)-P.left;P.right-A(:,1);B(:,1)-P.left;P.right-B(:,1)]);
self=PolygonDistance(A,B);obstacle=inf;
for j=1:size(P.obstacles,1)
 b=P.obstacles(j,:);C=[b(1),b(3);b(2),b(3);b(2),b(4);b(1),b(4)];
 obstacle=min([obstacle,PolygonDistance(A,C),PolygonDistance(B,C)]);
end
r=x(1:2)-x(4:5);d=norm(r);
ok=min([wall,self,obstacle])>=P.clearance-1e-7 && d<=P.L+1e-7 && d>=.25-1e-7 ...
 && x(2)>=P.ymin && x(2)<=P.ymax && x(5)>=P.ymin && x(5)<=P.ymax;
end
