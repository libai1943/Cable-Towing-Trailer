function M=WarehouseDistanceReserves(guess,fine,P)
% Constraint-specific observed clearance loss on the common reference replay.
% These internal reserves are frozen during refinement; final validation is
% unchanged and is required after any iterate moves from the reference.
n=size(guess.X,2);o=size(P.obstacles,1);M.obstacle=zeros(n,2,o);M.wall=zeros(n,2,4);M.self=zeros(n,1);
for k=1:n
 A={};F={};
 for b=1:2
  A{b}=WarehouseBody(guess.X(:,k),P,b);F{b}=WarehouseBody(fine.nodes(:,k),P,b);
  for j=1:o
   z=P.obstacles(j,:);V=[z(1),z(3);z(2),z(3);z(2),z(4);z(1),z(4)];
   M.obstacle(k,b,j)=max(0,PolygonDistance(A{b},V)-PolygonDistance(F{b},V));
  end
  ca=[min(A{b}(:,1))-P.left,P.right-max(A{b}(:,1)),min(A{b}(:,2))-P.ymin,P.ymax-max(A{b}(:,2))];
  cf=[min(F{b}(:,1))-P.left,P.right-max(F{b}(:,1)),min(F{b}(:,2))-P.ymin,P.ymax-max(F{b}(:,2))];
  M.wall(k,b,:)=max(0,ca-cf);
 end
 M.self(k)=max(0,PolygonDistance(A{1},A{2})-PolygonDistance(F{1},F{2}));
end
end
