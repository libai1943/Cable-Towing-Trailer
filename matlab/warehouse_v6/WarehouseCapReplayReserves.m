function [margins,report]=WarehouseCapReplayReserves(margins,anchor,P)
% Keep a certified feasible anchor available inside the buffered subproblem.
% Half of each obstacle/wall clearance surplus remains unused. Each body
% receives at most one quarter of the shared body-to-body clearance surplus.
plan=P.clearance+P.planBuffer;caps=zeros(size(margins));minimum=inf;
for k=1:size(anchor.X,2)
 A={WarehouseBody(anchor.X(:,k),P,1),WarehouseBody(anchor.X(:,k),P,2)};
 self=PolygonDistance(A{1},A{2});minimum=min(minimum,self);
 for b=1:2
  V=A{b};gap=min([V(:,1)-P.left;P.right-V(:,1);V(:,2)-P.ymin;P.ymax-V(:,2)]);
  for j=1:size(P.obstacles,1)
   o=P.obstacles(j,:);O=[o(1),o(3);o(2),o(3);o(2),o(4);o(1),o(4)];gap=min(gap,PolygonDistance(V,O));
  end
  minimum=min(minimum,gap);caps(k,b)=max(0,min((gap-plan)/2,(self-plan)/4));
 end
end
assert(minimum>=plan-1e-7,'Anchor does not satisfy the common planning clearance');
before=margins;margins=min(margins,caps);
report=struct('anchorMinimumClearance',minimum,'planningClearance',plan,'fraction',.5,...
 'numberCapped',sum(before>caps+1e-10,'all'),'totalEntries',numel(before),...
 'maximumOriginal',max(before,[],'all'),'maximumCapped',max(margins,[],'all'));
end
