function A=WarehouseCableAudit(X,P)
hits=0;wall=inf;lengthError=0;sides=zeros(1,size(X,2));
for i=1:size(X,2)
 x=X(:,i);best=inf;chosen=0;
 for side=[-1,1]
  [C,e,sag]=WarehouseCable(x(1:2),x(4:5),P.L,side);local=0;
  w=min([C(:,1)-P.left;P.right-C(:,1);C(:,2)-P.ymin;P.ymax-C(:,2)]);
  polys={WarehouseBody(x,P,1),WarehouseBody(x,P,2)};
  for j=1:size(P.obstacles,1),b=P.obstacles(j,:);polys{end+1}=[b(1),b(3);b(2),b(3);b(2),b(4);b(1),b(4)];end %#ok<AGROW>
  for j=1:numel(polys)
   [G,g]=WarehouseHalfspaces(polys{j});
   % Curve sample segments: conservative obstacle expansion by chord error.
   if j>2,g=g+sag/32^2;else,g=g-1e-8;end
   for k=1:size(C,1)-1
    aa=G*C(k,:)'-g;v=G*(C(k+1,:)-C(k,:))';t0=0;t1=1;
    for z=1:numel(g)
     if abs(v(z))<1e-14,if aa(z)>0,t0=2;break;end
     elseif v(z)>0,t1=min(t1,-aa(z)/v(z));else,t0=max(t0,-aa(z)/v(z));end
    end
    local=local+(t1>t0+1e-10);
   end
  end
  score=local+100*(w<P.clearance);
  if score<best,best=score;chosen=side;be=e;bw=w;bh=local;end
 end
 sides(i)=chosen;hits=hits+bh;wall=min(wall,bw);lengthError=max(lengthError,be);
end
% A per-node routing witness is not a continuous cable-shape dynamics model.
A=struct('accepted',hits==0&&wall>=P.clearance-1e-7&&lengthError<1e-7,...
 'hits',hits,'wall',wall,'lengthError',lengthError,'sides',sides,'sideChanges',sum(diff(sides)~=0));
end
