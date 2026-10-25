function [route,A]=WarehouseSelectCableRoute(x,P)
% Symmetric route first, then the same finite family of asymmetric cubics.
polys={WarehouseBody(x,P,1),WarehouseBody(x,P,2)};
for j=1:size(P.obstacles,1),o=P.obstacles(j,:);polys{end+1}=[o(1),o(3);o(2),o(3);o(2),o(4);o(1),o(4)];end %#ok<AGROW>
best=inf;
for beta=[0,.25,.5,.75,-.25,-.5,-.75]
 for side=[-1,1]
  [C,e,bound]=WarehouseAsymmetricCable(x(1:2),x(4:5),P.L,beta,side);hits=0;
  wall=min([C(:,1)-P.left;P.right-C(:,1);C(:,2)-P.ymin;P.ymax-C(:,2)])-bound;
  for j=1:numel(polys)
   [G,g]=WarehouseHalfspaces(polys{j});if j>2,g=g+bound;else,g=g-1e-8;end
   for k=1:size(C,1)-1
    aa=G*C(k,:)'-g;v=G*(C(k+1,:)-C(k,:))';t0=0;t1=1;
    for z=1:numel(g)
     if abs(v(z))<1e-14,if aa(z)>0,t0=2;break;end
     elseif v(z)>0,t1=min(t1,-aa(z)/v(z));else,t0=max(t0,-aa(z)/v(z));end
    end
    hits=hits+(t1>t0+1e-10);
   end
  end
  okay=hits==0&&wall>=P.clearance-1e-7&&e<1e-7;score=hits+100*(wall<P.clearance);
  if score<best
   route=C;A=struct('accepted',okay,'hits',hits,'wall',wall,'lengthError',e,'beta',beta,'side',side);best=score;
  end
  if okay,return;end
 end
end
end
