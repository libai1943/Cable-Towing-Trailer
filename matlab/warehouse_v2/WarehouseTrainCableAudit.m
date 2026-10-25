function A=WarehouseTrainCableAudit(X,P)
% Fixed-material-length, per-node geometric routing witnesses for each link.
hits=0;wall=inf;len=0;sides=zeros(P.count,size(X,2));
for k=1:size(X,2)
 x=X(:,k);V=cell(1,P.count+1);anchors=zeros(2,P.count+1);anchors(:,1)=x(1:2);
 for b=1:P.count+1
  if b==1,pos=x(1:2);th=x(3);else,ids=3*(b-1)+(1:3);th=x(ids(3));pos=x(ids(1:2))-P.a*[cos(th);sin(th)];anchors(:,b)=pos-P.b(b)*[cos(th);sin(th)];end
  rot=[cos(th),-sin(th);sin(th),cos(th)];V{b}=P.vertices{b}*rot'+pos';
 end
 polys=V;
 for j=1:size(P.obstacles,1),o=P.obstacles(j,:);polys{end+1}=[o(1),o(3);o(2),o(3);o(2),o(4);o(1),o(4)];end %#ok<AGROW>
 for i=1:P.count
  ids=3*i+(1:3);h=x(ids(1:2));best=inf;
  for side=[-1,1]
   [C,e,sag]=WarehouseCable(anchors(:,i),h,P.L,side);local=0;
   cw=min([C(:,1)-P.left;P.right-C(:,1);C(:,2)-P.ymin;P.ymax-C(:,2)]);
   for b=1:numel(polys)
    [G,g]=WarehouseHalfspaces(polys{b});if b>P.count+1,g=g+sag/32^2;else,g=g-1e-8;end
    for j=1:size(C,1)-1
     aa=G*C(j,:)'-g;v=G*(C(j+1,:)-C(j,:))';t0=0;t1=1;
     for z=1:numel(g)
      if abs(v(z))<1e-14,if aa(z)>0,t0=2;break;end
      elseif v(z)>0,t1=min(t1,-aa(z)/v(z));else,t0=max(t0,-aa(z)/v(z));end
     end
     local=local+(t1>t0+1e-10);
    end
   end
   score=local+100*(cw<P.clearance);
   if score<best,best=score;be=e;bw=cw;bh=local;bs=side;end
  end
  hits=hits+bh;wall=min(wall,bw);len=max(len,be);sides(i,k)=bs;
 end
end
A=struct('accepted',hits==0&&wall>=P.clearance-1e-7&&len<1e-7,'hits',hits,'wall',wall,'lengthError',len,'sides',sides,...
 'sideChanges',sum(diff(sides,1,2)~=0,2),'scope','Per-node cable versus bodies, obstacles and walls; no cable shape dynamics or cable-cable contact model');
end
