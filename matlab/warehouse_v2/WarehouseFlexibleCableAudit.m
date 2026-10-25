function A=WarehouseFlexibleCableAudit(X,P)
hits=0;wall=inf;len=0;okay=true;betas=zeros(1,size(X,2));sides=betas;
for k=1:size(X,2)
 [~,a]=WarehouseSelectCableRoute(X(:,k),P);hits=hits+a.hits;wall=min(wall,a.wall);len=max(len,a.lengthError);okay=okay&&a.accepted;betas(k)=a.beta;sides(k)=a.side;
end
A=struct('accepted',okay,'hits',hits,'wall',wall,'lengthError',len,'betas',betas,'sides',sides,'sideChanges',sum(diff(sides)~=0),'asymmetricNodes',nnz(betas));
end
