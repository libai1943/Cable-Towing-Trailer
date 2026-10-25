function d=WarehouseDual(G,g,n)
% Exact support-function dual from pairs of active normal generators.
d=zeros(size(g));best=inf;
for i=1:numel(g)
 for j=i+1:numel(g)
  A=G([i,j],:)';if abs(det(A))<1e-12,continue;end
  v=A\n;if min(v)<-1e-9,continue;end
  v=max(v,0);cost=g([i,j])'*v;
  if cost<best,best=cost;d(:)=0;d([i,j])=v;end
 end
end
assert(isfinite(best)&&norm(G'*d-n)<1e-7,'Could not initialize OBCA dual.');
end
