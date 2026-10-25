function seed=WarehouseRolloutSeed(R,P)

% Refresh continuous initialization with the same exact discrete max law.

seed=R;N=numel(R.S);seed.X=zeros(6,N+1);seed.X(:,1)=P.initial;

for i=1:N,[seed.X(:,i+1),seed.W(i),seed.E(i)]=WarehouseRopeStep(seed.X(:,i),seed.S(i),seed.K(i),P);end

seed.Z=double(seed.E>=0);seed.cost=sum(abs(seed.S))+P.regularization*sum(diff(seed.K).^2);

seed.travel=sum(abs(seed.S));

end

