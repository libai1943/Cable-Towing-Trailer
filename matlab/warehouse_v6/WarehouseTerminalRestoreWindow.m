function [trial,info]=WarehouseTerminalRestoreWindow(trial,anchor,P)

% Three-dimensional shooting correction on the common terminal straight.

% Signed travel and zero-sum yaw bases preserve the tractor terminal heading.

n=numel(trial.S);window=min(n,max(9,ceil(n/4)));idx=n-window+1:n;

info=struct('used',false,'iterations',0,'residual',inf,'window',window);

if numel(idx)<9,return;end

edges=round(linspace(0,numel(idx),4));B=zeros(3,n);B(1,idx)=1/numel(idx);

for j=1:2

 p=idx(edges(j)+1:edges(j+1));q=idx(edges(j+1)+1:edges(j+2));B(j+1,p)=1/numel(p);B(j+1,q)=-1/numel(q);

end

baseS=trial.S;baseY=trial.S.*trial.K;delta=zeros(3,1);

target=[anchor.X(4:5,end)-P.a*[cos(anchor.X(6,end));sin(anchor.X(6,end))];anchor.X(6,end)];

for iteration=1:5

 [current,ok]=roll(delta);if ~ok,return;end

 r=endpoint(current)-target;info.residual=max(abs(r));info.iterations=iteration;

 if info.residual<2e-6,trial=current;info.used=true;return;end

 J=zeros(3);h=1e-4;

 for j=1:3,d=delta;d(j)=d(j)+h;[v,~]=roll(d);J(:,j)=(endpoint(v)-endpoint(current))/h;end

 step=-pinv(J,1e-8)*r;step=step/max([1,abs(step(1))/1.0,abs(step(2))/.3,abs(step(3))/.3]);

 accepted=false;

 for rate=[1,.5,.25,.125]

  d=delta+rate*step;[v,ok]=roll(d);

  if ok&&norm(endpoint(v)-target)<norm(r),delta=d;trial=v;accepted=true;break;end

 end

 if ~accepted,return;end

end

info.used=true;info.residual=max(abs(endpoint(trial)-target));

 function [v,ok]=roll(d)

  s=baseS+d(1)*B(1,:);y=baseY+d(2)*B(2,:)+d(3)*B(3,:);k=zeros(size(s));nz=abs(s)>1e-10;k(nz)=y(nz)./s(nz);

  ok=all(abs(s)<=P.smax+1e-8)&all(abs(k)<=1/P.radius+1e-8)&all(abs(y(~nz))<1e-10);

  v=WarehouseRolloutSeed(struct('S',s,'K',k),P);

 end

 function v=endpoint(c)

  v=[c.X(4:5,end)-P.a*[cos(c.X(6,end));sin(c.X(6,end))];c.X(6,end)];

 end

end

