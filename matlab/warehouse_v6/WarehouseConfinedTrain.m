function WarehouseConfinedTrain()

% Constructive extension with three differently shaped passive carts.

% This validates serial transmission; it is not a multi-trailer optimizer benchmark.

P=RopeParameters;P.L=1.335;P.a=.7;P.radius=.4;P.clearance=.1;

P.vertices={BodyPolygon([0,0],0,P.front),BodyPolygon([0,0],0,P.rear),...

 [.7,.40;-.5,.40;-.5,-.40;.7,-.40], [.7,0;.43,.55;-.35,.42;-.35,-.42;.43,-.55]};

P.b=[0,.35,.5,.35];P.count=3;P.left=0;P.right=16;P.ymin=0;P.ymax=16;

% Fixed warehouse loading lane, pallet island and modular storage racks.

P.obstacles=[];P.obstacleKinds={};

for x=[.5,4.2,7.9,11.6]

 P.obstacles(end+1,:)=[x,x+2.0,8.6,14.6];P.obstacleKinds{end+1}='rack';

 P.obstacles(end+1,:)=[x,x+2.0,.4,2.2];P.obstacleKinds{end+1}='station';

end

P.obstacles(end+1,:)=[3.0,5.2,4.8,6.0];P.obstacleKinds{end+1}='pallet';

P.obstacles(end+1,:)=[13.0,15.0,4.0,7.4];P.obstacleKinds{end+1}='rack';

storage=struct('layout','Fixed modular racks and loading stations; no swept-path filling');

u=[-P.radius*pi/2,-1/P.radius;3.2,0;2.0*pi/2,1/2;7.4,0]';S=[];K=[];

for j=1:size(u,2),n=ceil(abs(u(1,j))/.08);S=[S,repmat(u(1,j)/n,1,n)];K=[K,repmat(u(2,j),1,n)];end %#ok<AGROW>

x=zeros(3+3*P.count,1);anchor=[0;0];

for i=1:P.count,ids=3*i+(1:3);x(ids(1:2))=anchor-[P.L;0];q=x(ids(1:2))-[P.a;0];anchor=q-[P.b(i+1);0];end

for body=0:P.count,ix=3*body+(1:2);x(ix)=x(ix)+[11;3];end;P.initial=x;[X,W,A]=replay(S,K,P,1);[XF,WF,AF]=replay(S,K,P,8); %#ok<ASGLU>

[X,W,A]=replay(S,K,P,1);[XF,WF,AF]=replay(S,K,P,8);

AC=WarehouseTrainCableAudit(X,P);ACF=WarehouseTrainCableAudit(XF,P);

out='results/warehouse_v6/train';if ~isfolder(out),mkdir(out);end

R=struct('X',X,'W',W,'S',S,'K',K,'audit',A,'fineAudit',AF,'cableAudit',AC,'fineCableAudit',ACF,'accepted',A.accepted&&AF.accepted&&AC.accepted&&ACF.accepted,...

 'evidence','Constructive serial-kinematic feasibility demonstration, not an optimized multi-trailer benchmark');

fprintf('SERIAL audit body=%.4f fine=%.4f cable=%d/%d\n',A.clearance,AF.clearance,AC.hits,ACF.hits);save(fullfile(out,'three_trailers.mat'),'R','P','storage');

fid=fopen(fullfile(out,'three_trailers.json'),'w');fwrite(fid,jsonencode(struct('P',P,'R',R,'storage',storage),PrettyPrint=true));fclose(fid);

fprintf('TRAIN accepted=%d coarseClearance=%.4f fineClearance=%.4f reverse=%.3g length=%.3g\n',R.accepted,A.clearance,AF.clearance,A.reverse,A.lengthExcess);

end

function [X,W,A]=replay(S,K,P,refine)

x=P.initial;X=zeros(numel(x),numel(S)+1);X(:,1)=x;W=zeros(P.count,numel(S));gap=inf;rope=0;reverse=0;slack=0;

for k=1:numel(S)

 for sub=1:refine

  y=x;y(1:2)=x(1:2)+S(k)/refine*[cos(x(3));sin(x(3))];y(3)=x(3)+S(k)/refine*K(k);

  oldAnchor=x(1:2);newAnchor=y(1:2);

  for i=1:P.count

   ids=3*i+(1:3);h=x(ids(1:2));theta=x(ids(3));r=oldAnchor-h;d=norm(r);n=r/d;dp=newAnchor-oldAnchor;c=n'*dp;

   rad=P.L^2-dot(dp,dp)+c*c;assert(rad>0&&d>norm(dp));eta=d+c-sqrt(rad);w=max(0,eta);

   y(ids(1:2))=h+w*n;y(ids(3))=theta+w/P.a*(n'*[-sin(theta);cos(theta)]);W(i,k)=W(i,k)+w;

   rope=max(rope,norm(newAnchor-y(ids(1:2)))-P.L);reverse=max(reverse,-w*n'*[cos(theta);sin(theta)]);

   if eta<0,slack=max(slack,norm(y(ids)-x(ids)));end

   oldAnchor=h-(P.a+P.b(i+1))*[cos(theta);sin(theta)];newAnchor=y(ids(1:2))-(P.a+P.b(i+1))*[cos(y(ids(3)));sin(y(ids(3)))];

  end

  x=y;V=cell(1,P.count+1);

  for b=1:P.count+1

   if b==1,pos=x(1:2);th=x(3);else,ids=3*(b-1)+(1:3);th=x(ids(3));pos=x(ids(1:2))-P.a*[cos(th);sin(th)];end

   rot=[cos(th),-sin(th);sin(th),cos(th)];V{b}=P.vertices{b}*rot'+pos';

   gap=min([gap;V{b}(:,1)-P.left;P.right-V{b}(:,1);V{b}(:,2)-P.ymin;P.ymax-V{b}(:,2)]);

   for j=1:b-1,gap=min(gap,PolygonDistance(V{b},V{j}));end

   for j=1:size(P.obstacles,1),o=P.obstacles(j,:);poly=[o(1),o(3);o(2),o(3);o(2),o(4);o(1),o(4)];gap=min(gap,PolygonDistance(V{b},poly));end

  end

 end

 X(:,k+1)=x;

end

A=struct('clearance',gap,'lengthExcess',rope,'reverse',reverse,'slackMotion',slack,...

 'accepted',gap>=P.clearance-1e-5&&rope<1e-6&&reverse<1e-7&&slack<1e-10,'refine',refine);

end

