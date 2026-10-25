function V=WarehouseValidate(R,P,refine)
if nargin<3,refine=1;end
N=numel(R.S);x=P.initial;nodes=zeros(6,N+1);nodes(:,1)=x;
wall=inf;self=inf;obs=inf;rope=0;reverse=0;modeResidual=0;slackMotion=0;
for i=1:N
 for j=1:refine
  [y,w,e]=WarehouseRopeStep(x,R.S(i)/refine,R.K(i),P);
  n=(x(1:2)-x(4:5))/norm(x(1:2)-x(4:5));if ~isfield(P,'rigid')||~P.rigid,reverse=max(reverse,-w*n'*[cos(x(6));sin(x(6))]);end
  if e<0,slackMotion=max(slackMotion,norm(y(4:6)-x(4:6)));end
  [~,a,b,c]=WarehouseGeometry(y,P);wall=min(wall,a);self=min(self,b);obs=min(obs,c);
  rope=max(rope,norm(y(1:2)-y(4:5))-P.L);x=y;
 end
 nodes(:,i+1)=x;if refine==1&&isfield(R,'W'),modeResidual=max(modeResidual,abs(w-R.W(i)));end
end
q=x(4:5)-P.a*[cos(x(6));sin(x(6))];nodeError=max(abs(nodes-R.X),[],'all');
goalError=abs(q-P.goal);tol=P.goalXYTolerance;
if refine>1,tol=tol+P.replayTolerance;end
headingTol=P.goalTolerance;if refine>1,headingTol=P.taskGoalTolerance;end
V=struct('refine',refine,'wall',wall,'self',self,'obstacle',obs,'ropeExcess',rope,...
 'rearReverse',reverse,'slackMotion',slackMotion,'maxNodeError',nodeError,'modeResidual',modeResidual,...
 'goalError',goalError,'headingError',abs(x(6)-P.goalAngle),'frontHeadingError',abs(x(3)-P.goalAngle),'nodes',nodes);
V.discreteFeasible=min([wall,self,obs])>=P.clearance-1e-5&&rope<1e-6&&reverse<1e-7&&all(goalError<=tol+1e-5)&&V.headingError<=headingTol+1e-5&&V.frontHeadingError<1e-5;
V.accepted=V.discreteFeasible&&(refine~=1||nodeError<1e-5);
end
