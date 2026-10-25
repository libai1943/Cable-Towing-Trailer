function cases=EulerCases(group)
% Case design frozen before evaluation; development and evaluation distinct.
if nargin<1,group='algorithm';end
cases=struct('name',{},'split',{},'P',{});
if strcmp(group,'mechanism')
 for j=1:18
  P=RopeParameters();P.obstacles=zeros(0,4);P.rho=1;
  lengths=[1.3,1.5,1.7];P.L=lengths(mod(j-1,3)+1);
  P.B=P.a+P.L;C=StripCertificate(P);
  fractions=[.3,.6,.85];P.extra=fractions(mod(floor((j-1)/3),3)+1)*C.minimumExtraWidthForLeavingBox;
  if j>9,P.radius=.5;P.deltaMax=atan(P.Lf/P.radius);end
  P=finish(P);cases(j)=struct('name',sprintf('G%02d',j),'split','mechanism','P',P);
 end
 return
end
for j=1:24
 P=RopeParameters();P.obstacles=zeros(0,4);P.rho=1;
 if j<=6,split='development';else,split='evaluation';end
 switch mod(j-1,6)+1
  case 1,P.extra=.04;
  case 2,P.extra=.20;
  case 3,P.extra=.50;P.L=1.1;
  case 4,P.extra=.15;P.L=1.5;
  case 5,P.extra=.3;P.radius=.55;P.deltaMax=atan(P.Lf/P.radius);
  case 6,P.extra=.40;
 end
 if j>6
  band=floor((j-1)/6);extras=[0,.025,.065,.10];P.extra=P.extra+extras(band+1);
  P.L=P.L+.035*band;
 end
 P=finish(P);
  P.goalY=6.10;P.goalTolerance=.004;P.planBuffer=.015;
 if mod(j-1,6)==5,P.initial(4)=P.initial(1)-.9*P.L;end
 if j>=13 && j<=18,P.obstacles=[P.left,.85,3.1,4.0];end
 if j>=19,P.obstacles=[2.15,P.right,3.4,4.3];end
 cases(j)=struct('name',sprintf('A%02d',j),'split',split,'P',P);
end
end
function P=finish(P)
P.B=P.a+P.L;P.left=-P.rear(2)-P.clearance-P.extra/2;
P.right=P.B+P.front(1)+P.clearance+P.extra/2;
P.initial=[P.B;0;0;P.a;0;0];P.planBuffer=.008;
P.goalY=6.05;P.goalAngle=pi/2;P.goalTolerance=.008;
P.taskGoalY=6.0;P.taskGoalTolerance=.01;
P.smax=.18;P.mesh=100;P.regularization=.002;
end
