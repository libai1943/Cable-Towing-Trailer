function R=WarehouseSquareStrategy(seed,P,method,opts)
timer=tic;if ~isfield(opts,'timeLimit'),opts.timeLimit=30;end
if ~isfield(seed,'audit'),seed.audit=AuditWarehouseTrajectory(seed,P);end
if ismember(method,{'recover','rgr','rrp'})
 reserve=0;if strcmp(method,'recover'),reserve=6;end
 a=opts;a.timeLimit=min(5,opts.timeLimit/3);a.maxIter=300;a.epscomp=1e-4;a.label=[opts.label '_relaxed'];
 first=SolveWarehouseAMPL(seed,P,'complementarity',a);
 if isfield(first,'S')&&all(isfinite(first.S)),guess=WarehouseRolloutSeed(first,P);else,guess=seed;end
 a.timeLimit=max(.1,opts.timeLimit-toc(timer)-reserve);a.maxIter=600;a.epscomp=0;a.label=[opts.label '_refine'];
 if strcmp(method,'rrp')
  second=SolveWarehouseAMPL(guess,P,'max',a);
 else
  fine=WarehouseValidate(guess,P,8);margins=zeros(size(guess.X,2),2);
  for t=1:size(margins,1),for b=1:2,margins(t,b)=max(vecnorm(WarehouseBody(guess.X(:,t),P,b)-WarehouseBody(fine.nodes(:,t),P,b),2,2));end;end
  a.margin=margins;second=SolveWarehouseMarginAMPL(guess,P,'max',a);
 end
 R=second;R.stages={first,second};
elseif strcmp(method,'max_recover')
 a=opts;a.timeLimit=max(.1,opts.timeLimit-6);R=SolveWarehouseAMPL(seed,P,'max',a);R.stages={};
elseif strcmp(method,'portfolio')
 R=WarehouseStrategy(seed,P,'portfolio',opts);
else
 a=opts;if strcmp(method,'minlp'),a.nlpTimeLimit=28;end
 R=SolveWarehouseAMPL(seed,P,method,a);R.stages={};
end
R=FinalizeSquareRecord(R,seed,P);R.method=method;
R.prefix=struct('seconds',toc(timer),'rawVerified',R.rawVerified,'retainedVerified',R.retainedVerified,...
 'retainedCost',R.retainedCost,'retainedTravel',R.retainedTravel,'improved',R.improved);
if ismember(method,{'recover','max_recover'})
 anchor=R.selected;
 [repair,report]=WarehouseRecoverControls(anchor,R.stages,P,max(0,opts.timeLimit-toc(timer)));
 R.repairReport=report;
 if ~isempty(repair),R.stages{end+1}=repair;R=FinalizeSquareRecord(R,seed,P);end
end
R.totalSeconds=toc(timer);R.algorithmBudget=opts.timeLimit;
end
