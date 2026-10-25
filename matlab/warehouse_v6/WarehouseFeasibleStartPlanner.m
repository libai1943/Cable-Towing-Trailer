function R=WarehouseFeasibleStartPlanner(seed,P,opts)
% Restore a feasible primal trajectory before buffered direct-MAX refinement.
timer=tic;if ~isfield(opts,'timeLimit'),opts.timeLimit=30;end
if ~isfield(seed,'audit'),seed.audit=AuditWarehouseTrajectory(seed,P);end
a=opts;a.timeLimit=min(5,opts.timeLimit/3);a.maxIter=300;a.epscomp=1e-4;a.label=[opts.label '_relaxed'];
first=SolveWarehouseAMPL(seed,P,'complementarity',a);
[warm,warmReport]=WarehouseFeasibleStartRecovery(seed,{first},P,min(4,max(0,opts.timeLimit-toc(timer)-4)));
guess=seed;stages={first};if ~isempty(warm),guess=warm;stages{end+1}=warm;end
fine=WarehouseValidate(guess,P,8);margins=zeros(size(guess.X,2),2);
for t=1:size(margins,1),for b=1:2,margins(t,b)=max(vecnorm(WarehouseBody(guess.X(:,t),P,b)-WarehouseBody(fine.nodes(:,t),P,b),2,2));end;end
[margins,capReport]=WarehouseCapReplayReserves(margins,guess,P);
a.timeLimit=max(.1,opts.timeLimit-toc(timer)-4);a.maxIter=600;a.epscomp=0;a.label=[opts.label '_refine'];a.margin=margins;
second=SolveWarehouseMarginAMPL(guess,P,'max',a);stages{end+1}=second;
R=second;R.stages=stages;R=FinalizeSquareRecord(R,seed,P);R.method='recover_feasible';R.reserveCap=capReport;R.warmReport=warmReport;
R.prefix=struct('seconds',toc(timer),'rawVerified',R.rawVerified,'retainedVerified',R.retainedVerified,...
 'retainedCost',R.retainedCost,'retainedTravel',R.retainedTravel,'improved',R.improved,'stageCount',numel(R.stages));
[repair,report]=WarehousePrioritizedRecovery(R.selected,R.stages,P,max(0,opts.timeLimit-toc(timer)));
R.repairReport=report;if ~isempty(repair),R.stages{end+1}=repair;R=FinalizeSquareRecord(R,seed,P);end
R.totalSeconds=toc(timer);R.algorithmBudget=opts.timeLimit;
end
