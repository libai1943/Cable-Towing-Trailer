function R=WarehouseDistancePlanner(seed,P,opts)

% Same model and tolerance. Prioritize exact-refinement controls for recovery.

timer=tic;if ~isfield(opts,'timeLimit'),opts.timeLimit=30;end

if ~isfield(seed,'audit'),seed.audit=AuditWarehouseTrajectory(seed,P);end

a=opts;a.timeLimit=min(5,opts.timeLimit/3);a.maxIter=300;a.epscomp=1e-4;a.label=[opts.label '_relaxed'];

first=SolveWarehouseAMPL(seed,P,'complementarity',a);

if isfield(first,'S')&&all(isfinite(first.S)),guess=WarehouseRolloutSeed(first,P);else,guess=seed;end

fine=WarehouseValidate(guess,P,8);M=WarehouseDistanceReserves(guess,fine,P);

a.timeLimit=max(.1,opts.timeLimit-toc(timer)-6);a.maxIter=600;a.epscomp=0;a.label=[opts.label '_refine'];a.distanceMargin=M;

second=SolveWarehouseDistanceAMPL(guess,P,'max',a);

R=second;R.stages={first,second};R=FinalizeSquareRecord(R,seed,P);R.method='recover_distance';

R.prefix=struct('seconds',toc(timer),'rawVerified',R.rawVerified,'retainedVerified',R.retainedVerified,...

 'retainedCost',R.retainedCost,'retainedTravel',R.retainedTravel,'improved',R.improved);

[repair,report]=WarehousePrioritizedRecovery(R.selected,R.stages,P,max(0,opts.timeLimit-toc(timer)));

R.repairReport=report;

if ~isempty(repair),R.stages{end+1}=repair;R=FinalizeSquareRecord(R,seed,P);end

R.totalSeconds=toc(timer);R.algorithmBudget=opts.timeLimit;

end

