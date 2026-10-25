function R=WarehouseMaxRecoveryPlanner(seed,P,opts)
timer=tic;a=opts;a.timeLimit=max(.1,opts.timeLimit-6);
R=SolveWarehouseAMPL(seed,P,'max',a);R.stages={};R=FinalizeSquareRecord(R,seed,P);R.method='max_recover_priority';
R.prefix=struct('seconds',toc(timer),'rawVerified',R.rawVerified,'retainedVerified',R.retainedVerified,...
 'retainedCost',R.retainedCost,'retainedTravel',R.retainedTravel,'improved',R.improved);
[repair,report]=WarehousePrioritizedRecovery(R.selected,R.stages,P,max(0,opts.timeLimit-toc(timer)));
R.repairReport=report;if ~isempty(repair),R.stages{end+1}=repair;R=FinalizeSquareRecord(R,seed,P);end
R.totalSeconds=toc(timer);R.algorithmBudget=opts.timeLimit;
end
