function R=WarehouseInvokePlanner(seed,P,method,opts)
% Record a failed pipeline as a failed new output, retaining its physical seed.
% Underlying frozen algorithm branches and acceptance criteria are unchanged.
timer=tic;
try
 if startsWith(method,'fx_'),R=WarehouseFactorialPlanner(seed,P,opts,extractAfter(method,'fx_'));
 elseif startsWith(method,'fi_'),R=WarehouseFeasibleStartAblation(seed,P,opts,extractAfter(method,'fi_'));
 elseif strcmp(method,'recover_feasible'),R=WarehouseFeasibleStartPlanner(seed,P,opts);
 elseif strcmp(method,'recover_capped'),R=WarehouseAnchorCappedPlanner(seed,P,opts);
 elseif strcmp(method,'recover_distance'),R=WarehouseDistancePlanner(seed,P,opts);
 elseif strcmp(method,'recover_priority'),R=WarehousePrioritizedPlanner(seed,P,opts);
 elseif strcmp(method,'minlp')
  opts.nlpTimeLimit=max(.1,opts.timeLimit-2);R=SolveWarehouseAMPL(seed,P,method,opts);R.stages={};R=FinalizeSquareRecord(R,seed,P);R.totalSeconds=toc(timer);
 else,R=WarehouseSquareStrategy(seed,P,method,opts);
 end
catch problem
 R=struct('method',method,'stages',{{}},'status','pipeline_exception','exceptionIdentifier',problem.identifier,'exceptionMessage',problem.message);
 R=FinalizeSquareRecord(R,seed,P);R.totalSeconds=toc(timer);R.algorithmBudget=opts.timeLimit;
 R.prefix=struct('seconds',R.totalSeconds,'rawVerified',false,'retainedVerified',R.retainedVerified,...
  'retainedCost',R.retainedCost,'retainedTravel',R.retainedTravel,'improved',false,'stageCount',0);
 fprintf('Recorded pipeline failure (%s): %s\n',method,problem.message);
end
end
