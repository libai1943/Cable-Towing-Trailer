function R=FinalizeSquareRecord(R,seed,P)
% Common acceptance for nonlinear outputs and explicitly labeled repairs.
if isfield(R,'stages')&&~isempty(R.stages),candidates=R.stages;else,candidates={R};end
if isfield(seed,'audit'),seedAudit=seed.audit;else,seedAudit=AuditWarehouseTrajectory(seed,P);end
best=[];audits=cell(size(candidates));solverAccepted=false;
for j=1:numel(candidates)
 c=candidates{j};if ~isfield(c,'X'),continue;end
 if isfield(c,'audit'),A=c.audit;else,A=AuditWarehouseTrajectory(c,P);end
 c.audit=A;c.cost=A.cost;c.travel=A.travel;audits{j}=A;candidates{j}=c;
 available=true;if isfield(c,'solverCandidateAvailable'),available=c.solverCandidateAvailable;end
 integerOkay=~isfield(c,'integerError')||c.integerError<1e-6;
 valid=A.accepted&&available&&integerOkay;
 if valid&&~isfield(c,'recovery'),solverAccepted=true;end
 if valid&&(isempty(best)||c.cost<best.cost),best=c;end
end
R.stages=candidates;R.stageAudits=audits;R.rawVerified=~isempty(best);R.rawSolverAccepted=solverAccepted;
R.seedAudit=seedAudit;R.selectedSource='search_seed';R.selected=seed;R.retainedVerified=seedAudit.accepted;
if ~isempty(best)
 R.optimizerCandidate=best;
 if ~seedAudit.accepted||best.cost<seed.cost-1e-7
  for name={'selected','stages','stageAudits','optimizerCandidate'},if isfield(best,name{1}),best=rmfield(best,name{1});end;end
  R.selected=best;R.selectedSource='planner';R.retainedVerified=true;
 end
end
R.retainedCost=R.selected.cost;R.retainedTravel=R.selected.travel;
R.improved=R.retainedVerified&&R.retainedCost<seed.cost-1e-5;
R.recoverySelected=isfield(R.selected,'recovery');
end
