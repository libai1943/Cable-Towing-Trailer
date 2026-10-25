function [best,report]=WarehouseRecoverControls(seed,candidates,P,seconds)

% Backtrack in signed travel and yaw increments toward a validated seed.

% The common physical model and validator remain unchanged.

clock=tic;best=[];rows={};alphaGrid=2.^-(0:10);

for c=1:numel(candidates)

 cand=candidates{c};if ~isfield(cand,'S')||any(~isfinite(cand.S))||any(~isfinite(cand.K)),continue;end

 S=cand.S;limit=abs(S)/P.radius;Y=S.*cand.K;target=P.goalAngle-P.initial(3);

 % Project yaw increments onto the common curvature bounds and heading sum.

 if abs(target)>sum(limit)+1e-10,continue;end

 lo=-2*sum(limit)-abs(target)-1;hi=-lo;

 weights=max(abs(S),1e-8);

 for it=1:65

  shift=(lo+hi)/2;YP=max(-limit,min(limit,Y+shift*weights));

  if sum(YP)>target,hi=shift;else,lo=shift;end

 end

 Y=max(-limit,min(limit,Y+((lo+hi)/2)*weights));

 for alpha=alphaGrid

  if toc(clock)>seconds,break;end

  s=(1-alpha)*seed.S+alpha*S;y=(1-alpha)*(seed.S.*seed.K)+alpha*Y;

  cap=abs(s)/P.radius;if abs(target)>sum(cap)||any(abs(s)>P.smax+1e-9),continue;end

  left=-2*sum(cap)-abs(target)-1;right=-left;weight=max(abs(s),1e-8);

  for iteration=1:65

   mid=(left+right)/2;z=max(-cap,min(cap,y+mid*weight));

   if sum(z)>target,right=mid;else,left=mid;end

  end

  y=max(-cap,min(cap,y+(left+right)/2*weight));

  k=zeros(size(s));nz=abs(s)>1e-10;k(nz)=y(nz)./s(nz);

  if any(abs(y(~nz))>1e-10),continue;end

  trial=WarehouseRolloutSeed(struct('S',s,'K',k),P);

  [trial,terminalRestore]=WarehouseTerminalRestore(trial,seed,P);

  if trial.cost>=seed.cost-1e-5,continue;end

  A=AuditWarehouseTrajectory(trial,P);

  row=struct('candidate',c,'alpha',alpha,'accepted',A.accepted,'cost',A.cost,...

   'fineClearance',A.fineClearance,'cableHits',A.cableHits,'fineGoalError',A.fineGoalError,'fineHeadingError',A.fineHeadingError);

  rows{end+1}=row; %#ok<AGROW>

  if A.accepted

   trial.status='recovered_controls';trial.solverCandidateAvailable=true;trial.audit=A;

   trial.recovery=struct('candidate',c,'alpha',alpha,'terminalRestore',terminalRestore);trial.rawVerified=true;

   if isempty(best)||trial.cost<best.cost,best=trial;end

   break

  end

 end

end

report=struct('seconds',toc(clock),'trials',[rows{:}],'found',~isempty(best));

end

