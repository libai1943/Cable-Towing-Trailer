function VerifyConfinedResults()
% Fresh primal-geometry and recursive-Euler audits, independent of solver flags.
groups={'main','final','confirm','unique','ablations','matched','factorial','paired','rigid','smoothing','repeat1','repeat2','repeat3','shapes','budgets'};
rows={};count=0;
for g=1:numel(groups)
 files=dir(fullfile('results/warehouse_v6',groups{g},'*.mat'));
 for j=1:numel(files)
  d=load(fullfile(files(j).folder,files(j).name));if ~isfield(d,'R'),continue;end
  A=AuditWarehouseTrajectory(d.R.selected,d.P);
  assert(A.accepted==d.R.retainedVerified,'Acceptance mismatch: %s',files(j).name);
  assert(abs(A.cost-d.R.retainedCost)<1e-6,'Cost mismatch: %s',files(j).name);
  if ~isfield(d.P,'rigid')||~d.P.rigid,assert(A.accepted,'Invalid cable output: %s',files(j).name);end
  raw=false;native=false;
  for k=1:numel(d.R.stages)
   c=d.R.stages{k};if ~isfield(c,'X'),continue;end
   B=AuditWarehouseTrajectory(c,d.P);
   available=~isfield(c,'solverCandidateAvailable')||c.solverCandidateAvailable;
   integerOkay=~isfield(c,'integerError')||c.integerError<1e-6;
   valid=B.accepted&&available&&integerOkay;raw=raw||valid;
   native=native||(valid&&~isfield(c,'recovery'));
  end
  assert(raw==d.R.rawVerified&&native==d.R.rawSolverAccepted,'New-output audit mismatch: %s',files(j).name);
  count=count+1;rows{end+1}=struct('file',files(j).name,'group',groups{g},'retainedAccepted',A.accepted,...
   'rawAccepted',raw,'nativeAccepted',native,'cost',A.cost,'fineClearance',A.fineClearance); %#ok<AGROW>
  if mod(count,20)==0,fprintf('Freshly verified %d records.\n',count);end
 end
end
z=load('results/warehouse_v6/train/three_trailers.mat');assert(z.R.accepted);
q=struct('passed',true,'recordCount',count,'records',[rows{:}],'trainSavedAuditAccepted',z.R.accepted);
fid=fopen('results/warehouse_v6/verification.json','w');fwrite(fid,jsonencode(q,PrettyPrint=true));fclose(fid);
fprintf('VERIFIED %d confined-warehouse result records.\n',count);
end
