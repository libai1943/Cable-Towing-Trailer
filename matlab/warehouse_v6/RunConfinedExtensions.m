function RunConfinedExtensions(group)
if nargin<1,group='all';end
if ismember(group,{'all','portfolio_unique'})
 for i=1:9
  d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',i));P=d.P;w=d.w;
  label=sprintf('confined_unique_W%02d_portfolio_unique',i);dest=destination('unique',label);if isfile(dest),continue;end
  schedules={w.Z};switches=find(diff(w.Z)~=0);
  for j=2:3
   z=w.Z;for sw=switches,idx=max(1,min(numel(z),sw+(j==3)));z(idx)=1-z(idx);end
   duplicate=any(cellfun(@(s)isequal(s,z),schedules));if ~duplicate,schedules{end+1}=z;end
  end
  timer=tic;stages={};
  for j=1:numel(schedules)
   seed=w;seed.Z=schedules{j};opts=struct('timeLimit',max(.1,(30-toc(timer))/(numel(schedules)-j+1)),'label',sprintf('%s_branch%d',label,j));
   stages{j}=SolveWarehouseAMPL(seed,P,'fixed',opts);
  end
  R=stages{1};R.stages=stages;R=FinalizeSquareRecord(R,w,P);R.method='portfolio_unique';R.uniqueSchedules=numel(schedules);R.totalSeconds=toc(timer);saveRecord(R,P,dest);
 end
end
if ismember(group,{'all','ablations'})
 RunConfinedStudy(1:9,{'rrp','max_recover'},'ablations');
end
if ismember(group,{'all','rigid'})
 for i=1:9
  d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',i));P=d.P;P.rigid=true;
  w=WarehouseRolloutSeed(d.w,P);w.audit=AuditWarehouseTrajectory(w,P);
  runOne(w,P,'rigid','rigid',sprintf('confined_rigid_W%02d_rigid',i));
 end
end
if ismember(group,{'all','smoothing'})
 for i=[2,5,9]
  d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',i));P=d.P;w=d.w;
  for beta=[100,1000]
   label=sprintf('confined_smoothing_W%02d_lse%d',i,beta);dest=destination('smoothing',label);if isfile(dest),continue;end
   timer=tic;R=SolveWarehouseAMPL(w,P,'lse',struct('label',label,'timeLimit',30,'beta',beta));R.stages={};R=FinalizeSquareRecord(R,w,P);R.method=sprintf('lse%d',beta);R.totalSeconds=toc(timer);saveRecord(R,P,dest);
  end
 end
end
if ismember(group,{'all','repeats'})
 cfg=jsondecode(fileread('research/warehouse_v6/final_method.json'));
 for repeat=1:3,RunConfinedAdditional([2,5,9],{cfg.method,'max'},sprintf('repeat%d',repeat));end
end
end
function runOne(w,P,method,group,label)
dest=destination(group,label);if isfile(dest),return;end
R=WarehouseSquareStrategy(w,P,method,struct('label',label,'timeLimit',30));saveRecord(R,P,dest);
end
function dest=destination(group,label)
folder=fullfile('results/warehouse_v6',group);if ~isfolder(folder),mkdir(folder);end;dest=fullfile(folder,[label '.mat']);
end
function saveRecord(R,P,dest)
save(dest,'R','P');record=R;record.P=P;
for name={'stages','selected','optimizerCandidate'},if isfield(record,name{1}),record=rmfield(record,name{1});end;end
fid=fopen(strrep(dest,'.mat','.json'),'w');fwrite(fid,jsonencode(record,PrettyPrint=true));fclose(fid);
fprintf('%s new=%d retained=%d improve=%d cost=%.6f seconds=%.2f\n',dest,R.rawVerified,R.retainedVerified,R.improved,R.retainedCost,R.totalSeconds);
end
