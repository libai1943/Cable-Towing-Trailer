function R=WarehouseStrategy(seed,P,method,opts)

% Control-variable comparison. Shared seed/model/OBCA/acceptance; strategies

% alter only mode representation or the specified refinement pipeline.

if nargin<4,opts=struct();end

wholeTimer=tic;

if ~isfield(opts,'timeLimit'),opts.timeLimit=20;end

if ~isfield(opts,'label'),opts.label='development';end

if strcmp(method,'portfolio')

 stages=cell(1,3);timer=tic;

 for j=1:3

  a=opts;a.timeLimit=max(.1,(opts.timeLimit-toc(timer))/(4-j));a.label=sprintf('%s_mode%d',opts.label,j);

  alternate=seed;

  if j>1

   switches=find(diff(seed.Z)~=0);

   for sw=switches,idx=max(1,min(numel(seed.Z),sw+(j==3)));alternate.Z(idx)=1-alternate.Z(idx);end

  end

  stages{j}=SolveWarehouseAMPL(alternate,P,'fixed',a);

 end

 R=stages{1};R.stages=stages;R.method=method;R.seconds=toc(timer);

elseif strcmp(method,'fixed_fast')

 a=opts;a.maxIter=900;R=SolveWarehouseAMPL(seed,P,'fixed',a);R.method=method;R.stages={};

elseif ~ismember(method,{'adaptive','adaptive_fast','no_rollout','no_polish','relaxation','relaxation_fast','relaxation_no_rollout','relaxation_no_polish'})

 R=SolveWarehouseAMPL(seed,P,method,opts);R.stages={};

else

 timer=tic;a=opts;

 fast=ismember(method,{'adaptive_fast','no_rollout','no_polish','relaxation_fast','relaxation_no_rollout','relaxation_no_polish'});

 if fast,a.maxIter=300;end

 if startsWith(method,'relaxation')

  a.timeLimit=min(5,opts.timeLimit/3);a.epscomp=1e-4;

  first=SolveWarehouseAMPL(seed,P,'complementarity',a);

 else

  a.timeLimit=min(5,opts.timeLimit/3);first=SolveWarehouseAMPL(seed,P,'max',a);

 end

 R=first;R.stages={first};

 if ~endsWith(method,'no_polish') && isfield(first,'X') && all(isfinite(first.X),'all')

  if endsWith(method,'no_rollout'),secondSeed=first;else,secondSeed=WarehouseRolloutSeed(first,P);end

  a.timeLimit=max(.1,opts.timeLimit-toc(timer));a.epscomp=0;

  if fast,a.maxIter=600;end

  a.label=[opts.label '_polish'];

  if startsWith(method,'relaxation'),secondMethod='max';else,secondMethod='fixed';end

  second=SolveWarehouseAMPL(secondSeed,P,secondMethod,a);stages={first,second};

  if second.rawVerified && (~first.rawVerified || second.cost<first.cost),R=second;end

  R.stages=stages;

 end

 R.method=method;R.seconds=toc(timer);

end

R.selectedSource='search_seed';R.selected=seed;

V=WarehouseValidate(seed,P,1);R.retainedVerified=V.accepted;

if R.rawVerified && (~V.accepted || R.cost<seed.cost-1e-7),R.selected=R;R.selected=rmfieldIf(R.selected,{'selected','stages'});R.selectedSource='optimizer';R.retainedVerified=true;end

R.retainedCost=R.selected.cost;R.retainedTravel=R.selected.travel;

R.improved=strcmp(R.selectedSource,'optimizer') && R.retainedCost<seed.cost-1e-5;

R.totalSeconds=toc(wholeTimer);

end

function R=rmfieldIf(R,fields)

for j=1:numel(fields),if isfield(R,fields{j}),R=rmfield(R,fields{j});end;end

end

