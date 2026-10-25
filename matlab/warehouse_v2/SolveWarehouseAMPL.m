function R=SolveWarehouseAMPL(seed,P,method,opts)

if nargin<4,opts=struct();end

timerAll=tic;

defaults=struct('epscomp',0,'beta',100,'timeLimit',20,'maxIter',3000,'label','pilot','solutionLimit',1000000,'nlpTimeLimit',4,'disablePump',false);

for f=fieldnames(defaults)',if ~isfield(opts,f{1}),opts.(f{1})=defaults.(f{1});end;end

root=fileparts(fileparts(fileparts(mfilename('fullpath'))));folder=fullfile(root,'results/warehouse_v2/runtime');

if ~isfolder(folder),mkdir(folder);end

WriteWarehouseAMPL(seed,P,method,folder,opts);

for file={'status.txt','states.txt','controls.txt','result.txt','integers.txt','process.json'},p=fullfile(folder,file{1});if isfile(p),delete(p);end;end

old=pwd;guard=onCleanup(@()cd(old));cd(folder);t=tic;

[code,~]=system(sprintf('powershell.exe -NoProfile -ExecutionPolicy Bypass -File ../../../tools/run_ampl_budget.ps1 -Budget %.6g',opts.timeLimit));seconds=toc(t);clear guard

R=struct('method',method,'options',opts,'seconds',seconds,'exitCode',code,'P',P,'rawVerified',false,'status','missing_output','cost',inf);

out=fullfile(root,'results/warehouse_v2/logs');if ~isfolder(out),mkdir(out);end

copyfile(fullfile(folder,'solver.log'),fullfile(out,[opts.label '_' method '.log']));

inputDir=fullfile(root,'results/warehouse_v2/run_inputs',[opts.label '_' method]);if ~isfolder(inputDir),mkdir(inputDir);end

for fn={'problem.dat','initial.run','ipopt.opt','run.run','mode.mod','process.json','states.txt','controls.txt','status.txt','integers.txt'}

 if isfile(fullfile(folder,fn{1})),copyfile(fullfile(folder,fn{1}),fullfile(inputDir,fn{1}));end

end

if strcmp(method,'minlp'),copyfile(fullfile(folder,'bonmin.opt'),fullfile(inputDir,'bonmin.opt'));end

if isfile(fullfile(folder,'process.json')),R.process=jsondecode(fileread(fullfile(folder,'process.json')));end

if isfile(fullfile(folder,'status.txt'))

 st=readmatrix(fullfile(folder,'status.txt'),'FileType','text');xx=readmatrix(fullfile(folder,'states.txt'),'FileType','text');u=readmatrix(fullfile(folder,'controls.txt'),'FileType','text');

 R.X=xx(:,2:7)';R.S=u(:,2)';R.K=u(:,3)';R.W=u(:,4)';R.E=u(:,5)';R.Z=double(R.E>=0);

 R.solveResult=st(1);R.reportedObjective=st(2);R.cost=sum(abs(R.S))+P.regularization*sum(diff(R.K).^2);R.solverSeconds=st(3);R.status=strtrim(fileread(fullfile(folder,'result.txt')));

 R.travel=sum(abs(R.S));R.V=WarehouseValidate(R,P,1);R.rawVerified=R.V.accepted;

 R.inputChange=max([max(abs(R.X-seed.X),[],'all'),max(abs(R.S-seed.S)),max(abs(R.K-seed.K))]);

 R.solverCandidateAvailable=startsWith(R.status,'solved') || R.inputChange>1e-7;

 R.rawVerified=R.rawVerified && R.solverCandidateAvailable;

 R.boundsViolation=max([0,max(abs(R.S))-P.smax,max(abs(R.K))-1/P.radius]);

 R.rawVerified=R.rawVerified && R.boundsViolation<1e-7;

 if strcmp(method,'minlp')

  if isfile(fullfile(folder,'integers.txt')),z=readmatrix(fullfile(folder,'integers.txt'),'FileType','text');R.integerError=max(abs(z(:,2)-round(z(:,2))));else,R.integerError=inf;end

  R.rawVerified=R.rawVerified && R.integerError<1e-6;

 end

end

R.totalSeconds=toc(timerAll);

end

