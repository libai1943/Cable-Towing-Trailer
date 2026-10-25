function RunConfinedStudy(ids,methods,group,prefix)

if nargin<1,ids=1:9;end

if nargin<2,methods={'recover','max','logic','complementarity','fixed','portfolio','minlp','rgr'};end

if nargin<3,group='main';end

if nargin<4,prefix='W';end

out=fullfile('results/warehouse_v6',group);if ~isfolder(out),mkdir(out);end

for id=ids

 tag=sprintf('%s%02d',prefix,id);d=load(fullfile('results/warehouse_v6/seeds',[tag '.mat']));P=d.P;w=d.w;w.audit=d.a;

 order=circshift(1:numel(methods),mod(id,numel(methods)));

 for j=order

  method=methods{j};label=sprintf('confined_%s_%s_%s',group,tag,method);dest=fullfile(out,[label '.mat']);if isfile(dest),continue;end

  opts=struct('label',label,'timeLimit',30);R=WarehouseSquareStrategy(w,P,method,opts);save(dest,'R','P');

  record=R;record.P=P;

  for name={'stages','selected','optimizerCandidate'},if isfield(record,name{1}),record=rmfield(record,name{1});end;end

  fid=fopen(strrep(dest,'.mat','.json'),'w');fwrite(fid,jsonencode(record,PrettyPrint=true));fclose(fid);

  fprintf('%s new=%d nlp=%d retained=%d improve=%d recovery=%d J=%.6f seconds=%.2f\n',label,R.rawVerified,R.rawSolverAccepted,R.retainedVerified,R.improved,R.recoverySelected,R.retainedCost,R.totalSeconds);

 end

end

end

