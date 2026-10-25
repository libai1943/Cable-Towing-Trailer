function RunConfinedFactorialExtension()
out='results/warehouse_v6/factorial';if ~isfolder(out),mkdir(out);end
variants={'no_initial_no_reserve','no_initial_no_cap'};
for c=1:9
 d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',c));P=d.P;w=d.w;
 for j=circshift(1:2,mod(c,2))
  variant=variants{j};label=sprintf('confined_factorial_W%02d_fx_%s',c,variant);dest=fullfile(out,[label '.mat']);if isfile(dest),continue;end
  timer=tic;
  try
   R=WarehouseFactorialPlanner(w,P,struct('label',label,'timeLimit',30),variant);
  catch problem
   R=struct('method',['fx_' variant],'stages',{{}},'status','pipeline_exception','exceptionMessage',problem.message);
   R=FinalizeSquareRecord(R,w,P);R.totalSeconds=toc(timer);fprintf('Recorded pipeline failure: %s\n',problem.message);
  end
  save(dest,'R','P');record=R;record.P=P;
  for name={'stages','selected','optimizerCandidate'},if isfield(record,name{1}),record=rmfield(record,name{1});end;end
  fid=fopen(strrep(dest,'.mat','.json'),'w');fwrite(fid,jsonencode(record,PrettyPrint=true));fclose(fid);
  fprintf('%s new=%d J=%.6f seconds=%.2f\n',label,R.rawVerified,R.retainedCost,R.totalSeconds);
 end
end
end
