function RunConfinedMatchedAblations()
out='results/warehouse_v6/matched';if ~isfolder(out),mkdir(out);end
variants={'no_initial','no_reserve','no_cap'};
for c=1:9
 d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',c));P=d.P;w=d.w;
 order=circshift(1:3,mod(c,3));
 for j=order
  variant=variants{j};label=sprintf('confined_matched_W%02d_fi_%s',c,variant);dest=fullfile(out,[label '.mat']);if isfile(dest),continue;end
  R=WarehouseInvokePlanner(w,P,['fi_' variant],struct('label',label,'timeLimit',30));save(dest,'R','P');record=R;record.P=P;
  for name={'stages','selected','optimizerCandidate'},if isfield(record,name{1}),record=rmfield(record,name{1});end;end
  fid=fopen(strrep(dest,'.mat','.json'),'w');fwrite(fid,jsonencode(record,PrettyPrint=true));fclose(fid);
  fprintf('%s new=%d J=%.6f seconds=%.2f\n',label,R.rawVerified,R.retainedCost,R.totalSeconds);
 end
end
end
