function RunConfinedSafeBudgets()

cfg=jsondecode(fileread('research/warehouse_v6/final_method.json'));

out='results/warehouse_v6/budgets';if ~isfolder(out),mkdir(out);end

for budget=[15,60]

 for c=[2,5,9]

  d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',c));P=d.P;w=d.w;

  methods={cfg.method,'max','minlp'};order=circshift(1:3,mod(c,3));

  for j=order

   method=methods{j};label=sprintf('confined_budget%d_W%02d_%s',budget,c,method);dest=fullfile(out,[label '.mat']);if isfile(dest),continue;end

   opts=struct('label',label,'timeLimit',budget);R=WarehouseInvokePlanner(w,P,method,opts);

   save(dest,'R','P');record=R;record.P=P;

   for name={'stages','selected','optimizerCandidate'},if isfield(record,name{1}),record=rmfield(record,name{1});end;end

   fid=fopen(strrep(dest,'.mat','.json'),'w');fwrite(fid,jsonencode(record,PrettyPrint=true));fclose(fid);

   fprintf('%s new=%d J=%.6f seconds=%.2f\n',label,R.rawVerified,R.retainedCost,R.totalSeconds);

  end

 end

end

end

