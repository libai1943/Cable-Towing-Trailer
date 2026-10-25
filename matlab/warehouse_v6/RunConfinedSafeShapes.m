function RunConfinedSafeShapes()

cfg=jsondecode(fileread('research/warehouse_v6/final_method.json'));

for c=1:5

 d=load(sprintf('results/warehouse_v6/shape_seeds/S%02d.mat',c));P=d.P;w=d.w;

 for method={cfg.method,'max'}

  label=sprintf('confined_shapes_S%02d_%s',c,method{1});folder='results/warehouse_v6/shapes';if ~isfolder(folder),mkdir(folder);end

  dest=fullfile(folder,[label '.mat']);if isfile(dest),continue;end

  opts=struct('label',label,'timeLimit',30);R=WarehouseInvokePlanner(w,P,method{1},opts);

   save(dest,'R','P');record=R;record.P=P;

  for name={'stages','selected','optimizerCandidate'},if isfield(record,name{1}),record=rmfield(record,name{1});end;end

  fid=fopen(strrep(dest,'.mat','.json'),'w');fwrite(fid,jsonencode(record,PrettyPrint=true));fclose(fid);

  fprintf('%s new=%d J=%.6f seconds=%.2f\n',label,R.rawVerified,R.retainedCost,R.totalSeconds);

 end

end

end

