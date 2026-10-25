function RunConfinedCoreAblations()
RunConfinedStudy(1:9,{'rrp'},'ablations');
folder='results/warehouse_v6/ablations';if ~isfolder(folder),mkdir(folder);end
for c=1:9
 d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',c));P=d.P;w=d.w;
 label=sprintf('confined_ablations_W%02d_max_recover_priority',c);dest=fullfile(folder,[label '.mat']);if isfile(dest),continue;end
 R=WarehouseMaxRecoveryPlanner(w,P,struct('label',label,'timeLimit',30));save(dest,'R','P');record=R;record.P=P;
 for name={'stages','selected','optimizerCandidate'},if isfield(record,name{1}),record=rmfield(record,name{1});end;end
 fid=fopen(strrep(dest,'.mat','.json'),'w');fwrite(fid,jsonencode(record,PrettyPrint=true));fclose(fid);
 fprintf('%s new=%d J=%.6f seconds=%.2f\n',label,R.rawVerified,R.retainedCost,R.totalSeconds);
end
end
