function RunConfinedPairedRecovery()
cfg=jsondecode(fileread('research/warehouse_v6/final_method.json'));folder='results/warehouse_v6/paired';if ~isfolder(folder),mkdir(folder);end
for c=1:9
 d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',c));P=d.P;w=d.w;
 label=sprintf('confined_paired_W%02d_no_terminal_%s',c,cfg.method);dest=fullfile(folder,[label '.mat']);if isfile(dest),continue;end
 z=load(sprintf('results/warehouse_v6/%s/confined_%s_W%02d_%s.mat',cfg.group,cfg.group,c,cfg.method));original=z.R;
 R=original;R.stages=original.stages(1:original.prefix.stageCount);R=FinalizeSquareRecord(R,w,P);
 timer=tic;[repair,report]=WarehouseRecoveryNoTerminal(R.selected,R.stages,P,max(0,30-original.prefix.seconds));
 R.repairReport=report;if ~isempty(repair),R.stages{end+1}=repair;R=FinalizeSquareRecord(R,w,P);end
 R.method=['no_terminal_' cfg.method];R.totalSeconds=original.prefix.seconds+toc(timer);R.parentMethod=cfg.method;
 R.timingScope='Original measured prefix plus separately replayed component; not a fresh full-pipeline runtime';
 save(dest,'R','P');record=R;record.P=P;
 for name={'stages','selected','optimizerCandidate'},if isfield(record,name{1}),record=rmfield(record,name{1});end;end
 fid=fopen(strrep(dest,'.mat','.json'),'w');fwrite(fid,jsonencode(record,PrettyPrint=true));fclose(fid);
 fprintf('%s new=%d J=%.6f\n',label,R.rawVerified,R.retainedCost);
end
end
