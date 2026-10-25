function WriteWarehouseMarginAMPL(seed,P,method,folder,opts)
% The physical model is unchanged. Conservative internal margins are an
% algorithmic refinement device; final acceptance uses the common P.
WriteWarehouseAMPL(seed,P,method,folder,opts);
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
copyfile(fullfile(root,'ampl/warehouse_v2/rope_replay_margin.mod'),fullfile(folder,'common.mod'));
fid=fopen(fullfile(folder,'problem.dat'),'a');guard=onCleanup(@()fclose(fid));
fprintf(fid,'param replayMargin :=\n');
for t=1:size(opts.margin,1),for b=1:2,fprintf(fid,'%d %d %.17g\n',t-1,b,opts.margin(t,b));end;end
fprintf(fid,';\n');
end
