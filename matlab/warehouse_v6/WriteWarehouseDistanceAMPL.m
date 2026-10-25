function WriteWarehouseDistanceAMPL(seed,P,method,folder,opts)
WriteWarehouseAMPL(seed,P,method,folder,opts);
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
copyfile(fullfile(root,'ampl/warehouse_v6/rope_distance_margin.mod'),fullfile(folder,'common.mod'));
fid=fopen(fullfile(folder,'problem.dat'),'a');guard=onCleanup(@()fclose(fid));M=opts.distanceMargin;
fprintf(fid,'param obstacleMargin :=\n');
for k=1:size(M.obstacle,1),for b=1:2,for j=1:size(M.obstacle,3),fprintf(fid,'%d %d %d %.17g\n',k-1,b,j,M.obstacle(k,b,j));end;end;end
fprintf(fid,';\nparam wallMargin :=\n');
for k=1:size(M.wall,1),for b=1:2,for j=1:4,fprintf(fid,'%d %d %d %.17g\n',k-1,b,j,M.wall(k,b,j));end;end;end
fprintf(fid,';\nparam selfMargin :=\n');for k=1:numel(M.self),fprintf(fid,'%d %.17g\n',k-1,M.self(k));end;fprintf(fid,';\n');
end
