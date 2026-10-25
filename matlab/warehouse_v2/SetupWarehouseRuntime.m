function SetupWarehouseRuntime(runtimeSource)
% Use an explicitly supplied, locally licensed Windows solver installation.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
runtime=fullfile(root,'results/warehouse_v2/runtime');
if ~isfolder(runtime),mkdir(runtime);end
files={'ampl.exe','ampl.lic','libhsl.dll','ipopt.exe','bonmin.exe','libipoptfort.dll'};
if nargin>0
 for k=1:numel(files)
  source=fullfile(runtimeSource,files{k});assert(isfile(source),'Missing runtime file: %s',files{k});
  copyfile(source,fullfile(runtime,files{k}));
 end
end
for k=1:numel(files)
 assert(isfile(fullfile(runtime,files{k})),'Run RunStudy(''setup'',runtimeDirectory) with your licensed AMPL/COIN/HSL installation.');
end
end
