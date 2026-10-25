function RunStudy(action,argument)
% Reproduce the published study from archived controls and the common model.
if nargin<1,action='results';end
root=fileparts(mfilename('fullpath'));cd(root);
addpath('matlab');addpath('matlab/euler');addpath('matlab/warehouse_v2');
addpath('matlab/warehouse_v3');addpath('matlab/warehouse_v4');addpath('matlab/warehouse_v6');
switch lower(action)
 case 'results'
  d=jsondecode(fileread('results/warehouse_v6/summary.json'));disp(struct2table(d.main));
 case 'verify',VerifyConfinedResults;
 case 'maps',WarehouseConfinedMapFigure(true);
 case 'figures',WarehouseConfinedFigures;
 case 'setup'
  assert(nargin>1,'Supply the directory containing your licensed AMPL/COIN/HSL runtime.');
  SetupWarehouseRuntime(argument);
 case 'main'
  SetupWarehouseRuntime;
  RunConfinedSafeStudy(1:9,{'max','logic','complementarity','fixed','minlp','rgr'},'main');
  RunConfinedSafeStudy(1:9,{'fi_no_cap'},'confirm');RunConfinedExtensions('portfolio_unique');
 case 'extensions'
  SetupWarehouseRuntime;RunConfinedCoreAblations;
  RunConfinedSafeStudy(1:9,{'recover_feasible'},'final');
  RunConfinedMatchedAblations;RunConfinedFactorialExtension;RunConfinedPairedRecovery;
  RunConfinedExtensions('rigid');RunConfinedExtensions('smoothing');
  for k=1:3,RunConfinedSafeStudy([2,5,9],{'fi_no_cap','max'},sprintf('repeat%d',k));end
  RunConfinedSafeShapes;RunConfinedSafeBudgets;
 case 'all'
  RunStudy('main');RunStudy('extensions');RunStudy('verify');
 case 'workspace'
  assert(nargin>1,'Supply an unused directory for an independent reproduction.');
  assert(~isfolder(argument)&&~isfile(argument),'Destination exists; choose a new directory.');
  mkdir(argument);
  for name={'matlab','ampl','tools','research','paper'},copyfile(name{1},fullfile(argument,name{1}));end
  copyfile('RunStudy.m',fullfile(argument,'RunStudy.m'));
  mkdir(fullfile(argument,'results','warehouse_v6'));
  mkdir(fullfile(argument,'results','warehouse_v2'));
  mkdir(fullfile(argument,'results','euler'));
  for folder={'seeds','shape_seeds','train','layouts'}
   path=fullfile('results/warehouse_v6',folder{1});if isfolder(path),copyfile(path,fullfile(argument,path));end
  end
  for path={'results/warehouse_v2/mechanism','results/euler/mechanism','results/warehouse_v2/seeds'}
   copyfile(path{1},fullfile(argument,path{1}));
  end
  fprintf('Created %s. Change there, configure RunStudy(''setup'',runtime), then RunStudy(''all'').\n',argument);
 otherwise,error('Choose results, verify, maps, figures, setup, main, extensions, all, or workspace.');
end
end
