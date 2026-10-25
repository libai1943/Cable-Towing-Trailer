function WriteWarehouseAMPL(seed,P,method,folder,opts)

% Single common model and initial values; no method-specific state tuning.

root=fileparts(fileparts(fileparts(mfilename('fullpath'))));N=numel(seed.S);

fid=fopen(fullfile(folder,'problem.dat'),'w');guard=onCleanup(@()fclose(fid));

fprintf(fid,'data;\nparam N := %d;\n',N);

names={'L','a','kmax','smax','xmin','xmax','ymin','ymax','clearance','buffer','goalAngle','goalTolerance','regularization','O','epscomp','smoothbeta'};

values=[P.L,P.a,1/P.radius,P.smax,P.left,P.right,P.ymin,P.ymax,P.clearance,P.planBuffer,P.goalAngle,P.goalTolerance,P.regularization,size(P.obstacles,1),opts.epscomp,opts.beta];

for j=1:numel(names),fprintf(fid,'param %s := %.17g;\n',names{j},values(j));end

fprintf(fid,'param initial :=\n');for j=1:6,fprintf(fid,'%d %.17g\n',j,P.initial(j));end;fprintf(fid,';\n');

fprintf(fid,'param goal := 1 %.17g 2 %.17g;\nparam goalXYTolerance := 1 %.17g 2 %.17g;\n',P.goal,P.goalXYTolerance);

G=cell(1,2);gb=G;

fprintf(fid,'param nb := 1 %d 2 %d;\n',size(P.vertices{1},1),size(P.vertices{2},1));

for b=1:2,[G{b},gb{b}]=WarehouseHalfspaces(P.vertices{b});end

fprintf(fid,'param body :=\n');

for b=1:2,for j=1:numel(gb{b}),fprintf(fid,'%d %d %.17g\n',b,j,gb{b}(j));end;end;fprintf(fid,';\n');

fprintf(fid,'param G :=\n');

for b=1:2,for j=1:size(G{b},1),for d=1:2,fprintf(fid,'%d %d %d %.17g\n',b,j,d,G{b}(j,d));end;end;end;fprintf(fid,';\n');

fprintf(fid,'param obs :=\n');

for b=1:size(P.obstacles,1),g=P.obstacles(b,[2,1,4,3]).*[1,-1,1,-1];for j=1:4,fprintf(fid,'%d %d %.17g\n',b,j,g(j));end;end;fprintf(fid,';\n');

fprintf(fid,'param zseed :=\n');for i=1:N,fprintf(fid,'%d %d\n',i-1,seed.Z(i));end;fprintf(fid,';\n');

fprintf(fid,'param Xseed :=\n');for i=1:N+1,for j=1:6,fprintf(fid,'%d %d %.17g\n',i-1,j,seed.X(j,i));end;end;fprintf(fid,';\n');

fprintf(fid,'param Sseed :=\n');for i=1:N,fprintf(fid,'%d %.17g\n',i-1,seed.S(i));end;fprintf(fid,';\n');

fprintf(fid,'param Kseed :=\n');for i=1:N,fprintf(fid,'%d %.17g\n',i-1,seed.K(i));end;fprintf(fid,';\n');clear guard

fid=fopen(fullfile(folder,'initial.run'),'w');guard=onCleanup(@()fclose(fid));

fprintf(fid,'let {t in T,j in 1..6} X[t,j] := Xseed[t,j];\nlet {t in I} S[t] := Sseed[t];\nlet {t in I} K[t] := Kseed[t];\nlet {t in I} absS[t] := abs(Sseed[t]);\n');

for i=1:N,fprintf(fid,'let W[%d] := %.17g;\n',i-1,seed.W(i));end

if strcmp(method,'minlp'),fprintf(fid,'let {t in I} Z[t] := zseed[t];\n');end

for i=1:N+1

 x=seed.X(:,i);p=WarehousePose(x,P);polys={WarehouseBody(x,P,1),WarehouseBody(x,P,2)};

 th=[x(3),x(6)];pos=[p(1:2),p(6:7)];

 for b=1:2

  rot=[cos(th(b)),-sin(th(b));sin(th(b)),cos(th(b))];

  normals=[1,0;-1,0;0,1;0,-1];

  for side=1:4,n=normals(side,:)';m=WarehouseDual(G{b},gb{b},-rot'*n);for j=1:numel(m),fprintf(fid,'let wallmu[%d,%d,%d,%d] := %.17g;\n',i-1,b,side,j,m(j));end;end

  for ob=1:size(P.obstacles,1)

   box=P.obstacles(ob,:);C=[box(1),box(3);box(2),box(3);box(2),box(4);box(1),box(4)];

   n=separator(polys{b},C);la=positive(n);mu=WarehouseDual(G{b},gb{b},-rot'*n);

   for j=1:4,fprintf(fid,'let lam[%d,%d,%d,%d] := %.17g;\n',i-1,b,ob,j,la(j));end

   for j=1:numel(mu),fprintf(fid,'let mu[%d,%d,%d,%d] := %.17g;\n',i-1,b,ob,j,mu(j));end

  end

 end

 n=separator(polys{1},polys{2});r1=[cos(th(1)),-sin(th(1));sin(th(1)),cos(th(1))];r2=[cos(th(2)),-sin(th(2));sin(th(2)),cos(th(2))];

 la=WarehouseDual(G{2},gb{2},r2'*n);mu=WarehouseDual(G{1},gb{1},-r1'*n);

 for j=1:numel(la),fprintf(fid,'let selflam[%d,%d] := %.17g;\n',i-1,j,la(j));end

 for j=1:numel(mu),fprintf(fid,'let selfmu[%d,%d] := %.17g;\n',i-1,j,mu(j));end

end

clear guard

if strcmp(method,'rigid'),common='rigid_common.mod';else,common='rope_common.mod';end

copyfile(fullfile(root,'ampl/warehouse_v2',common),fullfile(folder,'common.mod'));

copyfile(fullfile(root,['ampl/warehouse_v2/mode_' method '.mod']),fullfile(folder,'mode.mod'));

fid=fopen(fullfile(folder,'run.run'),'w');guard=onCleanup(@()fclose(fid));

fprintf(fid,'reset;\noption presolve 0;\nmodel common.mod;\nmodel mode.mod;\ndata problem.dat;\ncommands initial.run;\n');

if strcmp(method,'minlp'),fprintf(fid,'option solver bonmin;\n');else,fprintf(fid,'option solver ipopt;\n');end

fprintf(fid,'solve;\nprintf "%%d %%.17g %%.17g\\n",solve_result_num,cost,_solve_elapsed_time > "status.txt";\n');

fprintf(fid,'printf {t in T} "%%d %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",t,X[t,1],X[t,2],X[t,3],X[t,4],X[t,5],X[t,6] > "states.txt";\n');

fprintf(fid,'printf {t in I} "%%d %%.17g %%.17g %%.17g %%.17g\\n",t,S[t],K[t],W[t],e[t] > "controls.txt";\n');

if strcmp(method,'minlp'),fprintf(fid,'printf {t in I} "%%d %%.17g\\n",t,Z[t] > "integers.txt";\n');end

fprintf(fid,'printf "%%s\\n",solve_result > "result.txt";\n');

clear guard

fid=fopen(fullfile(folder,'ipopt.opt'),'w');guard=onCleanup(@()fclose(fid));

fprintf(fid,'linear_solver ma57\nprint_level 5\nmax_iter %d\nmax_cpu_time %.6g\ntol 1e-7\nconstr_viol_tol 1e-7\nacceptable_tol 1e-6\nacceptable_iter 8\nmu_strategy adaptive\nbound_relax_factor 0\n',opts.maxIter,max(.05,opts.timeLimit-1.5));

clear guard

if strcmp(method,'minlp')

 fid=fopen(fullfile(folder,'bonmin.opt'),'w');guard=onCleanup(@()fclose(fid));

 fprintf(fid,'bonmin.algorithm B-BB\nbonmin.time_limit %.6g\nbonmin.bb_log_level 1\nbonmin.nlp_log_level 0\nlinear_solver ma57\nprint_level 5\nmax_iter %d\nmax_cpu_time %.6g\ntol 1e-7\nconstr_viol_tol 1e-7\nbound_relax_factor 0\nbonmin.num_retry_unsolved_random_point 0\nbonmin.nlp_failure_behavior fathom\nbonmin.number_strong_branch 0\nbonmin.solution_limit %d\n',max(.1,opts.timeLimit-2),opts.maxIter,opts.nlpTimeLimit,opts.solutionLimit);

 if opts.disablePump,fprintf(fid,'bonmin.heuristic_feasibility_pump no\nbonmin.pump_for_minlp no\nbonmin.num_resolve_at_root 0\nbonmin.num_resolve_at_node 0\n');end

 clear guard

end

end

function p=positive(n),p=[max(n(1),0);max(-n(1),0);max(n(2),0);max(-n(2),0)];end

function n=separator(A,B)

edges=[diff([A;A(1,:)]);diff([B;B(1,:)])];axes=[-edges(:,2),edges(:,1)];axes=axes./vecnorm(axes,2,2);

aa=A*axes';bb=B*axes';[~,j]=max([min(aa)-max(bb),min(bb)-max(aa)]);

if j<=size(axes,1),n=axes(j,:)';else,n=-axes(j-size(axes,1),:)';end

end

