# Common first-order Euler transcription. Only mode_*.mod is exchanged.
# State: tractor rear axle p, heading phi, trailer hitch h, heading theta.
# Trailer axle q=h-a*e(theta) is an algebraic output, never a separate state.
param N integer > 1;
set T := 0..N; set I := 0..N-1; set B := 1..2; set E := 1..4; param nb{B} integer > 2; set BE{b in B} := 1..nb[b];
param L > 0; param a > 0; param kmax > 0; param smax > 0;
param xmin; param xmax; param ymin; param ymax;
param clearance > 0; param buffer >= 0;
param initial{1..6}; param goal{1..2}; param goalXYTolerance{1..2}; param goalAngle; param goalTolerance;
param G{b in B,BE[b],1..2}; param body{b in B,BE[b]};
param O integer >= 0; set OBS := 1..O;
param obs{OBS,E};
param zseed{I} binary;
param epscomp >= 0 default 0;
param smoothbeta > 0 default 100;
param regularization >= 0 default 0.002;
param objectiveMode integer default 0;
param Xseed{T,1..6}; param Sseed{I}; param Kseed{I};
param obstacleMargin{T,B,OBS} >= 0 default 0;
param wallMargin{T,B,E} >= 0 default 0;
param selfMargin{T} >= 0 default 0;
var X{T,1..6};
var S{I} >= -smax, <= smax;
var K{I} >= -kmax, <= kmax;
var W{I} >= 0, <= smax;
var absS{I} >= 0, <= smax;
var D{t in T} = sqrt((X[t,1]-X[t,4])^2+(X[t,2]-X[t,5])^2);
var nx{t in I} = (X[t,1]-X[t,4])/D[t];
var ny{t in I} = (X[t,2]-X[t,5])/D[t];
var c{t in I} = nx[t]*cos(X[t,3])+ny[t]*sin(X[t,3]);
var e{t in I} = D[t]+S[t]*c[t]-sqrt(L^2-S[t]^2*(1-c[t]^2));
var px{t in T,b in B} = if b=1 then X[t,1] else X[t,4]-a*cos(X[t,6]);
var py{t in T,b in B} = if b=1 then X[t,2] else X[t,5]-a*sin(X[t,6]);
var th{t in T,b in B} = if b=1 then X[t,3] else X[t,6];
subject to initial_state{j in 1..6}: X[0,j]=initial[j];
subject to bound_x{t in T,j in {1,4}}: xmin <= X[t,j] <= xmax;
subject to bound_y{t in T,j in {2,5}}: ymin <= X[t,j] <= ymax;
subject to bound_angle{t in T,j in {3,6}}: -3.141592653589793 <= X[t,j] <= 3.141592653589793;
subject to length_bound{t in T}: 0.25 <= D[t] <= L;
subject to travel_positive{t in I}: absS[t]>=S[t];
subject to travel_negative{t in I}: absS[t]>=-S[t];
subject to front_x{t in I}: X[t+1,1]=X[t,1]+S[t]*cos(X[t,3]);
subject to front_y{t in I}: X[t+1,2]=X[t,2]+S[t]*sin(X[t,3]);
subject to front_heading{t in I}: X[t+1,3]=X[t,3]+S[t]*K[t];
subject to hitch_x{t in I}: X[t+1,4]=X[t,4]+W[t]*nx[t];
subject to hitch_y{t in I}: X[t+1,5]=X[t,5]+W[t]*ny[t];
subject to rear_heading{t in I}: X[t+1,6]=X[t,6]+W[t]/a*(ny[t]*cos(X[t,6])-nx[t]*sin(X[t,6]));
subject to rear_no_reverse{t in I}: W[t]*(nx[t]*cos(X[t,6])+ny[t]*sin(X[t,6])) >= 0;
subject to target_x: goal[1]-goalXYTolerance[1] <= px[N,2] <= goal[1]+goalXYTolerance[1];
subject to target_y: goal[2]-goalXYTolerance[2] <= py[N,2] <= goal[2]+goalXYTolerance[2];
subject to target_rear_heading: goalAngle-goalTolerance <= X[N,6] <= goalAngle+goalTolerance;
subject to target_front_heading: X[N,3]=goalAngle;
# General convex footprint OBCA, including occupied boundary halfspaces.
param wallnx{s in E} := if s=1 then 1 else if s=2 then -1 else 0;
param wallny{s in E} := if s=3 then 1 else if s=4 then -1 else 0;
param wallg{s in E} := if s=1 then xmin else if s=2 then -xmax else if s=3 then ymin else -ymax;
var wallmu{T,b in B,E,BE[b]} >= 0;
subject to wall_dual_x{t in T,b in B,s in E}:
 sum{j in BE[b]} G[b,j,1]*wallmu[t,b,s,j]+cos(th[t,b])*wallnx[s]+sin(th[t,b])*wallny[s]=0;
subject to wall_dual_y{t in T,b in B,s in E}:
 sum{j in BE[b]} G[b,j,2]*wallmu[t,b,s,j]-sin(th[t,b])*wallnx[s]+cos(th[t,b])*wallny[s]=0;
subject to wall_separation{t in T,b in B,s in E}:
 px[t,b]*wallnx[s]+py[t,b]*wallny[s]-wallg[s]-sum{j in BE[b]}body[b,j]*wallmu[t,b,s,j]>=clearance+buffer+wallMargin[t,b,s];
var lam{T,B,OBS,E} >= 0; var mu{T,b in B,OBS,BE[b]} >= 0;
var ox{t in T,b in B,o in OBS}=lam[t,b,o,1]-lam[t,b,o,2];
var oy{t in T,b in B,o in OBS}=lam[t,b,o,3]-lam[t,b,o,4];
subject to obstacle_dual_x{t in T,b in B,o in OBS}:
 sum{j in BE[b]}G[b,j,1]*mu[t,b,o,j]+cos(th[t,b])*ox[t,b,o]+sin(th[t,b])*oy[t,b,o]=0;
subject to obstacle_dual_y{t in T,b in B,o in OBS}:
 sum{j in BE[b]}G[b,j,2]*mu[t,b,o,j]-sin(th[t,b])*ox[t,b,o]+cos(th[t,b])*oy[t,b,o]=0;
subject to obstacle_normal{t in T,b in B,o in OBS}: ox[t,b,o]^2+oy[t,b,o]^2<=1;
subject to obstacle_separation{t in T,b in B,o in OBS}:
 px[t,b]*ox[t,b,o]+py[t,b]*oy[t,b,o]-sum{j in E}obs[o,j]*lam[t,b,o,j]-sum{j in BE[b]}body[b,j]*mu[t,b,o,j]>=clearance+buffer+obstacleMargin[t,b,o];
var selflam{T,BE[2]} >= 0; var selfmu{T,BE[1]} >= 0;
var slx{t in T}=sum{j in BE[2]}G[2,j,1]*selflam[t,j];
var sly{t in T}=sum{j in BE[2]}G[2,j,2]*selflam[t,j];
var snx{t in T}=cos(th[t,2])*slx[t]-sin(th[t,2])*sly[t];
var sny{t in T}=sin(th[t,2])*slx[t]+cos(th[t,2])*sly[t];
subject to self_dual_x{t in T}:
 sum{j in BE[1]}G[1,j,1]*selfmu[t,j]+cos(th[t,1])*snx[t]+sin(th[t,1])*sny[t]=0;
subject to self_dual_y{t in T}:
 sum{j in BE[1]}G[1,j,2]*selfmu[t,j]-sin(th[t,1])*snx[t]+cos(th[t,1])*sny[t]=0;
subject to self_normal{t in T}: snx[t]^2+sny[t]^2<=1;
subject to self_separation{t in T}:
 (px[t,1]-px[t,2])*snx[t]+(py[t,1]-py[t,2])*sny[t]-sum{j in BE[1]}body[1,j]*selfmu[t,j]-sum{j in BE[2]}body[2,j]*selflam[t,j]>=clearance+buffer+selfMargin[t];
minimize cost: if objectiveMode=0 then sum{t in I}absS[t]+regularization*sum{t in 1..N-1}(K[t]-K[t-1])^2
 else sum{t in I}(S[t]-Sseed[t])^2+0.01*sum{t in I}(K[t]-Kseed[t])^2+0.001*sum{t in T,j in 1..6}(X[t,j]-Xseed[t,j])^2;
