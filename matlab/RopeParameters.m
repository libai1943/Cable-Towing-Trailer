function P=RopeParameters()
% Compact logistics tug and passive cart. Distances m, times s, angles rad.
% Ideal cable-aligned hitch mobility; rho=1 is the nominal kinematic closure.
P.L=1.3; P.a=.7; P.Lf=.4;
P.front=[.6 0 .35]; % front extent, rear extent, half width from rear axle
P.rear=[.7 .35 .55];
P.rho=1; P.deltaMax=pi/4; P.vMax=.5; P.aMax=.5; P.wMax=.8;
P.clearance=.10; P.extra=.04; P.scale=1;
P.planBuffer=.015;
P.B=P.L+P.a; P.radius=P.Lf/tan(P.deltaMax);
P.left=-P.rear(2)-P.clearance-P.extra/2;
P.right=P.B+P.front(1)+P.clearance+P.extra/2;
P.ymin=-2; P.ymax=12; P.pull=9; P.sample=.01;
end
