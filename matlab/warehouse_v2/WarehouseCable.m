function [C,err,sag]=WarehouseCable(p,h,L,side)
% Smooth quadratic cable route. Analytic arc length is exactly L; no stretch.
if nargin<4,side=-1;end
p=p(:)';h=h(:)';d=norm(p-h);n=(p-h)/d;normal=side*[-n(2),n(1)];
if d>=L-1e-10,C=[h;p];err=abs(d-L);sag=0;return;end
lo=0;hi=L;
for it=1:38
 a=(lo+hi)/2;len=.5*sqrt(d*d+16*a*a)+d*d/(8*a)*asinh(4*a/d);
 if len>L,hi=a;else,lo=a;end
end
sag=(lo+hi)/2;t=linspace(0,1,33)';C=h+t*(p-h)+4*sag*t.*(1-t)*normal;
err=abs(.5*sqrt(d*d+16*sag*sag)+d*d/(8*sag)*asinh(4*sag/d)-L);
end
