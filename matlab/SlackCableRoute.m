function C=SlackCableRoute(p,h,L)
% Continuous equal-length V route on the clockwise side of h -> p.
p=p(:)';h=h(:)';r=p-h;d=norm(r);
if d>=L-1e-12,C=[h;p];return;end
n=[-r(2),r(1)]/max(d,eps);
mid=(p+h)/2-.5*sqrt(max(0,L^2-d^2))*n;
C=[h;mid;p];
end
