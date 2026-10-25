function hit=SegmentBodyIntersects(a,b,p,theta,g)
% Segment enters open rectangle interior; endpoint contact at a hitch allowed.
R=[cos(theta),-sin(theta);sin(theta),cos(theta)];
aa=R'*(a(:)-p(:));bb=R'*(b(:)-p(:));v=bb-aa;
lo=[-g(2);-g(3)]+1e-9;hi=[g(1);g(3)]-1e-9;
t0=0;t1=1;hit=false;
for k=1:2
    if abs(v(k))<1e-14
        if aa(k)<lo(k) || aa(k)>hi(k),return;end
    else
        ts=sort(([lo(k);hi(k)]-aa(k))/v(k));
        t0=max(t0,ts(1));t1=min(t1,ts(2));
        if t0>t1,return;end
    end
end
hit=t1>t0+1e-10;
end
