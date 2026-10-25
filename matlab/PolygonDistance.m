function d=PolygonDistance(A,B)
% Independent Euclidean convex polygon distance, with SAT overlap detection.
separated=false;
for C={A,B}
    V=C{1}; E=circshift(V,-1,1)-V;
    for j=1:size(E,1)
        n=[-E(j,2);E(j,1)]; aa=A*n; bb=B*n;
        if max(aa)<min(bb)-1e-12 || max(bb)<min(aa)-1e-12
            separated=true;
        end
    end
end
if ~separated, d=0; return; end
d=inf;
for k=1:2
    if k==1, V=A; W=B; else, V=B; W=A; end
    for i=1:size(V,1)
        for j=1:size(W,1)
            a=W(j,:); b=W(mod(j,size(W,1))+1,:); ab=b-a;
            u=max(0,min(1,dot(V(i,:)-a,ab)/dot(ab,ab)));
            d=min(d,norm(V(i,:)-a-u*ab));
        end
    end
end
end
