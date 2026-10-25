function [C,err,chordBound]=WarehouseAsymmetricCable(p,h,L,beta,side)
% Smooth cubic routing curve, with a fixed material length. Its extra shape
% freedom is used only by the mechanism-study routing audit, not the NLP.
if beta==0,[C,err,sag]=WarehouseCable(p,h,L,side);chordBound=sag/32^2;return;end
p=p(:)';h=h(:)';r=p-h;d=norm(r);normal=side*[-r(2),r(1)]/d;
if d>=L-1e-10,C=[h;p];err=abs(d-L);chordBound=0;return;end
persistent u weights
if isempty(u)
 k=(1:127)';off=k./sqrt(4*k.^2-1);[V,D]=eig(diag(off,1)+diag(off,-1));[z,id]=sort(diag(D));u=(z+1)/2;weights=V(1,id).^2;
end
derivative=@(t)4*((1-beta)+2*(-1+3*beta).*t-6*beta.*t.^2);
df=derivative(u);lo=0;hi=L;
for j=1:44
 alpha=(lo+hi)/2;len=weights*sqrt(d*d+(alpha*df).^2);
 if len>L,hi=alpha;else,lo=alpha;end
end
alpha=(lo+hi)/2;t=linspace(0,1,129)';f=4*t.*(1-t).*(1+beta*(2*t-1));C=h+t*r+alpha*f*normal;
lenCheck=integral(@(t)sqrt(d*d+(alpha*derivative(t)).^2),0,1,'AbsTol',1e-12,'RelTol',1e-12);
err=abs(lenCheck-L);chordBound=alpha*max(abs([8*(-1+3*beta),-8-24*beta]))/(8*128^2);
end
