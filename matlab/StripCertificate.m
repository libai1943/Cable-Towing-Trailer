function C=StripCertificate(P)
% A configuration-space disconnection certificate, independent of controls.
% Conventional fixed-tongue trailer: p=q+(L+a)*e(theta).
A=P.B+P.rear(2); b=P.rear(3); c=P.front(1); d=P.front(3);
thetaStar=atan2(b,A); phiStar=atan2(d,c);
barriers=[hypot(A,b)-A,hypot(c,d)-c];
C=struct('thetaBarrier',thetaStar,'phiBarrier',phiStar,...
    'minimumExtraWidthForLeavingBox',min(barriers),...
    'actualExtraWidth',P.right-P.left-2*P.clearance-A-c,...
    'rearBarrier',barriers(1),'frontBarrier',barriers(2));
C.margin=C.minimumExtraWidthForLeavingBox-C.actualExtraWidth;
C.certified=C.actualExtraWidth>=0 && C.margin>1e-10;
C.scope='Every continuous fixed-tongue rigid-trailer path in the strip; arbitrary forward/reverse controls. Target orientation must leave the angular box.';
end
