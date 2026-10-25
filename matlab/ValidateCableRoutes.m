function V=ValidateCableRoutes(T,P)
% Material-length planar routes, checked separately from car collision geometry.
lengthError=0;wall=inf;hits=0;maxElbowStep=0;previous=[];
for k=1:size(T,1)
    s=T(k,:);p=s(2:3);q=s(7:8);h=q+P.a*[cos(s(9)),sin(s(9))];
    C=SlackCableRoute(p,h,P.L);
    lengthError=max(lengthError,abs(sum(vecnorm(diff(C),2,2))-P.L));
    wall=min([wall;C(:,1)-P.left;P.right-C(:,1)]);
    for j=1:size(C,1)-1
        hits=hits+SegmentBodyIntersects(C(j,:),C(j+1,:),p,s(4),P.front);
        hits=hits+SegmentBodyIntersects(C(j,:),C(j+1,:),q,s(9),P.rear);
        if isfield(P,'obstacles')
            for ob=1:size(P.obstacles,1)
                b=P.obstacles(ob,:);center=[mean(b(1:2)),mean(b(3:4))];
                shape=[diff(b(1:2))/2,diff(b(1:2))/2,diff(b(3:4))/2];
                hits=hits+SegmentBodyIntersects(C(j,:),C(j+1,:),center,0,shape);
            end
        end
    end
    if size(C,1)==3
        if ~isempty(previous),maxElbowStep=max(maxElbowStep,norm(C(2,:)-previous));end
        previous=C(2,:);
    else,previous=[];end
end
V=struct('maximumCableLengthError',lengthError,'minimumCableWallDistance',wall,...
    'bodyInteriorIntersections',hits,'maximumElbowSampleDisplacement',maxElbowStep);
V.accepted=lengthError<1e-7 && wall>P.clearance && hits==0;
end
