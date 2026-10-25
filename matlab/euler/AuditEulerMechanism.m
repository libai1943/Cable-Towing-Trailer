function AuditEulerMechanism()
f=dir('results/euler/mechanism/G*.mat');rows=struct([]);
for i=1:numel(f)
 d=load(fullfile(f(i).folder,f(i).name));R=d.R;P=d.P;N=size(R.X,2);T=zeros(N,9);
 for j=1:N,p=EulerPose(R.X(:,j),P);T(j,:)=[j-1,p(1:3)',0,0,p(6:7)',p(5)];end
 V=ValidateCableRoutes(T,P);row=struct('caseName',f(i).name,'routeAccepted',V.accepted,'maximumMaterialLengthError',V.maximumCableLengthError,'cableBodyHits',V.bodyInteriorIntersections,'minimumCableWallDistance',V.minimumCableWallDistance);
 if i==1,rows=row;else,rows(i)=row;end
end
writetable(struct2table(rows),'results/euler/mechanism/cable_audit.csv');disp(struct2table(rows));
end
