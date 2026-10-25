function WarehouseReauditMechanism()
% Read-only use of the archived mechanism witnesses, with a new curved-cable audit.
out='results/warehouse_v2/mechanism';if ~isfolder(out),mkdir(out);end
rows=cell(1,18);
for i=1:18
 d=load(sprintf('results/euler/mechanism/G%02d.mat',i));
 P=d.P;R=d.R;P.vertices={BodyPolygon([0,0],0,P.front),BodyPolygon([0,0],0,P.rear)};
 P.goal=R.X(4:5,end)-P.a*[cos(R.X(6,end));sin(R.X(6,end))];P.goalAngle=pi/2;
 P.goalXYTolerance=[.12;.12];P.replayTolerance=.15;P.goalTolerance=.008;P.taskGoalTolerance=.025;P.smax=.18;P.regularization=.001;
 A=AuditWarehouseTrajectory(R,P);symmetricAudit=A;routeAudit=WarehouseFlexibleCableAudit(R.X,P);
 finer=WarehouseValidate(R,P,8);fineRouteAudit=WarehouseFlexibleCableAudit(finer.nodes,P);
 A.accepted=A.stateAccepted&&A.fineStateAccepted&&A.boundsViolation<1e-7&&routeAudit.accepted&&fineRouteAudit.accepted;
 A.cableAccepted=routeAudit.accepted;A.cableHits=routeAudit.hits;A.cableLengthError=routeAudit.lengthError;A.cableSideChanges=routeAudit.sideChanges;P.mechanismEnhancedRoute=true;
 cert=d.cert;row=d.row;row.newAuditAccepted=A.accepted;row.smoothCableAccepted=A.cableAccepted;row.fineClearance=A.fineClearance;row.cableLengthError=A.cableLengthError;row.asymmetricNodes=routeAudit.asymmetricNodes;row.symmetricRouteAccepted=symmetricAudit.cableAccepted;
 row.fineRouteAccepted=fineRouteAudit.accepted;rows{i}=row;save(fullfile(out,sprintf('G%02d.mat',i)),'P','R','cert','row','A','symmetricAudit','routeAudit','fineRouteAudit');
 fprintf('G%02d smoothCable=%d independentReplay=%d fineClearance=%.4f\n',i,A.cableAccepted,A.accepted,A.fineClearance);
end
writetable(struct2table([rows{:}]),fullfile(out,'mechanism.csv'));
end
