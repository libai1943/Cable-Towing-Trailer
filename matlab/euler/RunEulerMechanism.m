function RunEulerMechanism()
C=EulerCases('mechanism');rows=struct([]);out='results/euler/mechanism';if ~isfolder(out),mkdir(out);end
for c=1:numel(C)
 P=C(c).P;cert=StripCertificate(P);x=P.initial;X=x;S=[];K=[];W=[];D=norm(x(1:2)-x(4:5));
 % Reverse quarter turn, then forward along the aisle; every step is Euler.
 ns=ceil(P.radius*pi/2/.005);ds=-P.radius*pi/2/ns;
 for j=1:ns,advance(ds,-1/P.radius);end
 for j=1:5000
  p=EulerPose(x,P);if p(7)>=6.1 && abs(x(6)-pi/2)<=.006,break;end
  advance(.005,0);
 end
 R=struct('X',X,'S',S,'K',K,'W',W,'P',P,'travel',sum(abs(S)));
 V=EulerValidate(R,P,1);R.V=V;
 row=struct('caseName',C(c).name,'L',P.L,'radius',P.radius,'extra',P.extra,'rigidCertifiedImpossible',cert.certified,...
  'certificateMargin',cert.margin,'ropeAccepted',V.accepted,'tractorTravel',R.travel,'minimumEndpointDistance',min(D),...
  'maximumEndpointDistance',max(D),'materialCableLength',P.L,'minimumClearance',min([V.wall,V.self,V.obstacle]),'headingError',V.headingError);
 if c==1,rows=row;else,rows(c)=row;end
 save(fullfile(out,[C(c).name '.mat']),'R','P','cert','row');
 fprintf('%s rigid_impossible=%d rope_pass=%d gap_range=[%.5f %.5f]\n',C(c).name,cert.certified,V.accepted,min(D),max(D));
end
writetable(struct2table(rows),fullfile(out,'mechanism.csv'));save(fullfile(out,'mechanism.mat'),'rows');
 function advance(s,k)
  [x,w]=EulerRopeStep(x,s,k,P);X(:,end+1)=x;S(end+1)=s;K(end+1)=k;W(end+1)=w;D(end+1)=norm(x(1:2)-x(4:5));
 end
end
