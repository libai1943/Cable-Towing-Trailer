function layouts=WarehouseConfinedLayouts()

% Layout-first warehouse models. Rack modules, cross aisles and work zones

% are defined before any control trajectory. No swept-path space filling.

base=load('results/warehouse_v2/seeds/C02.mat','P');P0=base.P;

names={'Blocked-aisle transfer','Rack-end transfer','Cross-aisle exchange',...

 'Adjacent-aisle return','Offset storage blocks','Packing-area turn',...

 'Pallet staging bypass','Mixed storage return','Perimeter replenishment'};

out='results/warehouse_v6/layouts';if ~isfolder(out),mkdir(out);end

layouts=cell(1,9);

for c=1:9

 P=P0;P.left=0;P.right=16;P.ymin=0;P.ymax=16;P.obstacles=zeros(0,4);P.obstacleKinds={};

 P.caseName=sprintf('W%02d',c);P.name=names{c};P.layoutFamily=c;P.zones=[];P.zoneNames={};

 % Rows are deliberately finite rack modules, accessible from open aisles.

 switch c

  case 1

   rows([.4,4.1,7.8,11.5],2.2,[3.2,5.8;9.7,14.6]);

   add([2.65,4.05,10.4,11.6],'pallet');rack([7.8,10.0,6.8,8.4]);rack([11.5,13.7,6.8,8.4]);

   zone([.5,14.0,.25,2.7],'Receiving / dispatch');

  case 2

   rows([.4,4.1,7.8,11.5],2.2,[3.2,6.0;9.8,14.6]);

   zone([.5,14,.25,2.6],'Receiving');zone([.4,15.5,7.3,9.7],'Cross aisle');

  case 3

   rows([.4,4.1,7.8,11.5],2.2,[3.2,5.7;9.4,14.6]);

   rack([.4,2.6,7.0,8.2]);rack([7.8,10.0,6.8,8.0]);rack([11.5,13.7,6.8,8.0]);

   zone([.5,14,.25,2.6],'Transfer area');

  case 4

   rows([.4,4.1,7.8,11.5],2.2,[3.2,6.0;6.2,9.0]);

   for x=[.4,4.1,7.8,11.5],rack([x,x+2.2,12.2,15.2]);end

   zone([.5,14,.25,2.6],'Dispatch');zone([.5,14.5,10,12.8],'End cross aisle');

  case 5

   rows([.4,4.1,7.8,11.5],2.2,[2.4,4.4;12.4,15.0]);

   rows([1.85,5.55,9.25,12.95],2.2,[8.0,9.0]);

   zone([.5,14,.25,2.6],'Inbound buffer');zone([.5,15.5,7.0,9.0],'Transfer cross aisle');

  case 6

   rows([.4,4.1,7.8,11.5],2.2,[4.8,6.1;9.0,14.6]);

   for x=[.6,5.0,9.4,13.0],add([x,min(x+2.4,15.4),.5,1.4],'station');end

   zone([.5,15.4,1.7,3.8],'Packing / consolidation');

  case 7

   rows([.4,4.1,7.8,11.5],2.2,[3.2,6.6;10.2,14.6]);

   add([6.4,7.6,10.4,11.6],'pallet');add([10.0,11.2,6.6,7.6],'pallet');

   add([14.1,15.3,8.8,9.8],'pallet');

   zone([.5,6.2,.25,2.6],'Inbound');zone([7,15.3,7.9,10.7],'Staging access');

  case 8

   rack([.4,2.6,3.2,8.4]);rack([.4,2.6,8.6,14.5]);

   rack([4.1,6.3,3.5,6.0]);rack([4.1,6.3,10.0,14.5]);

   for y=[3.5,6.8,10.1,13.4],rack([7.8,10.8,y,y+1.4]);rack([12.3,15.3,y,y+1.4]);end

   zone([.5,14.8,.25,2.6],'Replenishment / outbound');

  case 9

   for x=[4.8,8.0],rack([x,x+1.8,4.0,9.3]);end

   rows([.4,13.0],2.0,[4.0,8.8;9.0,14.5]);

   for x=[4.8,8.0,11.2],rack([x,x+1.8,11.7,14.7]);end

   for x=[4.8,8.0,11.2],add([x,x+1.8,.4,2.1],'pallet');end

   zone([.5,14.5,.25,2.0],'Dispatch / replenishment');

 end

 % Nominal modular rack width is 2.0 m, with 1.7 m picking aisles.

 % This fixed layout parameter is independent of any trajectory search.

 for j=1:size(P.obstacles,1)

  if strcmp(P.obstacleKinds{j},'rack')&&abs(diff(P.obstacles(j,1:2))-2.2)<1e-8

   P.obstacles(j,1:2)=P.obstacles(j,1:2)+[.1,-.1];

  end

 end

 P.nominalAisleWidth=1.7;

 P.layoutArea=sum((P.obstacles(:,2)-P.obstacles(:,1)).*(P.obstacles(:,4)-P.obstacles(:,3)));

 assert(all(P.obstacles(:,2)-P.obstacles(:,1)<=2.4+1e-9 | P.obstacles(:,4)-P.obstacles(:,3)<=1.8+1e-9),'Non-rack solid block');

 for i=1:size(P.obstacles,1),for j=1:i-1

  o=P.obstacles(i,:);b=P.obstacles(j,:);overlap=min(o(2),b(2))-max(o(1),b(1));depth=min(o(4),b(4))-max(o(3),b(3));assert(overlap<=1e-9||depth<=1e-9);

 end;end

 layouts{c}=P;save(fullfile(out,[P.caseName '.mat']),'P');

 fid=fopen(fullfile(out,[P.caseName '.json']),'w');fwrite(fid,jsonencode(P,PrettyPrint=true));fclose(fid);

end

 function rows(xs,width,ys)

  for x=xs,for y=1:size(ys,1),rack([x,x+width,ys(y,:)]);end;end

 end

 function rack(o),add(o,'rack');end

 function add(o,kind),P.obstacles(end+1,:)=o;P.obstacleKinds{end+1}=kind;end

 function zone(o,name),P.zones(end+1,:)=o;P.zoneNames{end+1}=name;end

end

