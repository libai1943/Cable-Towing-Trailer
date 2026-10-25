function WarehouseConfinedFigures()

% Publication plots use identical square physical maps and one spatial scale.

set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman',...

 'defaultAxesFontSize',8,'defaultTextFontSize',8,'defaultLegendFontSize',8,'defaultTextInterpreter','none');

out='paper/figures';figdir='results/warehouse_v6/figures';if ~isfolder(figdir),mkdir(figdir);end

colors=[0,162,232;34,177,76;163,73,164;255,127,39;237,28,36;63,72,204;0,130,170;215,160,0]/255;

WarehouseConfinedMapFigure(true);copyfile('results/warehouse_v6/figures/warehouse_cases.pdf',fullfile(out,'warehouse_cases.pdf'));

% Reproduce the unchanged mechanical certificate from its preserved G10 data.

d=load('results/warehouse_v2/mechanism/G10.mat');P=d.P;R=d.R;cert=d.cert;

f=canvas(516,279);tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

turn=find(R.K<0,1,'last')+1;pull=find(R.W>1e-6,1)+1;ids=[1,turn,pull];phases={'Taut','Reposition','Re-engage'};

for j=1:3

 ax=nexttile;hold on;poses(R,P,ids(j));

 xline(P.left,'k-');xline(P.right,'k-');axis equal;xlim([P.left-.10,P.right+.10]);ylim([-1.1,2.3]);box off;set(ax,'XColor','none','YColor','none');

 xlabel('');text(.5,-.03,sprintf('(%c) %s',96+j,phases{j}),'Units','normalized','HorizontalAlignment','center','VerticalAlignment','top');

end

ax=nexttile;hold on;sweep(R,P,30);path(R,P);poses(R,P,[1,size(R.X,2)]);xline(P.left,'k-');xline(P.right,'k-');finishMap(P);ylim([-1,9]);text(.5,-.02,'(d) Complete reorientation','Units','normalized','HorizontalAlignment','center','VerticalAlignment','top');

nexttile;dist=[0,cumsum(abs(R.S))];D=vecnorm(R.X(1:2,:)-R.X(4:5,:));stop=dist(find(D<P.L-1e-4,1,'last')+1);

hold on;patch([0,stop,stop,0],[.75,.75,1.36,1.36],[.91,.97,1],'EdgeColor','none');plot(dist,D,'Color',colors(3,:),'LineWidth',1.2);yline(P.L,'--','Color',colors(4,:));xlim([0,2.6]);ylim([.75,1.36]);ylabel('Endpoint distance (m)');xlabel('(e) Repositioning travel (m)');text(.07,1.02,'Cart waits','Color',colors(1,:));grid on;

nexttile;angles=linspace(0,cert.thetaBarrier,150);aa=P.a+P.L+P.rear(2);margin=P.extra-(aa*cos(angles)+P.rear(3)*sin(angles)-aa);

hold on;patch([0,rad2deg(angles(end)),rad2deg(angles(end)),0],[-.05,-.05,0,0],[1,.92,.92],'EdgeColor','none');plot(rad2deg(angles),margin,'Color',colors(5,:),'LineWidth',1.2);yline(0,'k:');ylim([-.05,.025]);ylabel('Rigid width margin (m)');xlabel('(f) Trailer heading (deg)');text(5,-.038,'Rigid blocked','Color',colors(5,:));grid on;savePlot(f,'mechanism_revision');

% Predeclared route details, same physical dimensions in every panel.

f=canvas(516,193);

for j=1:3

 ids=[2,5,9];c=ids(j);[P,R]=getCase(c);ax=axes(f,'Units','points','Position',[7+(j-1)*172,22,158,158]);

 map(P);sweep(R,P,32);path(R,P);poses(R,P,unique(round(linspace(1,size(R.X,2),4))));

 fn=sprintf('results/warehouse_v6/main/confined_main_W%02d_max.mat',c);

 if isfile(fn),b=load(fn);plot(b.R.selected.X(1,:),b.R.selected.X(2,:),'--','Color',colors(5,:),'LineWidth',.8);end

 finishMap(P);panelLabel(ax,sprintf('(%c) W%02d',96+j,c));

end

annotation(f,'textbox',[.08,.945,.84,.05],'String','Tractor: cyan     Cart: green     Direct-MAX tractor: red dashed     All panels: 16 m x 16 m',...

 'EdgeColor','none','HorizontalAlignment','center','FontName','Times New Roman','FontSize',8,'Margin',0);

savePlot(f,'trajectory_details');

% Repositioning evidence; consistent axes and compact bottom panel labels.

[P,R]=getCase(9);dist=[0,cumsum(abs(R.S))];f=canvas(516,171);

labels={'(a) Cable separation','(b) Transmitted motion','(c) Vehicle headings'};

for j=1:3

 ax=axes(f,'Units','points','Position',[35+(j-1)*171,44,126,117]);hold on;box off;grid on;

 if j==1,plot(dist,vecnorm(R.X(1:2,:)-R.X(4:5,:)),'Color',colors(3,:),'LineWidth',1.1);yline(P.L,'--','Color',colors(4,:));ylabel('Endpoint distance (m)');

 elseif j==2,stairs(dist(1:end-1),R.S,'Color',colors(1,:),'LineWidth',1);stairs(dist(1:end-1),R.W,'Color',colors(2,:),'LineWidth',1);yline(0,':');ylabel('Step displacement (m)');legend({'Tractor','Cart hitch'},'Box','off','Location','southeast');

 else,plot(dist,rad2deg(R.X(3,:)),'Color',colors(1,:),'LineWidth',1);plot(dist,rad2deg(R.X(6,:)),'Color',colors(2,:),'LineWidth',1);ylabel('Heading (deg)');legend({'Tractor','Cart'},'Box','off','Location','southeast');end

 xlim([0,min(2.6,dist(end))]);xlabel('Tractor travel (m)');

 annotation(f,'textbox',[(j-1)/3,0,1/3,.075],'String',labels{j},'EdgeColor','none','HorizontalAlignment','center','FontName','Times New Roman','FontSize',8,'Margin',0);

end

savePlot(f,'mode_evidence');

% Aggregate chart is computed directly from saved independently audited runs.

methods={'proposed','max','logic','complementarity','fixed','portfolio_unique','minlp','rgr'};

names={'Proposed','MAX','LOGIC','COMP','FIX','PORT','MINLP','RGR'};stats=zeros(8,3);

for j=1:8

 times=[];gains=[];success=0;

 for c=1:9

  d=getRecord(c,methods{j});s=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',c));

  times(end+1)=d.R.totalSeconds;gains(end+1)=100*(1-d.R.retainedTravel/s.w.travel);success=success+d.R.rawVerified;

 end

 stats(j,:)=[success,mean(gains),median(times)];

end

f=canvas(516,179);titles={'(a) New validated outputs','(b) Mean travel reduction (%)','(c) Median backend time (s)'};

for j=1:3

 ax=axes(f,'Units','points','Position',[57+(j-1)*170,31,108,142]);hold on;

 for k=1:8,barh(k,stats(k,j),.67,'FaceColor',colors(k,:),'EdgeColor','none');end

 set(ax,'YDir','reverse','YTick',1:8);ylim([.35,8.65]);

 if j==1,set(ax,'YTickLabel',names);xlim([0,10]);xticks([0,3,6,9]);else,set(ax,'YTickLabel',{});xlim([0,max(stats(:,j))*1.25+.01]);end

 for k=1:8

  fmt='%.1f';if j==1,fmt='%d';end;text(stats(k,j)+ax.XLim(2)*.025,k,sprintf(fmt,stats(k,j)),'VerticalAlignment','middle','FontSize',8);

 end

 grid on;ax.YGrid='off';box off;

 annotation(f,'textbox',[(j-1)/3,0,1/3,.09],'String',titles{j},'EdgeColor','none','HorizontalAlignment','center','FontName','Times New Roman','FontSize',8,'Margin',0);

end

savePlot(f,'comparison_summary');

% Measured quality versus computation, separate from nominal-allowance acceptance.

summary=jsondecode(fileread('results/warehouse_v6/summary.json'));f=canvas(516,192);

methodNames={'Proposed','MAX','MINLP'};plotColors=colors([1,5,7],:);markers={'o','s','^'};

for j=1:2

 ax=axes(f,'Units','points','Position',[42+(j-1)*258,43,199,141]);hold on;box off;grid on;

 for m=1:3

  data=summary.budgets(strcmp({summary.budgets.method},methodNames{m}));[~,order]=sort([data.budget]);data=data(order);

  if j==1,x=[data.medianSeconds];y=[data.meanTravelReduction];else,x=[data.budget];y=[data.new];end

  plot(x,y,['-' markers{m}],'Color',plotColors(m,:),'MarkerFaceColor',plotColors(m,:),'LineWidth',1,'MarkerSize',4);

 end

 if j==1,xlabel('Measured median time (s)');ylabel('Mean travel reduction (%)');

 else,xlabel('Nominal allowance (s)');ylabel('New validated outputs');ylim([-.15,3.45]);yticks(0:3);xticks([15,30,60]);end

 legend(methodNames,'Location','best','Box','off');

 labels={'(a) Quality and measured time','(b) New-output acceptance'};

 annotation(f,'textbox',[(j-1)/2,0,.5,.08],'String',labels{j},'EdgeColor','none','HorizontalAlignment','center','FontName','Times New Roman','FontSize',8,'Margin',0);

end

savePlot(f,'budget_sensitivity');

% Five cart polygons on exactly the unchanged W02 arena, arranged as 3 by 2.

f=canvas(516,365);

for j=1:5

 cfg=jsondecode(fileread('research/warehouse_v6/final_method.json'));d=load(sprintf('results/warehouse_v6/shapes/confined_shapes_S%02d_%s.mat',j,cfg.method));P=d.P;R=d.R.selected;

 ax=axes(f,'Units','points','Position',[7+mod(j-1,3)*172,200-floor((j-1)/3)*178,158,158]);

 map(P);sweep(R,P,30);path(R,P);poses(R,P,[1,round(size(R.X,2)/2),size(R.X,2)]);finishMap(P);panelLabel(ax,sprintf('(%c) S%02d',96+j,j));

end

ax=axes(f,'Units','points','Position',[351,22,158,158]);hold on;axis equal;axis off;xlim([-1,5]);ylim([-1,5]);

for j=1:5

 d=load(sprintf('results/warehouse_v6/shape_seeds/S%02d.mat',j));V=d.P.vertices{2};offset=[mod(j-1,2)*2.6,3.7-floor((j-1)/2)*1.65];

 patch(V(:,1)+offset(1),V(:,2)+offset(2),.82+.18*colors(2,:),'EdgeColor',colors(2,:),'LineWidth',.8);

 text(offset(1),offset(2)-.75,sprintf('S%02d',j),'HorizontalAlignment','center');

end

plot([2.5,3.5],[.4,.4],'k-','LineWidth',1);text(3,.1,'1 m','HorizontalAlignment','center');panelLabel(ax,'(f) Cart outlines');savePlot(f,'shape_trajectories');

% Serial demonstration: compact square map and two useful maneuver close-ups.

d=load('results/warehouse_v6/train/three_trailers.mat');P=d.P;R=d.R;f=canvas(516,334);trainColors=colors([1,2,6,5],:);

ax=axes(f,'Units','points','Position',[5,22,306,306]);map(P);hold on;

for b=1:4

 if b==1,pos=R.X(1:2,:);else,ix=3*(b-1)+(1:3);pos=R.X(ix(1:2),:)-P.a*[cos(R.X(ix(3),:));sin(R.X(ix(3),:))];end

 plot(pos(1,:),pos(2,:),'Color',trainColors(b,:),'LineWidth',1);

 for k=unique(round(linspace(1,size(R.X,2),28))),V=trainBody(R.X(:,k),P,b);patch(V(:,1),V(:,2),trainColors(b,:),'FaceAlpha',.05,'EdgeColor',trainColors(b,:),'EdgeAlpha',.3,'LineWidth',.3);end

end

for k=unique(round(linspace(1,size(R.X,2),4))),trainPose(R.X(:,k),P);end

trainLabels={'Tractor','Cart 1','Cart 2','Cart 3'};

for b=1:4,xx=.65+(b-1)*3.9;plot([xx,xx+.6],[15.45,15.45],'Color',trainColors(b,:),'LineWidth',1.3);text(xx+.8,15.45,trainLabels{b},'VerticalAlignment','middle','FontSize',8);end

finishMap(P);panelLabel(ax,'(a) Three-cart transport: 16 m x 16 m');

ids=[find(abs(R.K)>.01 & R.S>0,1,'first'),size(R.X,2)];phases={'Turning','Final formation'};

for j=1:2

 ax=axes(f,'Units','points','Position',[326,190-(j-1)*168,183,136]);map(P,false);trainPose(R.X(:,ids(j)),P);

 V=[];for b=1:4,V=[V;trainBody(R.X(:,ids(j)),P,b)];end;center=(min(V)+max(V))/2;

 axis equal;xx=center(1)+[-5.4,5.4];yy=center(2)+[-4,4];xlim(xx);ylim(yy);axis off;rectangle('Position',[xx(1),yy(1),diff(xx),diff(yy)],'EdgeColor','k','LineWidth',.5);

 panelLabel(ax,sprintf('(%c) %s',97+j,phases{j}));

end

savePlot(f,'train_extension');

 function f=canvas(w,h)

  f=figure('Color','w','Units','points','Position',[40,40,w,h],'Visible','off','Renderer','painters');

 end

 function savePlot(f,name)

  set(findall(f,'Type','axes'),'FontName','Times New Roman','FontSize',8);set(findall(f,'Type','text'),'FontName','Times New Roman','FontSize',8,'Interpreter','none');

  exportgraphics(f,fullfile(out,[name '.pdf']),'ContentType','vector','BackgroundColor','white');exportgraphics(f,fullfile(out,[name '.png']),'Resolution',160);

  savefig(f,fullfile(figdir,[name '.fig']));close(f);

 end

 function [P,R]=getCase(c)

  d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',c));P=d.P;R=d.w;

  cfg=jsondecode(fileread('research/warehouse_v6/final_method.json'));fn=sprintf('results/warehouse_v6/%s/confined_%s_W%02d_%s.mat',cfg.group,cfg.group,c,cfg.method);

  if isfile(fn),z=load(fn);R=z.R.selected;end

 end

 function d=getRecord(c,method)

  if strcmp(method,'proposed')

   cfg=jsondecode(fileread('research/warehouse_v6/final_method.json'));d=load(sprintf('results/warehouse_v6/%s/confined_%s_W%02d_%s.mat',cfg.group,cfg.group,c,cfg.method));

  elseif strcmp(method,'portfolio_unique')

   d=load(sprintf('results/warehouse_v6/unique/confined_unique_W%02d_portfolio_unique.mat',c));

  else,d=load(sprintf('results/warehouse_v6/main/confined_main_W%02d_%s.mat',c,method));end

 end

 function panelLabel(ax,label)

  units=ax.Units;ax.Units='points';pos=ax.Position;ax.Units=units;

  f=ancestor(ax,'figure');fs=f.Position;

  annotation(f,'textbox',[pos(1)/fs(3),(pos(2)-15)/fs(4),pos(3)/fs(3),13/fs(4)],'String',label,'EdgeColor','none','HorizontalAlignment','center','VerticalAlignment','middle','FontName','Times New Roman','FontSize',8,'Margin',0,'Interpreter','none');

 end

 function map(P,frame)

  if nargin<2,frame=true;end

  hold on;

for obstacleIndex=1:size(P.obstacles,1)

 o=P.obstacles(obstacleIndex,:);kind=P.obstacleKinds{obstacleIndex};

 if strcmp(kind,'rack'),edge=[1,.5,.15];fill=[1,.92,.81];elseif strcmp(kind,'pallet'),edge=[.70,.53,0];fill=[1,.96,.68];else,edge=[.39,.31,.72];fill=[.88,.86,1];end

 rectangle('Position',[o(1),o(3),o(2)-o(1),o(4)-o(3)],'EdgeColor',edge,'FaceColor',fill,'LineWidth',.55);

 if strcmp(kind,'rack')

  if o(4)-o(3)>o(2)-o(1)

   plot(mean(o(1:2))*[1,1],o(3:4),'Color',edge,'LineWidth',.25);

   for y=o(3)+1.2:1.2:o(4)-.1,plot(o(1:2),[y,y],'Color',edge,'LineWidth',.25);end

  else

   for x=o(1)+1.2:1.2:o(2)-.1,plot([x,x],o(3:4),'Color',edge,'LineWidth',.25);end

  end

 else,plot(o(1:2),o(3:4),'Color',edge,'LineWidth',.35);

 end

end



 if frame,rectangle('Position',[P.left,P.ymin,P.right-P.left,P.ymax-P.ymin],'EdgeColor','k','LineWidth',.5);end

 end

 function finishMap(P)

  axis equal;xlim([P.left,P.right]);ylim([P.ymin,P.ymax]);axis off;

 end

 function scaleBar(P)

  len=min(2,(P.right-P.left)/3);x=P.left+.15;y=P.ymin+.20;plot([x,x+len],[y,y],'k-','LineWidth',1.3);text(x+len/2,y+.15,sprintf('%.1f m',len),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',8);

 end

 function sweep(R,P,count)

  for k=unique(round(linspace(1,size(R.X,2),count)))

   for b=1:2,V=WarehouseBody(R.X(:,k),P,b);patch(V(:,1),V(:,2),colors(b,:),'FaceAlpha',.07,'EdgeColor',colors(b,:),'EdgeAlpha',.27,'LineWidth',.3);end

  end

 end

 function path(R,P)

  q=R.X(4:5,:)-P.a*[cos(R.X(6,:));sin(R.X(6,:))];plot(R.X(1,:),R.X(2,:),'-','Color',colors(1,:),'LineWidth',1.1);plot(q(1,:),q(2,:),'-','Color',colors(2,:),'LineWidth',1.1);

 end

 function poses(R,P,ids)

  hold on;

  for k=ids

   x=R.X(:,k);

   for b=1:2

    V=WarehouseBody(x,P,b);patch(V(:,1),V(:,2),.78+.22*colors(b,:),'EdgeColor',colors(b,:),'LineWidth',.7);

    if b==1,pos=x(1:2);th=x(3);axles=[.06,P.Lf];else,pos=x(4:5)-P.a*[cos(x(6));sin(x(6))];th=x(6);axles=0;end

    steer=0;if b==1,steer=atan(P.Lf*R.K(min(k,numel(R.K))));end

    wheel(pos,th,axles,P.vertices{b},steer);e=[cos(th);sin(th)];plot([pos(1),pos(1)+.35*e(1)],[pos(2),pos(2)+.35*e(2)],'Color',colors(b,:),'LineWidth',1.2);plot(pos(1),pos(2),'.','Color',colors(b,:),'MarkerSize',4);

   end

   C=WarehouseSelectCableRoute(x,P);

   plot(C(:,1),C(:,2),'Color',colors(3,:),'LineWidth',.8);

  end

 end

 function wheel(pos,th,axles,V,steer)

  if nargin<5,steer=0;end

  rot=[cos(th),-sin(th);sin(th),cos(th)];ww=.82*min(abs(min(V(:,2))),abs(max(V(:,2))));

  for aa=axles,angle=0;if aa>.3,angle=steer;end;for sign=[-1,1],p=([aa,ww*sign]+[-.06;.06]*[cos(angle),sin(angle)])*rot'+pos';plot(p(:,1),p(:,2),'k-','LineWidth',1.1);end;end

 end

 function V=trainBody(x,P,b)

  if b==1,pos=x(1:2);th=x(3);else,ix=3*(b-1)+(1:3);th=x(ix(3));pos=x(ix(1:2))-P.a*[cos(th);sin(th)];end

  rot=[cos(th),-sin(th);sin(th),cos(th)];V=P.vertices{b}*rot'+pos';

 end

 function trainPose(x,P)

  anchor=x(1:2);

  for b=1:4

   V=trainBody(x,P,b);patch(V(:,1),V(:,2),.78+.22*trainColors(b,:),'EdgeColor',trainColors(b,:),'LineWidth',.7);

   if b==1,pos=x(1:2);th=x(3);axles=[.06,P.Lf];else,ix=3*(b-1)+(1:3);th=x(ix(3));pos=x(ix(1:2))-P.a*[cos(th);sin(th)];axles=0;C=WarehouseCable(anchor,x(ix(1:2)),P.L);plot(C(:,1),C(:,2),'Color',colors(3,:),'LineWidth',.7);anchor=pos-P.b(b)*[cos(th);sin(th)];end

   wheel(pos,th,axles,P.vertices{b});

  end

 end

end

