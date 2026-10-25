function WarehouseConfinedMapFigure(usePlanner)
if nargin<1,usePlanner=false;end
set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman',...
 'defaultAxesFontSize',8,'defaultTextFontSize',8,'defaultTextInterpreter','none');
out='results/warehouse_v6/figures';if ~isfolder(out),mkdir(out);end
f=figure('Color','w','Units','points','Position',[20,20,516,538],'Visible','off','Renderer','painters');
colors=[0,162,232;34,177,76]/255;
for c=1:9
 d=load(sprintf('results/warehouse_v6/seeds/W%02d.mat',c));P=d.P;w=d.w;
 if usePlanner
  cfg=jsondecode(fileread('research/warehouse_v6/final_method.json'));
  z=load(sprintf('results/warehouse_v6/%s/confined_%s_W%02d_%s.mat',cfg.group,cfg.group,c,cfg.method));w=z.R.selected;
 end
 x=7+mod(c-1,3)*172;y=378-floor((c-1)/3)*178;
 axes(f,'Units','points','Position',[x,y,158,158]);hold on;
 drawMap(P);
 for b=1:2
  for k=unique(round(linspace(1,size(w.X,2),28)))
   V=WarehouseBody(w.X(:,k),P,b);
   patch(V(:,1),V(:,2),colors(b,:),'FaceAlpha',.045,'EdgeColor',colors(b,:),'EdgeAlpha',.48,'LineWidth',.25);
  end
  pos=w.X(1:2,:);if b==2,pos=w.X(4:5,:)-P.a*[cos(w.X(6,:));sin(w.X(6,:))];end
  plot(pos(1,:),pos(2,:),'-','Color',colors(b,:),'LineWidth',.70);
 end
 for k=[1,size(w.X,2)]
  state=w.X(:,k);
  for b=1:2
   V=WarehouseBody(state,P,b);patch(V(:,1),V(:,2),.75+.25*colors(b,:),'EdgeColor',colors(b,:),'LineWidth',.7);
   if b==1,pos=state(1:2);th=state(3);else,pos=state(4:5)-P.a*[cos(state(6));sin(state(6))];th=state(6);end
   rot=[cos(th),-sin(th);sin(th),cos(th)];width=.82*max(abs(P.vertices{b}(:,2)));
   axles=0;if b==1,axles=[.06,P.Lf];end
   for side=[-1,1]
    for axle=axles
     v=[axle-.06,side*width;axle+.06,side*width]*rot'+pos';plot(v(:,1),v(:,2),'k-','LineWidth',1);
    end
   end
  end
  rope=WarehouseSelectCableRoute(state,P);plot(rope(:,1),rope(:,2),'Color',[.64,.29,.64],'LineWidth',.7);
 end
 for item={1,'S';size(w.X,2),'G'}'
  k=item{1};label=item{2};p=w.X(1:2,k);
  text(p(1)+.35,p(2)-.45,label,'FontSize',7,'HorizontalAlignment','left');
 end
 axis equal;axis off;xlim([0,16]);ylim([0,16]);rectangle('Position',[0,0,16,16],'EdgeColor','k','LineWidth',.45);
 annotation(f,'textbox',[x/516,(y-14)/538,158/516,10/538],'String',sprintf('(%c) W%02d: %s',96+c,c,P.name),...
  'EdgeColor','none','HorizontalAlignment','center','FontName','Times New Roman','FontSize',7.2,'Margin',0);
end
set(f,'PaperUnits','points','PaperPosition',[0,0,516,538],'PaperSize',[516,538]);
name='warehouse_seed_cases';if usePlanner,name='warehouse_cases';end
exportgraphics(f,fullfile(out,[name '.pdf']),'ContentType','vector','BackgroundColor','white');
exportgraphics(f,fullfile(out,[name '.png']),'Resolution',180,'BackgroundColor','white');
savefig(f,fullfile(out,[name '.fig']));close(f);
end
function drawMap(P)
for j=1:size(P.obstacles,1)
 o=P.obstacles(j,:);kind=P.obstacleKinds{j};
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
end
