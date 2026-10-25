"""Vector scientific schematics; all lettering uses embedded Times New Roman."""
from pathlib import Path
import math,json
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.colors import HexColor,Color,white,black
root=Path(__file__).resolve().parents[1];out=root/'paper/figures';out.mkdir(exist_ok=True)
for name,file in [('TNR','times.ttf'),('TNR-B','timesbd.ttf'),('TNR-I','timesi.ttf')]:pdfmetrics.registerFont(TTFont(name,str(Path('C:/Windows/Fonts')/file)))
palette=json.loads((root/'research/warehouse_v2/palette.json').read_text())['colors']
blue=HexColor(palette['cyan']);green=HexColor(palette['green']);orange=HexColor(palette['orange']);purple=HexColor(palette['purple']);red=HexColor(palette['red'])
def pale(c,a=.18):return Color(1-a+a*c.red,1-a+a*c.green,1-a+a*c.blue)
def poly(c,pts,fill=white,stroke=black,lw=.6):
 p=c.beginPath();p.moveTo(*pts[0])
 for x,y in pts[1:]:p.lineTo(x,y)
 p.close();c.setFillColor(fill);c.setStrokeColor(stroke);c.setLineWidth(lw);c.drawPath(p,fill=1,stroke=1)
def txt(c,x,y,s,size=8,bold=False,align='left'):
 c.setFillColor(black);c.setFont('TNR-B' if bold else 'TNR',size)
 getattr(c,{'left':'drawString','center':'drawCentredString','right':'drawRightString'}[align])(x,y,s)
def line(c,a,b,color=black,lw=.6,dash=None):
 c.setStrokeColor(color);c.setLineWidth(lw);c.setDash(dash or []);c.line(*a,*b);c.setDash([])
def arrow(c,a,b,color=black,lw=.7,head=3,both=False):
 line(c,a,b,color,lw);th=math.atan2(b[1]-a[1],b[0]-a[0]);
 for tip,t in [(b,th)]+([(a,th+math.pi)] if both else []):
  poly(c,[tip,(tip[0]-head*math.cos(t-.4),tip[1]-head*math.sin(t-.4)),(tip[0]-head*math.cos(t+.4),tip[1]-head*math.sin(t+.4))],color,color,.4)
def vehicle(c,x,y,theta,scale=30,tractor=False,shape=None,color=None):
 color=color or (blue if tractor else green)
 V=shape or ([(.6,.35),(0,.35),(0,-.35),(.6,-.35)] if tractor else [(.7,.55),(-.35,.55),(-.35,-.55),(.7,-.55)])
 def p(a,b):return x+scale*(a*math.cos(theta)-b*math.sin(theta)),y+scale*(a*math.sin(theta)+b*math.cos(theta))
 poly(c,[p(*v) for v in V],pale(color,.25),color,.8)
 # Wheel symbols are contained within the declared footprint.
 for a in ([.08,.40] if tractor else [0]):
  for sign in [-1,1]:
   w=min(abs(min(v[1] for v in V)),abs(max(v[1] for v in V)))*.88*sign
   line(c,p(a-.055,w),p(a+.055,w),black,2.4)
 if tractor:
  poly(c,[p(.27,-.16),p(.45,-.16),p(.45,.16),p(.27,.16)],pale(blue,.45),blue,.5)
  sensor=p(.2,0);c.setFillColor(white);c.setStrokeColor(blue);c.circle(*sensor,2.1,fill=1)
 else:
  poly(c,[p(.40,.27),p(-.14,.27),p(-.14,-.27),p(.40,-.27)],pale(orange,.4),orange,.45)
 arrow(c,p(.13,0),p(.53,0),color,.6,2.3)
 return p
def cable(c,a,b,sag=0,color=purple,length=None):
 if length is not None:
  d=math.dist(a,b)
  if length<d-1e-7:raise ValueError('Cable material length is shorter than endpoint distance')
  lo=0.;hi=length
  for _ in range(60):
   alpha=(lo+hi)/2
   arc=.5*math.sqrt(d*d+16*alpha*alpha)+d*d/(8*alpha)*math.asinh(4*alpha/d)
   if arc>length:hi=alpha
   else:lo=alpha
  sag=math.copysign(4*(lo+hi)/6,sag or 1) if length>d+1e-7 else 0
 c.setStrokeColor(color);c.setLineWidth(1.15);p=c.beginPath();p.moveTo(*a)
 if sag:
  dx=b[0]-a[0];dy=b[1]-a[1];d=math.hypot(dx,dy);nx=-dy/d;ny=dx/d
  p.curveTo(a[0]+dx/3+nx*sag,a[1]+dy/3+ny*sag,a[0]+2*dx/3+nx*sag,a[1]+2*dy/3+ny*sag,*b)
 else:p.lineTo(*b)
 c.drawPath(p)
def dim(c,a,b,label,offset=(0,4)):
 arrow(c,a,b,black,.45,2.2,True);txt(c,(a[0]+b[0])/2+offset[0],(a[1]+b[1])/2+offset[1],label,8,align='center')
def make(name,w,h):return canvas.Canvas(str(out/name),pagesize=(w,h),initialFontName='TNR',initialFontSize=8,pageCompression=1)

c=make('warehouse_concept.pdf',516,222)
# Aligned panels, one cart each; black leaders identify, cyan arrows indicate motion.
poly(c,[(4,18),(286,18),(286,212),(4,212)],white,HexColor('#7F7F7F'),.5)
for x,y,w,h in [(14,154,70,47),(104,154,48,47),(224,154,50,47),(14,29,70,64),(104,29,48,64),(224,29,50,64)]:
 poly(c,[(x,y),(x+w,y),(x+w,y+h),(x,y+h)],pale(orange,.18),orange,.7)
 for z in range(3):
  bx=x+4+z*(w-8)/3;poly(c,[(bx,y+7),(bx+(w-13)/3,y+7),(bx+(w-13)/3,y+h-7),(bx,y+h-7)],pale(orange,.36),orange,.4)
 txt(c,x+w/2,y+h/2-3,'Rack',8,align='center')
vehicle(c,188,140,math.pi/2,30,True)
vehicle(c,188,80,math.pi/2,30,False)
cable(c,(188,101),(188,140),-1,length=40.05)
arrow(c,(188,166),(188,199),blue,1,4);txt(c,194,184,'Travel',8)
line(c,(199,147),(218,129),black,.55);line(c,(218,129),(274,129),black,.55)
txt(c,274,119,'Autonomous tractor',8,align='right')
line(c,(173,80),(160,110),black,.55);line(c,(160,110),(108,110),black,.55)
txt(c,108,116,'Passive cart',8)
txt(c,145,4,'(a) Confined warehouse transport',8,align='center')
for y,label,th,px,sag in [(195,'Pull',0,439.05,0),(126,'Reposition; cart waits',.6,432,-15),(44,'Pull from a new heading',1.05,399+math.sqrt(40.05**2-10**2),0)]:
 qx=378;vehicle(c,qx,y,0,30,False);hx=qx+21
 vehicle(c,px,y+(10 if th else 0),th,30,True)
 cable(c,(hx,y),(px,y+(10 if th else 0)),sag,length=40.05)
 if sag:arrow(c,(456,y+21),(439,y+30),blue,.7)
 else:arrow(c,(405,y+13),(428,y+13),blue,.7)
 txt(c,412,y-(38 if sag else 26),label,8,align='center')
txt(c,412,4,'(b) Reposition, then pull',8,align='center')
c.setTitle('Autonomous cable towing in confined warehouses');c.setAuthor('');c.save()

c=make('kinematics.pdf',516,225)
# Panel a: symbolic kinematics with a nonaligned cable.
q=(47,140);th=.16;p=(203,151);ph=.53;sc=51
vehicle(c,*q,th,sc,False);vehicle(c,*p,ph,sc,True)
h=(q[0]+.7*sc*math.cos(th),q[1]+.7*sc*math.sin(th));cable(c,h,p)
for pos,label,dy in [(q,'q',-13),(h,'h',8),(p,'p',-14)]:
 c.setFillColor(black);c.circle(*pos,1.5,fill=1);txt(c,pos[0],pos[1]+dy,label,9,align='center')
arrow(c,h,(h[0]+30,h[1]+1),purple,.65);txt(c,108,154,'n',9)
arrow(c,p,(p[0]+40*math.cos(ph),p[1]+40*math.sin(ph)),blue,.65);txt(c,236,181,'v e(φ)',8)
line(c,(q[0]-7,q[1]),(q[0]+50,q[1]),HexColor('#7F7F7F'),.4,[2,2]);txt(c,77,136,'θ',9)
line(c,p,(p[0]+42,p[1]),HexColor('#7F7F7F'),.4,[2,2]);txt(c,235,157,'φ',9)
dim(c,(q[0],108),(h[0],108),'a');dim(c,(30,92),(83,92),'cart length')
dim(c,(190,122),(211,133),'l',(-3,-10));txt(c,202,115,'f',6);dim(c,(85,190),(200,190),'D ≤ L')
txt(c,132,71,'(a) Geometry and motion variables',8,align='center')
# Panel b: unobscured material-length distinction.
for y,sag,lab in [(174,0,'D = L : pulling'),(137,-21,'D < L : cart stationary')]:
 ax=318;bx=478 if sag==0 else 445
 cable(c,(ax,y),(bx,y),sag,length=160)
 for xx in [ax,bx]:c.setFillColor(black);c.circle(xx,y,1.5,fill=1)
 txt(c,398,y+11,lab,8,align='center')
 if sag:line(c,(ax,y),(bx,y),HexColor('#7F7F7F'),.45,[2,2]);txt(c,398,83,'material length remains L',8,align='center')
txt(c,402,71,'(b) Slack is not stretch',8,align='center')
# Panel c: serial coupling with visibly different cart outlines.
for x,scale,shape in [(55,21,[(.7,0),(.4,.55),(-.35,.4),(-.35,-.4),(.4,-.55)]),(180,26,[ (.7,.4),(-.5,.4),(-.5,-.4),(.7,-.4)]),(308,23,None)]:vehicle(c,x,44,0,scale,False,shape)
vehicle(c,450,44,0,29,True)
for a,b in [((69.7,44),(167,44)),((198.2,44),(300,44)),((324.1,44),(450,44))]:cable(c,a,b,-9)
txt(c,55,21,'cart 3',8,align='center');txt(c,180,21,'cart 2',8,align='center');txt(c,308,21,'cart 1',8,align='center');txt(c,464,21,'tractor',8,align='center')
txt(c,260,2,'(c) Serial transmission with individual convex footprints',8,align='center')
c.setTitle('Kinematic model and fixed-material-length cable');c.setAuthor('');c.save()
print('Two vector schematics generated using the user palette and Times New Roman.')
