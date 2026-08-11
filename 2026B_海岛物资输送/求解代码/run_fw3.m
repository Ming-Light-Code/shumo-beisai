function [Zf,Mf,dailyLog]=run_fw3(weather)
all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];LOAD=120;MAX_D=30;WY=[20,15,28];WM=[4,5,3];sk_pid=[1,3,7,5,7,2];sk_dist=[3,6,3,3,4];sk_work=[3,0,6,0,0];
x=1;y=5;O=35;H=45;F=30;M=240;Z=100;seg=1;step=0;consec=0;sd=0;fin=0;day=1;ln=0;
ml=200;lx=zeros(1,ml);ly=zeros(1,ml);lO=zeros(1,ml);lH=zeros(1,ml);lF=zeros(1,ml);lM=zeros(1,ml);lZ=zeros(1,ml);la=cell(1,ml);
while day<=MAX_D&&~fin
 wx=weather{day};cO=0;cH=0;cF=0;zG=0;bO=0;bH=0;bF=0;dx=0;dy=0;
nid=0;for i=1:7,if all_xy(i,1)==x&&all_xy(i,2)==y,nid=i;break;end;end
if nid==2,fin=1;
elseif seg<=length(sk_dist)&&nid==sk_pid(seg+1)
if(nid==6||nid==7)&&~sd
 ro=0;rh=0;rf=0;
for k=seg+1:length(sk_dist)
ro=ro+sk_dist(k)*2;rh=rh+sk_dist(k)*3;rf=rf+sk_dist(k)*2;
if sk_work(k)>0,wd=sk_work(k);wp=sk_pid(k+1)-2;if wp>0&&wp<=3,wc=WM(wp);else,wc=1;end
ro=ro+wd*5;rh=rh+wd*4;rf=rf+wd*3;if wd>wc,ni=ceil(wd/wc)-1;ro=ro+ni;rh=rh+ni;rf=rf+ni;end;end
npid=sk_pid(k+1);if npid==6||npid==7,break;end;end
no=max(0,ceil(ro-O));nh=max(0,ceil(rh-H));nf=max(0,ceil(rf-F));sp=LOAD-(O+H+F);
if no+nh+nf>sp
if strcmp(wx,'normal'),cO=1;cH=1;cF=1;else,cO=3;cH=3;cF=2;end
else bO=no;bH=nh;bF=nf;
if bO*2+bH*1+bF*2<=M,sd=1;seg=seg+1;step=0;consec=0;else,fin=1;end;end
elseif nid>=3&&nid<=5&&sk_work(seg)>0
wt=nid-2;
if strcmp(wx,'normal')
if consec<WM(wt),cO=5;cH=4;cF=3;zG=WY(wt);consec=consec+1;sk_work(seg)=sk_work(seg)-1;
else,cO=1;cH=1;cF=1;consec=0;end
else,cO=3;cH=3;cF=2;end
if sk_work(seg)<=0,seg=seg+1;step=0;consec=0;sd=0;end
end
else
 if seg<=length(sk_dist)
tp=sk_pid(seg+1);tx=all_xy(tp,1);ty=all_xy(tp,2);
if x<tx,dx=1;elseif x>tx,dx=-1;elseif y<ty,dy=1;elseif y>ty,dy=-1;end
if strcmp(wx,'normal'),cO=2;cH=3;cF=2;else,cO=8;cH=4;cF=3;end
step=step+1;consec=0;end;end
x=x+dx;y=y+dy;O=O-cO+bO;H=H-cH+bH;F=F-cF+bF;
if bO+bH+bF>0,M=M-(bO*2+bH*1+bF*2);end;Z=Z+zG;
if O<0||H<0||F<0||M<0||O+H+F>LOAD,Zf=0;Mf=0;dailyLog=[];return;end
ln=ln+1;lx(ln)=x;ly(ln)=y;lO(ln)=O;lH(ln)=H;lF(ln)=F;lM(ln)=M;lZ(ln)=Z;day=day+1;end
Zf=Z;Mf=M;n=ln;dailyLog=struct('day',num2cell(1:n),'x',num2cell(lx(1:n)),'y',num2cell(ly(1:n)),'O',num2cell(lO(1:n)),'H',num2cell(lH(1:n)),'F',num2cell(lF(1:n)),'M',num2cell(lM(1:n)),'Z',num2cell(lZ(1:n)));end
