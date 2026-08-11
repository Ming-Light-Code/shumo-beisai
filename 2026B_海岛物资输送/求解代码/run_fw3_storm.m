function [Zf,Mf]=run_fw3_storm(weather)
 all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];LOAD=120;MAX_D=30;
 sk_pid=[1,6,2];sk_dist=[3,8];sk_work=[0,0];WM=[4,5,3];
 x=1;y=5;O=35;H=45;F=30;M=240;Z=100;seg=1;step=0;sd=0;fin=0;day=1;
 while day<=MAX_D&&~fin
 wx=weather{day};cO=0;cH=0;cF=0;bO=0;bH=0;bF=0;dx=0;dy=0;
 nid=0;for i=1:7,if all_xy(i,1)==x&&all_xy(i,2)==y,nid=i;break;end;end
if nid==2,fin=1;
 elseif seg<=length(sk_dist)&&nid==sk_pid(seg+1)
 if(nid==6||nid==7)&&~sd
 ro=0;rh=0;rf=0;
 for k=seg+1:length(sk_dist)
 ro=ro+sk_dist(k)*2;rh=rh+sk_dist(k)*3;rf=rf+sk_dist(k)*2;
 npid=sk_pid(k+1);if npid==6||npid==7,break;end;end
no=max(0,ceil(ro-O));nh=max(0,ceil(rh-H));nf=max(0,ceil(rf-F));sp=LOAD-(O+H+F);
 if no+nh+nf>sp
 if strcmp(wx,'normal'),cO=1;cH=1;cF=1;else,cO=3;cH=3;cF=2;end
 else bO=no;bH=nh;bF=nf;if bO*2+bH*1+bF*2<=M,sd=1;seg=seg+1;step=0;else,fin=1;end;end
 end;end
 else
 if seg<=length(sk_dist)
 tp=sk_pid(seg+1);tx=all_xy(tp,1);ty=all_xy(tp,2);
 if x<tx,dx=1;elseif x>tx,dx=-1;elseif y<ty,dy=1;elseif y>ty,dy=-1;end
 if strcmp(wx,'normal'),cO=2;cH=3;cF=2;else,cO=8;cH=4;cF=3;end
 step=step+1;end;end
 x=x+dx;y=y+dy;O=O-cO+bO;H=H-cH+bH;F=F-cF+bF;
 if bO+bH+bF>0,M=M-(bO*2+bH*1+bF*2);end
 if O<0||H<0||F<0||M<0||O+H+F>LOAD,Zf=0;Mf=0;return;end
 day=day+1;end
 Zf=Z;Mf=M;end
