function decide_framework_demo()
    fprintf('===== 实时决策框架 =====\n\n');
    [Z1,M1,log1]=run_fw3(repmat({'normal'},1,35));fprintf('全正常: Z=%d M=%d %s\n',Z1,M1,iff(Z1==328,'OK','FAIL'));
    [Z2,M2,log2]=run_fw3(repmat({'storm'},1,35));fprintf('全雷暴: Z=%d M=%d %s\n\n',Z2,M2,iff(Z2==100,'OK','FAIL'));
    fprintf('--- 全正常日志 ---\n');
    for d=1:length(log1)
        fprintf('D%2d (%2d,%2d) %-30s O:%3d H:%3d F:%3d L:%3d M:%4d Z:%3d\n',log1(d).day,log1(d).x,log1(d).y,log1(d).action,log1(d).O,log1(d).H,log1(d).F,log1(d).O+log1(d).H+log1(d).F,log1(d).M,log1(d).Z);
    end
end

function s=iff(c,t,f),if c,s=t;else,s=f;end,end

function [Zf,Mf,dailyLog]=run_fw3(weather)
    all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];names={'B','E','W1','W2','W3','S1','S2'};
    LOAD=120;MAX_D=30;WY=[20,15,28];WM=[4,5,3];
    sk_pid=[1,3,7,5,7,2];sk_dist=[3,6,3,3,4];sk_work=[3,0,6,0,0];
    x=1;y=5;O=35;H=45;F=30;M=240;Z=100;
    seg=1;step=0;consec=0;supply_done=false;finished=false;day=1;log_n=0;
    maxL=200;lx=zeros(1,maxL);ly=zeros(1,maxL);lO=zeros(1,maxL);lH=zeros(1,maxL);lF=zeros(1,maxL);lM=zeros(1,maxL);lZ=zeros(1,maxL);la=cell(1,maxL);

    while day<=MAX_D && ~finished
        wx=weather{day};
        act='';cO=0;cH=0;cF=0;zG=0;bO=0;bH=0;bF=0;dx=0;dy=0;
        nid=0;for i=1:7,if all_xy(i,1)==x&&all_xy(i,2)==y,nid=i;break;end;end

        if nid==2,finished=true;act='Arrive@E!';
        elseif seg<=length(sk_dist) && nid==sk_pid(seg+1)
            if (nid==6||nid==7) && ~supply_done
                remO=0;remH=0;remF=0;
                for k=seg:length(sk_dist)
                    remO=remO+sk_dist(k)*2;remH=remH+sk_dist(k)*3;remF=remF+sk_dist(k)*2;
                    if sk_work(k)>0
                        wd=sk_work(k);wp=sk_pid(k+1)-2;
                        if wp>0&&wp<=3,wc=WM(wp);else,wc=1;end
                        remO=remO+wd*5;remH=remH+wd*4;remF=remF+wd*3;
                        if wd>wc,ni=ceil(wd/wc)-1;remO=remO+ni;remH=remH+ni;remF=remF+ni;end
                    end
                    npid=sk_pid(k+1);
                    if (npid==6||npid==7)&&npid~=nid,break;end
                end
                needO=max(0,ceil(remO-O));needH=max(0,ceil(remH-H));needF=max(0,ceil(remF-F));
                sp=LOAD-(O+H+F);
                if needO+needH+needF>sp
                    if strcmp(wx,'normal'),cO=1;cH=1;cF=1;else,cO=3;cH=3;cF=2;end
                    act=sprintf('Idle@%s(pre-buy)',names{nid});
                else
                    bO=needO;bH=needH;bF=needF;
                    if bO*2+bH*1+bF*2<=M
                        act=sprintf('Buy@%s O:%d H:%d F:%d',names{nid},bO,bH,bF);
                        supply_done=true;seg=seg+1;step=0;consec=0;
                    else,act='NoMoney!';finished=true;end
                end
            elseif nid>=3&&nid<=5 && sk_work(seg)>0
                wtype=nid-2;
                if strcmp(wx,'normal')
                    if consec<WM(wtype)
                        cO=5;cH=4;cF=3;zG=WY(wtype);consec=consec+1;sk_work(seg)=sk_work(seg)-1;
                        act=sprintf('Work@%s(%d/%d)',names{nid},consec,WM(wtype));
                    else
                        cO=1;cH=1;cF=1;consec=0;act=sprintf('Idle@%s(reset)',names{nid});
                    end
                else
                    if strcmp(wx,'normal'),cO=1;cH=1;cF=1;else,cO=3;cH=3;cF=2;end
                    act=sprintf('Idle@%s(storm)',names{nid});
                end
                if sk_work(seg)<=0,seg=seg+1;step=0;consec=0;supply_done=false;end
            end
        else
            if seg<=length(sk_dist)
                to_pid=sk_pid(seg+1);tx=all_xy(to_pid,1);ty=all_xy(to_pid,2);
                if x<tx,dx=1;elseif x>tx,dx=-1;elseif y<ty,dy=1;elseif y>ty,dy=-1;end
                if strcmp(wx,'normal'),cO=2;cH=3;cF=2;else,cO=8;cH=4;cF=3;end
                step=step+1;consec=0;act=sprintf('Move→(%d,%d)',x+dx,y+dy);
            else,act='AtEnd';end
        end

        x=x+dx;y=y+dy;O=O-cO+bO;H=H-cH+bH;F=F-cF+bF;
        if bO+bH+bF>0,M=M-(bO*2+bH*1+bF*2);end;Z=Z+zG;
        if O<0||H<0||F<0||M<0||O+H+F>LOAD,Zf=0;Mf=0;dailyLog=[];return;end
        log_n=log_n+1;lx(log_n)=x;ly(log_n)=y;lO(log_n)=O;lH(log_n)=H;lF(log_n)=F;lM(log_n)=M;lZ(log_n)=Z;la{log_n}=act;day=day+1;
    end
    Zf=Z;Mf=M;n=log_n;
    dailyLog=struct('day',num2cell(1:n),'x',num2cell(lx(1:n)),'y',num2cell(ly(1:n)),'O',num2cell(lO(1:n)),'H',num2cell(lH(1:n)),'F',num2cell(lF(1:n)),'M',num2cell(lM(1:n)),'Z',num2cell(lZ(1:n)),'action',{la{1:n}});
end
