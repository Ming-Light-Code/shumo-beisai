function solve_q3_final()
% Task 3: Stochastic CP with expected consumption - OPTIMIZED
% Optimal found: Path B, w1=6(p=1), w2=4(p=1), Z=480, M=9

MAX_DAYS = 90; MAX_LOAD = 400;
all_xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
names = {"B","E","W1","W2","W3","S1","S2"};
WY = [20, 15, 28]; WM = [4, 5, 3];

dist = zeros(7);
for i=1:7, for j=1:7
    dist(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));
end, end

% Expected consumption
MO=3.2; MH=3.2; MF=2.2; PO=1.4; PH=1.4; PF=1.2; WO=5.6; WH=4.4; WF=3.6;

fprintf("=== Task 3: Full Enumeration for Optimality ===\n\n");

% ---- Enumerate ALL feasible splits for Path A (single W3 visit) ----
fprintf("--- Path A: B->S1->W3->S2->E ---\n");
O_s1=100-12*MO; H_s1=150-12*MH; F_s1=100-12*MF;
spare=400-(O_s1+H_s1+F_s1);
tr_O=31*MO; tr_H=31*MH; tr_F=31*MF;

best_A_Z=-inf; best_A_M=-inf; best_A_w=0; best_A_p=0;
for w=0:12
    s=w/3; r=mod(w,3);
    if r==0 && s>0, p=s-1; else, p=floor(w/3); end
    if w==0, p=0; end
    work_O=w*WO+p*PO; work_H=w*WH+p*PH; work_F=w*WF+p*PF;
    bO=max(0,tr_O+work_O-O_s1); bH=max(0,tr_H+work_H-H_s1); bF=max(0,tr_F+work_F-F_s1);
    tot=bO+bH+bF; cost=bO*2+bH+bF*2;
    if tot<=spare+0.01 && cost<=750
        % Check S2->E feasibility
        O2=O_s1+bO-20*MO-work_O-11*MO;
        H2=H_s1+bH-20*MH-work_H-11*MH;
        F2=F_s1+bF-20*MF-work_F-11*MF;
        if O2>=-0.01 && H2>=-0.01 && F2>=-0.01
            trE_O=10*MO; trE_H=10*MH; trE_F=10*MF;
            bE_O=max(0,trE_O-O2); bE_H=max(0,trE_H-H2); bE_F=max(0,trE_F-F2);
            totE=bE_O+bE_H+bE_F; spareE=400-max(0,O2)-max(0,H2)-max(0,F2);
            costE=bE_O*2+bE_H+bE_F*2;
            if totE<=spareE+0.01 && costE<=750-cost
                Z=200+w*WY(3); M=750-cost-costE;
                days=12+20+11+10+w+p;
                if Z>best_A_Z||(Z==best_A_Z&&M>best_A_M)
                    best_A_Z=Z; best_A_M=M; best_A_w=w; best_A_p=p;
                end
                fprintf("  w=%d p=%d: Z=%d M=%d days=%d\n",w,p,Z,round(M),days);
            end
        end
    end
end
fprintf("  Best A: w=%d p=%d Z=%d M=%d\n\n",best_A_w,best_A_p,best_A_Z,round(best_A_M));

% ---- Enumerate ALL feasible splits for Path B (double W3 visit) ----
fprintf("--- Path B: B->S1->W3->S2->W3->S2->E ---\n");
best_B_Z=-inf; best_B_M=-inf; best_B_w1=0; best_B_w2=0; best_B_p1=0; best_B_p2=0;
for w1=0:10
    s1=w1/3; r1=mod(w1,3);
    if r1==0&&s1>0, p1=s1-1; else, p1=floor(w1/3); end
    if w1==0, p1=0; end
    work1_O=w1*WO+p1*PO; work1_H=w1*WH+p1*PH; work1_F=w1*WF+p1*PF;
    b1_O=max(0,(20+11)*MO+work1_O-O_s1); b1_H=max(0,(20+11)*MH+work1_H-H_s1); b1_F=max(0,(20+11)*MF+work1_F-F_s1);
    tot1=b1_O+b1_H+b1_F; cost1=b1_O*2+b1_H+b1_F*2;
    if tot1>spare+0.01||cost1>750, continue; end
    O2=O_s1+b1_O-20*MO-work1_O-11*MO; H2=H_s1+b1_H-20*MH-work1_H-11*MH; F2=F_s1+b1_F-20*MF-work1_F-11*MF;
    if O2<-0.01||H2<-0.01||F2<-0.01, continue; end
    
    for w2=0:10
        s2=w2/3; r2=mod(w2,3);
        if r2==0&&s2>0, p2=s2-1; else, p2=floor(w2/3); end
        if w2==0, p2=0; end
        work2_O=w2*WO+p2*PO; work2_H=w2*WH+p2*PH; work2_F=w2*WF+p2*PF;
        tr2_O=32*MO; tr2_H=32*MH; tr2_F=32*MF;
        b2_O=max(0,tr2_O+work2_O-max(0,O2)); b2_H=max(0,tr2_H+work2_H-max(0,H2)); b2_F=max(0,tr2_F+work2_F-max(0,F2));
        tot2=b2_O+b2_H+b2_F; cost2=b2_O*2+b2_H+b2_F*2;
        spare2=400-max(0,O2)-max(0,H2)-max(0,F2);
        if tot2>spare2+0.01||cost2>750-cost1, continue; end
        total_days=12+20+11+11+11+10+w1+p1+w2+p2;
        if total_days>90, continue; end
        Z=200+(w1+w2)*WY(3); M=750-cost1-cost2;
        if Z>best_B_Z||(Z==best_B_Z&&M>best_B_M)
            best_B_Z=Z; best_B_M=M; best_B_w1=w1; best_B_w2=w2; best_B_p1=p1; best_B_p2=p2;
        end
        if w1+w2>=8, fprintf("  w1=%d(p=%d) w2=%d(p=%d): Z=%d M=%d days=%d\n",w1,p1,w2,p2,Z,round(M),total_days); end
    end
end
fprintf("  Best B: w1=%d(p=%d) w2=%d(p=%d) Z=%d M=%d\n\n",best_B_w1,best_B_p1,best_B_w2,best_B_p2,best_B_Z,round(best_B_M));

% ---- Select global best ----
if best_B_Z>best_A_Z || (best_B_Z==best_A_Z && best_B_M>best_A_M)
    use_B=true; best_Z=best_B_Z; best_M=best_B_M;
    path=[1,6,5,7,5,7,2]; w1=best_B_w1; p1=best_B_p1; w2=best_B_w2; p2=best_B_p2;
else
    use_B=false; best_Z=best_A_Z; best_M=best_A_M;
    path=[1,6,5,7,2]; w1=best_A_w; p1=best_A_p; w2=0; p2=0;
end

fprintf("===== GLOBAL OPTIMAL =====\n");
if use_B
    fprintf("Path: B->S1->W3(work %d+park %d)->S2->W3(work %d+park %d)->S2->E\n",w1,p1,w2,p2);
else
    fprintf("Path: B->S1->W3(work %d+park %d)->S2->E\n",w1,p1);
end
fprintf("Z = %d  M = %d\n\n",best_Z,round(best_M));

% ---- Generate Schedule ----
export_optimal(use_B, path, w1, p1, w2, p2, dist, all_xy, names, WY, WM, MO,MH,MF,PO,PH,PF,WO,WH,WF, MAX_DAYS, MAX_LOAD);
fprintf("\nDone. result.xls exported.\n");
end

function export_optimal(use_B, path, w1, p1, w2, p2, dist, all_xy, names, WY, WM, MO,MH,MF,PO,PH,PF,WO,WH,WF, MAX_DAYS, MAX_LOAD)
    O=100; H=150; F=100; M=750; Z=200;
    day=0; excel=cell(200,13);
    fprintf("===== DAY-BY-DAY SCHEDULE =====\n");
    fprintf("Day  | Pos (x,y)   | Action          |  O    H    F   Load |    Z      M\n");
    fprintf("-----|--------------|-----------------|--------------------|-------------\n");
    
    % --- Segment 1: B->S1 (12 cells) ---
    for i=1:12
        day=day+1; O=O-MO; H=H-MH; F=F-MF;
        x=round(1+(12-1)*i/12); y=round(15+(16-15)*i/12);
        if i<=3||i>=10
            fprintf("%4d | (%2d,%2d)      | move            | %4d %4d %4d %4d | %5d %6d\n", ...
                day,x,y,round(O),round(H),round(F),round(O+H+F),Z,round(M));
            excel{day,1}=day; excel{day,2}=x; excel{day,3}=y; excel{day,4}="move";
        elseif i==4, fprintf(" ... | ...         | ...             | ...  ...  ...  ... | ...   ...\n"); end
        excel{day,5}=round(O); excel{day,6}=round(H); excel{day,7}=round(F); excel{day,8}=Z; excel{day,9}=round(M);
    end
    % S1 supply for first W3
    tr1_O=31*MO; tr1_H=31*MH; tr1_F=31*MF;
    work1_O=w1*WO+p1*PO; work1_H=w1*WH+p1*PH; work1_F=w1*WF+p1*PF;
    bO=max(0,tr1_O+work1_O-O); bH=max(0,tr1_H+work1_H-H); bF=max(0,tr1_F+work1_F-F);
    cost=bO*2+bH+bF*2; M=M-cost; O=O+bO; H=H+bH; F=F+bF;
    fprintf("%4d | (%2d,%2d)      | SUPPLY(S1)      | %4d %4d %4d %4d | %5d %6d  (+O%d H%d F%d)\n", ...
        day,12,16,round(O),round(H),round(F),round(O+H+F),Z,round(M),round(bO),round(bH),round(bF));
    excel{day,4}=sprintf("SUPPLY(S1)"); excel{day,5}=round(O); excel{day,6}=round(H); excel{day,7}=round(F); excel{day,8}=Z; excel{day,9}=round(M);
    
    % --- Segment 2: S1->W3 (20 cells) ---
    for i=1:20
        day=day+1; O=O-MO; H=H-MH; F=F-MF;
        if i<=3||i>=18
            fprintf("%4d | (%2d,%2d)      | move            | %4d %4d %4d %4d | %5d %6d\n", ...
                day,round(12+(24-12)*i/20),round(16+(24-16)*i/20),round(O),round(H),round(F),round(O+H+F),Z,round(M));
        elseif i==4, fprintf(" ... | ...         | ...             | ...  ...  ...  ... | ...   ...\n"); end
    end
    
    % --- First W3 Work ---
    [day,O,H,F,Z] = do_work(day,O,H,F,Z,w1,p1,WO,WH,WF,PO,PH,PF,WY(3),24,24,"W3",excel,M);
    
    % --- Segment: W3->S2 (11 cells) ---
    for i=1:11
        day=day+1; O=O-MO; H=H-MH; F=F-MF;
        if i>=9
            fprintf("%4d | (%2d,%2d)      | move            | %4d %4d %4d %4d | %5d %6d\n", ...
                day,round(24+(21-24)*i/11),round(24+(16-24)*i/11),round(O),round(H),round(F),round(O+H+F),Z,round(M));
        elseif i==1, fprintf(" ... | ...         | ...             | ...  ...  ...  ... | ...   ...\n"); end
    end
    
    if use_B
        % --- S2 supply for second W3 ---
        tr2_O=32*MO; tr2_H=32*MH; tr2_F=32*MF;
        work2_O=w2*WO+p2*PO; work2_H=w2*WH+p2*PH; work2_F=w2*WF+p2*PF;
        bO2=max(0,tr2_O+work2_O-O); bH2=max(0,tr2_H+work2_H-H); bF2=max(0,tr2_F+work2_F-F);
        cost2=bO2*2+bH2+bF2*2; M=M-cost2; O=O+bO2; H=H+bH2; F=F+bF2;
        fprintf("%4d | (%2d,%2d)      | SUPPLY(S2)  (2nd)| %4d %4d %4d %4d | %5d %6d  (+O%d H%d F%d)\n", ...
            day,21,16,round(O),round(H),round(F),round(O+H+F),Z,round(M),round(bO2),round(bH2),round(bF2));
        excel{day,4}=sprintf("SUPPLY(S2)-2nd");
        
        % --- S2->W3 (11 cells) ---
        for i=1:11
            day=day+1; O=O-MO; H=H-MH; F=F-MF;
            if i<=3||i>=9
                fprintf("%4d | (%2d,%2d)      | move            | %4d %4d %4d %4d | %5d %6d\n", ...
                    day,round(21+(24-21)*i/11),round(16+(24-16)*i/11),round(O),round(H),round(F),round(O+H+F),Z,round(M));
            elseif i==4, fprintf(" ... | ...         | ...             | ...  ...  ...  ... | ...   ...\n"); end
        end
        
        % --- Second W3 Work ---
        [day,O,H,F,Z] = do_work(day,O,H,F,Z,w2,p2,WO,WH,WF,PO,PH,PF,WY(3),24,24,"W3",excel,M);
        
        % --- W3->S2 (11 cells) ---
        for i=1:11
            day=day+1; O=O-MO; H=H-MH; F=F-MF;
            if i>=9
                fprintf("%4d | (%2d,%2d)      | move            | %4d %4d %4d %4d | %5d %6d\n", ...
                    day,round(24+(21-24)*i/11),round(24+(16-24)*i/11),round(O),round(H),round(F),round(O+H+F),Z,round(M));
            elseif i==1, fprintf(" ... | ...         | ...             | ...  ...  ...  ... | ...   ...\n"); end
        end
    end
    
    % --- S2 supply for S2->E ---
    trE_O=10*MO; trE_H=10*MH; trE_F=10*MF;
    bOE=max(0,trE_O-O); bHE=max(0,trE_H-H); bFE=max(0,trE_F-F);
    costE=bOE*2+bHE+bFE*2; M=M-costE; O=O+bOE; H=H+bHE; F=F+bFE;
    fprintf("%4d | (%2d,%2d)      | SUPPLY(S2)      | %4d %4d %4d %4d | %5d %6d  (+O%d H%d F%d)\n", ...
        day,21,16,round(O),round(H),round(F),round(O+H+F),Z,round(M),round(bOE),round(bHE),round(bFE));
    excel{day,4}=sprintf("SUPPLY(S2)"); excel{day,5}=round(O); excel{day,6}=round(H); excel{day,7}=round(F); excel{day,8}=Z; excel{day,9}=round(M);
    
    % --- S2->E (10 cells) ---
    for i=1:10
        day=day+1; O=O-MO; H=H-MH; F=F-MF;
        fprintf("%4d | (%2d,%2d)      | move            | %4d %4d %4d %4d | %5d %6d\n", ...
            day,round(21+(30-21)*i/10),round(16+(15-16)*i/10),round(O),round(H),round(F),round(O+H+F),Z,round(M));
        excel{day,1}=day; excel{day,2}=round(21+(30-21)*i/10); excel{day,3}=round(16+(15-16)*i/10); excel{day,4}="move";
        excel{day,5}=round(O); excel{day,6}=round(H); excel{day,7}=round(F); excel{day,8}=Z; excel{day,9}=round(M);
    end
    
    fprintf("-----|--------------|-----------------|--------------------|-------------\n");
    fprintf("Final at E: Z=%d M=%d Day=%d\n",Z,round(M),day);
    
    T=cell2table(excel(1:day,1:9),"VariableNames",{'Day','PosX','PosY','Action','O','H','F','Z','M'});
    writetable(T,"C:\Users\ming\Desktop\任务3_随机天气CP方案\result.xls");
    fprintf("Exported %d rows to result.xls\n",day);
end

function [day,O,H,F,Z] = do_work(day,O,H,F,Z,w,p,WO,WH,WF,PO,PH,PF,yld,px,py,name,excel,M_val)
    rem=w;
    while rem>0
        chunk=min(rem,3);
        for wk=1:chunk
            day=day+1; O=O-WO; H=H-WH; F=F-WF; Z=Z+yld;
            fprintf("%4d | (%2d,%2d)      | work(%s)        | %4d %4d %4d %4d | %5d %6d\n", ...
                day,px,py,name,round(O),round(H),round(F),round(O+H+F),Z,round(M_val));
            excel{day,1}=day; excel{day,2}=px; excel{day,3}=py; excel{day,4}=sprintf("work(%s)",name);
            excel{day,5}=round(O); excel{day,6}=round(H); excel{day,7}=round(F); excel{day,8}=Z; excel{day,9}=round(M_val);
        end
        rem=rem-chunk;
        if rem>0
            day=day+1; O=O-PO; H=H-PH; F=F-PF;
            fprintf("%4d | (%2d,%2d)      | park(reset)     | %4d %4d %4d %4d | %5d %6d\n", ...
                day,px,py,round(O),round(H),round(F),round(O+H+F),Z,round(M_val));
            excel{day,1}=day; excel{day,2}=px; excel{day,3}=py; excel{day,4}="park(reset)";
            excel{day,5}=round(O); excel{day,6}=round(H); excel{day,7}=round(F); excel{day,8}=Z; excel{day,9}=round(M_val);
        end
    end
end



