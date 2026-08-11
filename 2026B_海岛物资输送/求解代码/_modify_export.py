
p = r"C:\Users\ming\Desktop\CUMCMThesis-master\code\export_task3_results.m"
with open(p, "r", encoding="utf-8") as f:
    content = f.read()

# ============================================================
# 2. Modify phase4_daily to return daily_log
# ============================================================
# Find phase4_daily function
old_p4_sig = "function [S,Z,M]=phase4_daily(sk,wx,all_xy,CMn,CWn,CSn,CMs,CWs,CSs,WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,IO,IH,IF,IM,IZ)"
new_p4_sig = "function [S,Z,M,daily_log]=phase4_daily(sk,wx,all_xy,CMn,CWn,CSn,CMs,CWs,CSs,WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,IO,IH,IF,IM,IZ)"

# Add daily_log init after struct creation
old_struct_init = 'S=struct("pos",all_xy(1,:),"O",IO,"H",IH,"F",IF,"M",IM,"Z",IZ,"cw",0,"day",0);'
new_struct_init = 'S=struct("pos",all_xy(1,:),"O",IO,"H",IH,"F",IF,"M",IM,"Z",IZ,"cw",0,"day",0);\ndaily_log={}; log_idx=0;'

# Wrap step_q3 calls with pre/post recording
# In phase4_daily, each [S,~,ok]=step_q3(...) needs pre-state capture + post-state log
# We add preX, preY, ... before each call and log_row after

# Simplest approach: replace the entire phase4_daily function body
p4_start = content.find(old_p4_sig)
if p4_start < 0:
    print("ERROR: phase4_daily not found")
else:
    # Find the closing "end" of phase4_daily
    # It ends with "Z=S.Z; M=S.M;\nend" before stoch_decision or before next function
    p4_body_start = content.find("\n", p4_start) + 1
    # Find end: look for pattern "Z=S.Z; M=S.M;\nend\n"
    end_marker = "Z=S.Z; M=S.M;\nend"
    p4_body_end = content.find(end_marker, p4_body_start)
    if p4_body_end < 0:
        # Try variant
        end_marker = "Z=S.Z; M=S.M;\nend"
        p4_body_end = content.find(end_marker, p4_body_start)
    p4_end = p4_body_end + len(end_marker)
    
    # Extract body between function line and Z=S.Z; M=S.M;
    # Apply replacements
    pre_body = content[p4_start:p4_body_end + len("Z=S.Z; M=S.M;")]
    post_end = content[p4_end:]
    
    # Build new phase4_daily
    new_p4 = '''function [S,Z,M,daily_log]=phase4_daily(sk,wx,all_xy,CMn,CWn,CSn,CMs,CWs,CSs,WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,IO,IH,IF,IM,IZ)
S=struct("pos",all_xy(1,:),"O",IO,"H",IH,"F",IF,"M",IM,"Z",IZ,"cw",0,"day",0);
daily_log={}; log_idx=0;
pid=sk.pid; tv=sk.tv; wi=sk.wi; wc=sk.wc; si=sk.si;
nw=sk.nw; ns=sk.ns; w1=sk.w1; b=sk.b; w2=sk.w2; di=0;
for k=1:length(pid)-1; fi=pid(k); ti=pid(k+1);
  if fi==8; ps=S.pos; else ps=all_xy(fi,:); end; pe=all_xy(ti,:);
  if ~isequal(ps,pe); cur=ps;
    while ~isequal(cur,pe) && S.day<MD
      di=di+1; w=wx(min(di,length(wx))); nxt=cur;
      if nxt(1)~=pe(1); nxt(1)=nxt(1)+sign(pe(1)-nxt(1)); else nxt(2)=nxt(2)+sign(pe(2)-nxt(2)); end
      preX=S.pos(1); preY=S.pos(2); preO=S.O; preH=S.H; preF=S.F; preM=S.M; preZ=S.Z;
      [S,~,ok]=step_q3(S,struct("type","move","t",nxt,"buyO",0,"buyH",0,"buyF",0,"cost",0),w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
      log_idx=log_idx+1; daily_log=log_row(daily_log,log_idx,S.day,w,preX,preY,preO,preH,preF,preM,preZ,"Mv",false,S.pos,0,0,0,0,S,ok);
      if ~ok; Z=S.Z; M=S.M; return; end; cur=nxt; end; end
  wk_idx=0; for j=1:nw; if wi(k)==j; wk_idx=j; break; end; end
  sup_idx=0; for j=1:ns; if si(k)==j; sup_idx=j; break; end; end
  if wk_idx>0&&wk_idx<=length(w1)
    for ww=1:w1(wk_idx); if S.day>=MD; break; end;
      r=abs(S.pos(1)-30)+abs(S.pos(2)-15);
      if S.O > CWe(1)+1.5*CMe(1)*r
        di=di+1; w=wx(min(di,length(wx)));
        preX=S.pos(1); preY=S.pos(2); preO=S.O; preH=S.H; preF=S.F; preM=S.M; preZ=S.Z;
        [S,~,ok]=step_q3(S,struct("type","work","t",pe,"buyO",0,"buyH",0,"buyF",0,"cost",0),w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
        wh=wc(wk_idx); aname=sprintf("Wk%d",wh);
        log_idx=log_idx+1; daily_log=log_row(daily_log,log_idx,S.day,w,preX,preY,preO,preH,preF,preM,preZ,aname,true,S.pos,0,0,0,0,S,ok);
        if ~ok; Z=S.Z; M=S.M; return; end; end; end
    if wk_idx<=length(b)&&b(wk_idx)>0 && S.day<MD
      di=di+1; w=wx(min(di,length(wx)));
      preX=S.pos(1); preY=S.pos(2); preO=S.O; preH=S.H; preF=S.F; preM=S.M; preZ=S.Z;
      [S,~,ok]=step_q3(S,struct("type","moor","t",pe,"buyO",0,"buyH",0,"buyF",0,"cost",0),w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
      log_idx=log_idx+1; daily_log=log_row(daily_log,log_idx,S.day,w,preX,preY,preO,preH,preF,preM,preZ,"Moor",false,S.pos,0,0,0,0,S,ok);
      if ~ok; Z=S.Z; M=S.M; return; end; end
    if wk_idx<=length(w2)
      for ww=1:w2(wk_idx); if S.day>=MD; break; end;
        r=abs(S.pos(1)-30)+abs(S.pos(2)-15);
        if S.O > CWe(1)+1.5*CMe(1)*r
          di=di+1; w=wx(min(di,length(wx)));
          preX=S.pos(1); preY=S.pos(2); preO=S.O; preH=S.H; preF=S.F; preM=S.M; preZ=S.Z;
          [S,~,ok]=step_q3(S,struct("type","work","t",pe,"buyO",0,"buyH",0,"buyF",0,"cost",0),w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
          wh=wc(wk_idx); aname=sprintf("Wk%d",wh);
          log_idx=log_idx+1; daily_log=log_row(daily_log,log_idx,S.day,w,preX,preY,preO,preH,preF,preM,preZ,aname,true,S.pos,0,0,0,0,S,ok);
          if ~ok; Z=S.Z; M=S.M; return; end; end; end; end; end
  if sup_idx>0 && S.day<MD
    rd=0; rw=0;
    for kk=k:length(tv); rd=rd+tv(kk);
      wi2=0; if kk<=length(wi); wi2=wi(kk); end
      if wi2>0&&wi2<=length(w1); rw=rw+w1(wi2)+b(wi2)+w2(wi2); end; end
    eO=rd*CMe(1)+rw*CWe(1); eH=rd*CMe(2)+rw*CWe(2); eF=rd*CMe(3)+rw*CWe(3);
    bO=floor(max(0,eO+0.5*sqrt((rd+rw)*VM(1))-S.O));
    bH=floor(max(0,eH+0.5*sqrt((rd+rw)*VW(1))-S.H));
    bF=floor(max(0,eF+0.5*sqrt((rd+rw)*VS(1))-S.F));
    free=LD-(S.O+S.H+S.F); tb=bO+bH+bF;
    if tb>free; sc=free/max(tb,1); bO=floor(bO*sc); bH=floor(bH*sc); bF=floor(bF*sc); end
    cost=bO*2+bH+bF*2;
    if cost>S.M; sc=max(0,S.M)/max(cost,1); bO=floor(bO*sc); bH=floor(bH*sc); bF=floor(bF*sc); cost=bO*2+bH+bF*2; end
    di=di+1; w=wx(min(di,length(wx)));
    preX=S.pos(1); preY=S.pos(2); preO=S.O; preH=S.H; preF=S.F; preM=S.M; preZ=S.Z;
    [S,~,ok]=step_q3(S,struct("type","supply","t",pe,"buyO",bO,"buyH",bH,"buyF",bF,"cost",cost),w,all_xy,WY,WM,CMn,CWn,CSn,CMs,CWs,CSs,LD);
    log_idx=log_idx+1; daily_log=log_row(daily_log,log_idx,S.day,w,preX,preY,preO,preH,preF,preM,preZ,"Sup",false,S.pos,bO,bH,bF,cost,S,ok);
    if ~ok; Z=S.Z; M=S.M; return; end; end
  if isequal(S.pos,all_xy(2,:)); break; end; end
Z=S.Z; M=S.M;
end'''

    content = content[:p4_start] + new_p4 + post_end
    print("Replaced phase4_daily")

# ============================================================
# 3. Update Phase 4 call in main body
# ============================================================
old_call = "wx=rand(1,MD)>0.8;[S,Z,M]=phase4_daily(best_sk,wx,all_xy,CMn,CWn,CSn,CMs,CWs,CSs,WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,IO,IH,IF,IM,IZ);"
new_call = "wx=rand(1,MD)>0.8;[S,Z,M,daily_log]=phase4_daily(best_sk,wx,all_xy,CMn,CWn,CSn,CMs,CWs,CSs,WY,WM,LD,MD,CMe,CWe,CSe,VM,VW,VS,IO,IH,IF,IM,IZ);"
content = content.replace(old_call, new_call)

# ============================================================
# 4. Add Phase 5-8 after Phase 4: SAA recording, summary, strategy, export
# ============================================================
# Find end of Phase 4 section in main
phase4_end_line = 'fprintf("  FINAL: Z=%d M=%d days=%d\\n",Z,M,S.day);'
if phase4_end_line in content:
    phase5_code = r'''
%% ====== Phase 5: Detailed SAA Recording for Best Skeleton ======
fprintf("\n--- Phase 5: Detailed SAA Recording (N=%d) ---\n", N_SAA);
saa_detail = cell(N_SAA, 14);
saa_succ = 0; saa_Zsum = 0; saa_Msum = 0;
for s = 1:N_SAA
    wx = rand(1,MD) > 0.8;
    [okS, ZS, MS, arrD, minOv, minHv, minFv, finO, finH, finF] = sim_path_q3_detailed(best_sk, wx, all_xy, names, CMn, CWn, CSn, CMs, CWs, CSs, ...
        WY, WM, LD, MD, CMe, CWe, CSe, VM, VW, VS, SAFE_Z);
    if okS
        saa_succ = saa_succ + 1;
        saa_Zsum = saa_Zsum + ZS;
        saa_Msum = saa_Msum + MS;
    end
    arrD_clamped = min(max(arrD, 1), MD);
    norm_days = sum(wx(1:arrD_clamped) == 0);
    storm_days = sum(wx(1:arrD_clamped) == 1);
    wx_str = strrep(strrep(num2str(wx), " ", ""), "  ", "");
    saa_detail(s, :) = {s, arrD, norm_days, storm_days, ZS, MS, okS, minOv, minHv, minFv, finO, finH, finF, wx_str};
end
saa_EZ = saa_Zsum / N_SAA;
saa_EM = saa_Msum / N_SAA;
saa_SR = saa_succ / N_SAA * 100;
fprintf("  E[Z]=%.0f, success=%.0f%%\n", saa_EZ, saa_SR);

%% ====== Phase 6: Compute Summary Indicators ======
summary = struct();
summary.arrival_day = S.day;
summary.final_Z = Z;
summary.final_M = M;
summary.final_O = S.O;
summary.final_H = S.H;
summary.final_F = S.F;
if ~isempty(daily_log)
    move_days = 0; moor_days = 0; work_days = 0;
    w1_days = 0; w2_days = 0; w3_days = 0;
    norm_days_total = 0; storm_days_total = 0;
    total_supply_cost = 0;
    max_load = 0; min_O = 100; min_H = 150; min_F = 100;
    fail_days = 0; all_feasible = true;
    for r = 1:size(daily_log, 1)
        act = daily_log{r, 10};
        if strcmp(act, "Mv"), move_days = move_days + 1; end
        if strcmp(act, "Moor"), moor_days = moor_days + 1; end
        if ~isempty(act) && (strcmp(act,"Wk1")||strcmp(act,"Wk2")||strcmp(act,"Wk3"))
            work_days = work_days + 1;
            if strcmp(act, "Wk1"), w1_days = w1_days + 1; end
            if strcmp(act, "Wk2"), w2_days = w2_days + 1; end
            if strcmp(act, "Wk3"), w3_days = w3_days + 1; end
        end
        if strcmp(act, "Sup"), moor_days = moor_days + 1; total_supply_cost = total_supply_cost + daily_log{r, 16}; end
        if daily_log{r, 2} == 0, norm_days_total = norm_days_total + 1; else storm_days_total = storm_days_total + 1; end
        load_val = daily_log{r, 28};
        if isnumeric(load_val) && load_val > max_load, max_load = load_val; end
        endO = daily_log{r, 24}; endH = daily_log{r, 25}; endF = daily_log{r, 26};
        if isnumeric(endO) && endO < min_O, min_O = endO; end
        if isnumeric(endH) && endH < min_H, min_H = endH; end
        if isnumeric(endF) && endF < min_F, min_F = endF; end
        if ~daily_log{r, 29}, fail_days = fail_days + 1; all_feasible = false; end
    end
    summary.move_days = move_days;
    summary.moor_days = moor_days;
    summary.work_days = work_days;
    summary.w1_days = w1_days;
    summary.w2_days = w2_days;
    summary.w3_days = w3_days;
    summary.norm_days = norm_days_total;
    summary.storm_days = storm_days_total;
    summary.supply_cost = total_supply_cost;
    summary.max_load = max_load;
    summary.min_O = min_O;
    summary.min_H = min_H;
    summary.min_F = min_F;
    summary.fail_days = fail_days;
    summary.all_feasible = all_feasible;
    summary.saa_EZ = saa_EZ;
    summary.saa_EM = saa_EM;
    summary.saa_SR = saa_SR;
end

%% ====== Phase 7: Generate Strategy Rules ======
strategy_rules = gen_strategy_rules(best_sk, all_xy, names, WY, WM, CMn, CWn, CSn, CMs, CWs, CSs, LD, MD, CMe, CWe, CSe, VM, VW, VS, SAFE_Z);

%% ====== Phase 8: Export to Excel ======
export_to_excel(output_file, summary, best_sk, daily_log, strategy_rules, saa_detail, N_SAA, names);
fprintf("\n========= RESULTS EXPORTED =========\n");
fprintf("  File: %s\n", output_file);
end
'''
    content = content.replace(phase4_end_line, phase4_end_line + phase5_code)
    print("Added Phase 5-8")

# ============================================================
# 5. Append log_row, gen_strategy_rules, export_to_excel, iif at end
# ============================================================
append_code = r'''
%% ======================================================================
%% Helper: builds one daily log row (29 columns)
%% ======================================================================
function dl=log_row(dl,idx,day,w,px,py,pO,pH,pF,pM,pZ,aname,isWork,pos,bO,bH,bF,cost,S,ok)
  dl{idx,1}=day; dl{idx,2}=w; dl{idx,3}=px; dl{idx,4}=py;
  dl{idx,5}=pO; dl{idx,6}=pH; dl{idx,7}=pF; dl{idx,8}=pM; dl{idx,9}=pZ;
  dl{idx,10}=aname; dl{idx,11}=isWork;
  if strcmp(aname,"Mv")
    dx=S.pos(1)-px; dy=S.pos(2)-py; dl{idx,12}=sprintf("(%+d,%+d)",dx,dy);
  else
    dl{idx,12}="-";
  end
  dl{idx,13}=bO; dl{idx,14}=bH; dl{idx,15}=bF; dl{idx,16}=cost;
  dl{idx,17}=pO-S.O+bO; dl{idx,18}=pH-S.H+bH; dl{idx,19}=pF-S.F+bF;
  dl{idx,20}=S.Z-pZ;
  dl{idx,21}=S.pos(1); dl{idx,22}=S.pos(2);
  dl{idx,23}=S.O; dl{idx,24}=S.H; dl{idx,25}=S.F;
  dl{idx,26}=S.M; dl{idx,27}=S.Z;
  dl{idx,28}=S.O+S.H+S.F; dl{idx,29}=ok;
end

%% ======================================================================
%% Generate Strategy Rules Table
%% ======================================================================
function rules = gen_strategy_rules(sk, all_xy, names, WY, WM, CMn, CWn, CSn, CMs, CWs, CSs, LD, MD, CMe, CWe, CSe, VM, VW, VS, Zf)
pid = sk.pid; tv = sk.tv; wi = sk.wi; wc = sk.wc; si = sk.si;
nw = sk.nw; ns = sk.ns; w1 = sk.w1; b = sk.b; w2 = sk.w2;
rules = {"??","????(x,y)","????","??????(O/H/F)","??????","??????","????/??"};
row = 2;
approx_day = 0; approx_O = 100; approx_H = 150; approx_F = 100; approx_M = 750;
for k = 1:length(pid)-1
    fi = pid(k); ti = pid(k+1);
    dist = tv(k);
    approx_day = approx_day + dist;
    approx_O = approx_O - dist * CMe(1);
    approx_H = approx_H - dist * CMe(2);
    approx_F = approx_F - dist * CMe(3);
    rule_day = approx_day;
    pos_str = sprintf("(%d,%d)", all_xy(ti,1), all_xy(ti,2));
    wk_idx = 0; for j = 1:nw, if wi(k) == j, wk_idx = j; break; end, end
    sup_idx = 0; for j = 1:ns, if si(k) == j, sup_idx = j; break; end, end
    if ti == 2
        rules{row,1}=rule_day; rules{row,2}=pos_str; rules{row,3}=approx_M;
        rules{row,4}=sprintf("O=%d,H=%d,F=%d",approx_O,approx_H,approx_F);
        rules{row,5}="????E"; rules{row,6}="????E";
        rules{row,7}="???????????"; break;
    end
    if wk_idx > 0 && wk_idx <= length(w1)
        wh = wc(wk_idx);
        normal_dec = sprintf("??W%d %d?", wh, w1(wk_idx));
        if b(wk_idx)>0, normal_dec = strcat(normal_dec, ",??1?"); end
        if w2(wk_idx)>0, normal_dec = strcat(normal_dec, sprintf(",???%d?", w2(wk_idx))); end
        storm_dec = sprintf("?O??????????");
        work_days = w1(wk_idx) + (b(wk_idx)>0) + w2(wk_idx);
        approx_O = approx_O - w1(wk_idx)*CWe(1)-w2(wk_idx)*CWe(1);
        approx_H = approx_H - w1(wk_idx)*CWe(2)-w2(wk_idx)*CWe(2);
        approx_F = approx_F - w1(wk_idx)*CWe(3)-w2(wk_idx)*CWe(3);
        if b(wk_idx)>0, approx_O=approx_O-CSe(1); approx_H=approx_H-CSe(2); approx_F=approx_F-CSe(3); end
        approx_day = approx_day + work_days;
        rules{row,1}=rule_day; rules{row,2}=pos_str; rules{row,3}=approx_M;
        rules{row,4}=sprintf("O=%d,H=%d,F=%d",approx_O,approx_H,approx_F);
        rules{row,5}=normal_dec; rules{row,6}=storm_dec;
        rules{row,7}=sprintf("??POI %s, Z+%d/?", names{ti}, WY(wh));
        row = row + 1;
    end
    if sup_idx > 0
        rem_dist=0; for kk=k+1:length(tv), rem_dist=rem_dist+tv(kk); end
        buyO=max(0,rem_dist*CMe(1)-approx_O); buyH=max(0,rem_dist*CMe(2)-approx_H); buyF=max(0,rem_dist*CMe(3)-approx_F);
        cost=buyO*2+buyH+buyF*2;
        if cost<=approx_M, approx_O=approx_O+buyO; approx_H=approx_H+buyH; approx_F=approx_F+buyF; approx_M=approx_M-cost; end
        approx_O=approx_O-CSe(1); approx_H=approx_H-CSe(2); approx_F=approx_F-CSe(3); approx_day=approx_day+1;
        rules{row,1}=rule_day; rules{row,2}=pos_str; rules{row,3}=approx_M;
        rules{row,4}=sprintf("O=%d,H=%d,F=%d",approx_O,approx_H,approx_F);
        rules{row,5}="????"; rules{row,6}="???????";
        rules{row,7}=sprintf("??? %s", names{ti});
        row = row + 1;
    end
end
end

%% ======================================================================
%% Export Results to Excel (4 sheets matching result.xls format)
%% ======================================================================
function export_to_excel(filename, summary, best_sk, daily_log, strategy_rules, saa_detail, N_SAA, names)
fprintf("  Writing sheet: ????...\n");
sh1 = cell(26, 4);
sh1{1,1} = "??3?????????????????";
sh1{1,2} = sprintf("??: %s", strjoin(names(best_sk.pid), "-"));
sh1{3,1} = "??"; sh1{3,2} = "??"; sh1{3,4} = "????";
indicators = {...
    "???", summary.arrival_day;
    "?????? Z", summary.final_Z;
    "???? M", summary.final_M;
    "???? O", summary.final_O;
    "???? H", summary.final_H;
    "???? F", summary.final_F;
    "?????", summary.move_days;
    "?????", summary.moor_days;
    "?????", summary.work_days;
    "W1 ????", summary.w1_days;
    "W2 ????", summary.w2_days;
    "W3 ????", summary.w3_days;
    "?????", summary.supply_cost;
    "??????", summary.norm_days;
    "??????", summary.storm_days;
    "??????", summary.max_load;
    "????????", sprintf("O=%d,H=%d,F=%d", summary.min_O, summary.min_H, summary.min_F);
    "??????", summary.fail_days;
    "???????", iif(summary.all_feasible, "?", "?");
    "?????? Z", round(summary.saa_EZ);
    "?????? M", round(summary.saa_EM);
    "???????", sprintf("%.0f%%%%", summary.saa_SR)};
for i = 1:size(indicators,1)
    sh1{i+4,1} = indicators{i,1};
    sh1{i+4,2} = indicators{i,2};
    sh1{i+4,4} = "???";
end
writecell(sh1, filename, "Sheet", "????", "Range", "A1");

fprintf("  Writing sheet: ??????...\n");
sh2_header = {"??","????","??x","??y","??O","??H","??F","??M","??Z",...
    "??","????","????","??O","??H","??F","????",...
    "??O","??H","??F","??Z",...
    "??x","??y","??O","??H","??F","??M","??Z","????","????"};
nrows = size(daily_log, 1);
sh2 = cell(nrows + 3, 29);
sh2{1,1} = "??3 result1??????????90??";
for j = 1:29, sh2{3, j} = sh2_header{j}; end
for r = 1:nrows
    for c = 1:29
        if c == 2
            if daily_log{r, c} == 0, sh2{r+3, c} = "??"; else sh2{r+3, c} = "??"; end
        elseif c == 11 || c == 29
            if daily_log{r, c}, sh2{r+3, c} = "?"; else sh2{r+3, c} = "?"; end
        else
            sh2{r+3, c} = daily_log{r, c};
        end
    end
end
writecell(sh2, filename, "Sheet", "??????", "Range", "A1");

fprintf("  Writing sheet: ????...\n");
nrules = size(strategy_rules, 1);
sh3 = cell(nrules + 4, 7);
sh3{1,1} = "??3?????????/????????";
sh3{2,1} = "??????????????????????????????????????";
for j = 1:7, sh3{4, j} = strategy_rules{1, j}; end
for r = 2:nrules
    for c = 1:7, sh3{r+3, c} = strategy_rules{r, c}; end
end
writecell(sh3, filename, "Sheet", "????", "Range", "A1");

fprintf("  Writing sheet: ??????...\n");
saa_header = {"????","???","????","????","??Z","??M","??????","??O","??H","??F","??O","??H","??F","????"};
n_sim = size(saa_detail, 1);
sh4 = cell(n_sim + 13, 14);
sh4{1,1} = "??3????????????";
sh4{4,1} = "????"; sh4{4,2} = "??";
sh4{5,1} = "???? Z"; sh4{5,2} = round(mean([saa_detail{:,5}]));
sh4{6,1} = "???? M"; sh4{6,2} = round(mean([saa_detail{:,6}]));
arrived = [saa_detail{:,7}];
sh4{7,1} = "?????"; sh4{7,2} = sprintf("%.0f%%%%", sum(arrived)/n_sim*100);
z_vals = [saa_detail{:,5}]; z_vals = z_vals(arrived == 1);
if ~isempty(z_vals), sh4{8,1} = "?? Z ???"; sh4{8,2} = round(std(z_vals)); end
for j = 1:14, sh4{12, j} = saa_header{j}; end
for r = 1:n_sim
    for c = 1:14
        if c == 7
            if saa_detail{r, c}, sh4{r+12, c} = "?"; else sh4{r+12, c} = "?"; end
        else
            sh4{r+12, c} = saa_detail{r, c};
        end
    end
end
writecell(sh4, filename, "Sheet", "??????", "Range", "A1");
fprintf("  All 4 sheets written successfully.\n");
end

%% Helper: inline if
function v = iif(cond, tval, fval)
if cond, v = tval; else v = fval; end
end
'''
content += append_code
print("Appended helper functions")

with open(p, "w", encoding="utf-8") as f:
    f.write(content)
print("All modifications applied")
