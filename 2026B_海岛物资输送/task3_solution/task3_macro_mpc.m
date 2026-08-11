function result = task3_macro_mpc(weather,randomSeed)
%TASK3_MACRO_MPC 任务3：单船宏观网络滚动期MILP
% 从 task4_cooperative_macro_mpc 退化而来——去掉合作、第二条船和资源交换，
% 保留单船 MILP 滚动时域 MPC + 蒙特卡洛可靠性验证框架。
%
% 用法：
%   result = task3_macro_mpc();
%   result = task3_macro_mpc(weather,2026);
%
% 方法：
% 1. 用功能点宏观时间扩展网络代替900个逐格节点；宏观移动弧的持续时间
%    等于曼哈顿距离，规划中按期望天气计资源消耗。
% 2. 每日只执行所选宏观移动弧的第一格，次日观察共同天气后重新求解。
% 3. 两阶段求解：先最大化teamZ，再固定Z最大化teamM。
%
% 输出：task3_result.xlsx、task3_best_route.png、结构体result。
% 依赖：Optimization Toolbox / intlinprog，兼容R2026a。
T=90; pn=0.8; ps=0.2;
if nargin<2 || isempty(randomSeed), randomSeed=2026; end
if nargin<1 || isempty(weather)
    rng(randomSeed,'twister');
    weather=repmat("正常",T,1);
    weather(rand(T,1)>=pn)="雷暴";
end
weather=normalizeWeather(weather,T);

p.T=T; p.capacity=400; p.B=[1,15]; p.E=[30,15];
p.S=[12,16;21,16]; p.W=[6,21;15,9;24,24];
p.waypoints=[10,15;15,15;20,15];  % B->E axis intermediate nodes
p.reward=[20,15,28]; p.maxWork=[4,5,3]; p.price=[2,1,2];
p.normalMove=[2,3,2]; p.normalIdle=[1,1,1]; p.normalWork=[5,4,3];
p.stormMove=[8,4,3]; p.stormIdle=[3,3,2]; p.stormWork=[8,6,6];
p.pNormal=pn; p.pStorm=ps; p.arrivalProbability=0.99;
p.expectedMove=pn*p.normalMove+ps*p.stormMove;
p.expectedIdle=pn*p.normalIdle+ps*p.stormIdle;
p.expectedWork=pn*p.normalWork+ps*p.stormWork;
p.lookahead=20; p.maxSolveTime=15; p.relativeGap=0.01;
p.minTravelToEnd=sum(abs(p.B-p.E));
p.nScenarios=5;  % K weather scenarios for stochastic MILP
p.monteCarloSamples=10000; p.confidenceLevel=0.95;

fprintf('单船MILP滚动时域MPC求解中...\n');
q=simulateStrategy(weather,p);
q.monteCarlo=validateFixedPolicyMonteCarlo(q,p,p.monteCarloSamples,randomSeed+100000);

if q.feasible
    fprintf('独立蒙特卡洛：%d次，成功率=%.4f，95%% Wilson下界=%.4f。\n', ...
        q.monteCarlo.samples,q.monteCarlo.successRate,q.monteCarlo.lower95);
    reliabilityPassed=q.monteCarlo.lower95>=p.arrivalProbability;
    if ~reliabilityPass
        warning('95%% Wilson下置信界未达到99%%；当前仅选择可行方案。');
    end
end

% 输出
outputFile='task3_result.xlsx';
if isfile(outputFile), delete(outputFile); end
writetable(q.schedule,outputFile,'Sheet','DailyPlan','Range','A1');

fig=figure('Color','w'); hold on;
plot(q.route(:,1),q.route(:,2),'-o','LineWidth',1.4,'MarkerSize',3);
plot(p.B(1),p.B(2),'gp','MarkerSize',14,'MarkerFaceColor','g');
plot(p.E(1),p.E(2),'rp','MarkerSize',14,'MarkerFaceColor','r');
plot(p.S(:,1),p.S(:,2),'bd','MarkerSize',8,'MarkerFaceColor','b');
plot(p.W(:,1),p.W(:,2),'m^','MarkerSize',8,'MarkerFaceColor','m');
grid on; axis equal; xlim([0.5,30.5]); ylim([0.5,30.5]);
xlabel('x'); ylabel('y'); title('任务3 单船MILP滚动时域路径');
legend('船','B','E','补给平台','作业点','Location','bestoutside');
saveas(fig,'task3_best_route.png');

fprintf('\n最终结果：teamZ=%d，teamM=%d，到达日=%d。\n',q.teamZ,q.teamM,q.arrivalDay);
fprintf('固定策略蒙特卡洛成功率=%.4f，95%% Wilson下界=%.4f，%s。\n', ...
    q.monteCarlo.successRate,q.monteCarlo.lower95,q.windowStatus);
result=struct('result',q,'weather',weather,'params',p);
end

function out=simulateStrategy(weather,p)
init=struct('position',p.B,'Water',100,'Fuel',150,'Food',100,'M',750,'Z',200, ...
    'workPoint',0,'consecutiveDays',0,'arrived',false,'target',[NaN,NaN]);
T=p.T; route=p.B; arrivalDay=0;
rows=cell(T,18); nRow=0; feasible=true; solveTotal=0;
timedOutWindows=0; allWindowsWithinTolerance=true;
buyHistory=zeros(T,3); actionTypeHistory=zeros(T,1,'uint8');
failureReason="";
for t=1:T
    if init.arrived, break; end
    H=min(max(min(ceil((T-t+1)*0.4),30),20),T-t+1); timer=tic;
    plan=solveSingleMacroMILP(init,weather(t),H,T-t+1,p);
    solveTotal=solveTotal+toc(timer);
    if ~plan.feasible
        feasible=false;
        if isfield(plan,'reason')
            failureReason=plan.reason;
            fprintf('第%d天规划不可行（%s）。\n',t,plan.reason);
        end
        break;
    end
    if ~plan.provenWithinTolerance
        timedOutWindows=timedOutWindows+1;allWindowsWithinTolerance=false;
    end
    old=init; init=plan.next;
    if ~old.arrived && init.arrived && arrivalDay==0, arrivalDay=t; end
    route(end+1,:)=init.position;
    buyHistory(t,:)=plan.buy;
    actionTypeHistory(t)=plan.actionType;
    nRow=nRow+1;
    rows(nRow,:)={t,char(weather(t)), ...
        posText(old.position),plan.action,posText(init.position), ...
        plan.buy(1),plan.buy(2),plan.buy(3),plan.gain, ...
        init.Water,init.Fuel,init.Food,init.M,init.Z, ...
        plan.predictedTeamZ,plan.chanceLevel,plan.stage1Gap,plan.stage2Gap};
    if mod(t,10)==0 || init.arrived
        fprintf('第%d天，teamZ=%d，M=%d。\n',t,init.Z,init.M);
    end
end
if arrivalDay==0, feasible=false; end
names={'Day','Weather','Start','Action','End', ...
    'BuyWater','BuyFuel','BuyFood','Gain','Water','Fuel', ...
    'Food','M','Z','PredictedTeamZ', ...
    'PlanningChanceTarget','Stage1Gap','Stage2Gap'};
schedule=cell2table(rows(1:nRow,:),'VariableNames',names);
if allWindowsWithinTolerance
    windowStatus="所有窗口达到设定求解容差";
else
    windowStatus="含超时的当前最好可行解";
end
controls=struct('days',nRow,'buy',buyHistory(1:nRow,:), ...
    'actionType',actionTypeHistory(1:nRow,:));
out=struct('feasible',feasible,'schedule',schedule,'arrivalDay',arrivalDay, ...
    'teamZ',init.Z,'teamM',init.M,'solveSeconds',solveTotal, ...
    'timedOutWindows',timedOutWindows,'windowStatus',windowStatus, ...
    'failureReason',failureReason, ...
    'route',route,'finalShip',init,'controls',controls);
end

function plan=solveSingleMacroMILP(initial,todayWeather,H,fullRemaining,p)
% 单船宏观MILP：当前节点 → 功能点(B/S/W/E) 宏观移动弧网络。
% 保留"已承诺目标必须完成"的语义（从task4移除合作后保留）。

% Generate K weather scenarios for stochastic optimization
nScen=min(p.nScenarios,5);
weatherScen=rand(nScen,H)<p.pStorm;  % nScen x H logical, true=storm
nodes=unique([initial.position;p.B;p.S;p.W;p.E;p.waypoints], ...
    'rows','stable'); nNode=size(nodes,1); nR=3; nS=2; nW=3;
Eidx=find(ismember(nodes,p.E,'rows'),1);
Sidx=zeros(1,nS); Widx=zeros(1,nW); startIdx=0;
for s=1:nS, Sidx(s)=find(ismember(nodes,p.S(s,:),'rows'),1); end
for w=1:nW, Widx(w)=find(ismember(nodes,p.W(w,:),'rows'),1); end
startIdx=find(ismember(nodes,initial.position,'rows'),1);

% 构建宏观移动弧：允许移动到所有功能点
from=[];to=[];duration=[];
functionalOrCommitted=ismember(nodes,[p.B;p.S;p.W;p.E;p.waypoints],'rows');
for i=1:nNode
    if i==Eidx, continue; end
    for j=1:nNode
        if i==j || ~functionalOrCommitted(j), continue; end
        d=sum(abs(nodes(i,:)-nodes(j,:)));
        if d>=1 && d<=H
            from(end+1,1)=i;to(end+1,1)=j;duration(end+1,1)=d;
        end
    end
end
nArc=numel(from);

% 补给平台路径检测：宏观弧已覆盖所有功能点对，MILP可自主选择分程路径
% 滚动MPC每日重规划进一步保证方向灵活性，无需额外弧约束
% 变量索引
next=0;
xIdx=reshape(next+(1:nNode*(H+1)),nNode,H+1);next=next+nNode*(H+1);
travelIdx=reshape(next+(1:nArc*H),nArc,H);next=next+nArc*H;
stayIdx=reshape(next+(1:nNode*H),nNode,H);next=next+nNode*H;
workIdx=reshape(next+(1:nW*H),nW,H);next=next+nW*H;
buyIdx=reshape(next+(1:nR*nS*H),nR,nS,H);next=next+nR*nS*H;
stateIdx=cell(nScen,1);
for kk=1:nScen
    stateIdx{kk}=reshape(next+(1:4*(H+1)),4,H+1);next=next+4*(H+1);
end
nVar=next; lb=zeros(nVar,1);ub=inf(nVar,1);

binary=[xIdx(:);travelIdx(:);stayIdx(:);workIdx(:)];
ub(buyIdx(:))=p.capacity;
for kk=1:nScen
    ub(stateIdx{kk}(1:3,:))=p.capacity;
    ub(stateIdx{kk}(4,:))=initial.M;
end
% 固定初态（所有场景相同）
init=[initial.Water;initial.Fuel;initial.Food;initial.M];
for kk=1:nScen
    lb(stateIdx{kk}(:,1))=init;ub(stateIdx{kk}(:,1))=init;
end

ub(binary)=1;
lb(xIdx(startIdx,1))=1;ub(xIdx(startIdx,1))=1;
for i=1:nNode
    if i~=startIdx,ub(xIdx(i,1))=0;end
end
if initial.arrived
    for i=1:nNode
        if i~=Eidx,ub(xIdx(i,:))=0;end
    end
end

intcon=[binary;buyIdx(:)];

% 机会约束规划参数
% opportunity constraint parameters (fixed: per-step quantile, monotonic)
riskPieces=max(1,2*ceil(fullRemaining/max(1,H)));
stageAlpha=1-(1-p.arrivalProbability)/riskPieces;
deltaActions=[p.stormMove-p.normalMove; ...
    p.stormIdle-p.normalIdle;p.stormWork-p.normalWork];
maxDelta=max(deltaActions,[],1);
for h=1:H
    nRandom=max(0,h-1);
    if nRandom>0
        q_h=binomialQuantile(nRandom,p.pStorm,stageAlpha);
        reserve=max(0,(q_h-p.pStorm*nRandom).*maxDelta);
    else
        reserve=zeros(1,3);
    end
    for ks=1:nScen
        lb(stateIdx{ks}(1:3,h+1))=max(lb(stateIdx{ks}(1:3,h+1)),reserve(:));
    end
end

% 禁止无效宏观弧和撤离后的作业
for h=1:H
    ub(travelIdx(duration>H-h+1,h))=0;
end
for w=1:nW
    % evacuation days = 2x remaining Manhattan distance to E, min 30, max 40
    remainingDistToE=sum(abs(initial.position-p.E));
    evacuationDays=min(40,max(30,2*remainingDistToE));
    if fullRemaining<=evacuationDays
        ub(workIdx(w,:))=0;
    end
end

eqCols=cell(0,1);eqVals=cell(0,1);beq=[];nEq=0;
inCols=cell(0,1);inVals=cell(0,1);b=[];nIn=0;

% 宏观时间扩展网络流
for h=1:H
    for i=1:nNode
        outArcs=travelIdx(from==i,h);
        cols=[outArcs;stayIdx(i,h)];vals=ones(numel(cols),1);
        wi=find(Widx==i,1);
        if ~isempty(wi),cols(end+1,1)=workIdx(wi,h);vals(end+1,1)=1;end
        addEq([cols;xIdx(i,h)],[vals;-1],0);

        cols=[xIdx(i,h+1);stayIdx(i,h)];vals=[1;-1];
        if ~isempty(wi),cols(end+1,1)=workIdx(wi,h);vals(end+1,1)=-1;end
        for a=find(to==i)'
            startTime=h-duration(a)+1;
            if startTime>=1
                cols(end+1,1)=travelIdx(a,startTime);vals(end+1,1)=-1;
            end
        end
        addEq(cols,vals,0);
    end
end

% 终点吸收
for h=1:H
    ub(travelIdx(from==Eidx,h))=0;
end

% Scenario-based resource balance: per-scenario weather consumption
for kScen=1:nScen
for h=1:H
    % Supply constraints (shared buy decisions, per-scenario state)
    for s=1:nS
        for r=1:3
            addLe([buyIdx(r,s,h);xIdx(Sidx(s),h)],[1;-p.capacity],0);
        end
    end
    allBuy=reshape(buyIdx(:,:,h),[],1);
    capCols=[stateIdx{kScen}(1:3,h);allBuy];
    capVals=[ones(3,1);ones(3*nS,1)];
    addLe(capCols,capVals,p.capacity);
    moneyCost=repmat(p.price(:),nS,1);
    addLe([allBuy;stateIdx{kScen}(4,h)],[moneyCost;-1],0);
    for r=1:3
        purchases=reshape(buyIdx(r,:,h),[],1);
        cols=[stateIdx{kScen}(r,h+1);stateIdx{kScen}(r,h);purchases];
        vals=[1;-1;-ones(nS,1)];
        for a=1:nArc
            d=duration(a);
            isStorm=weatherScen(kScen,h);
            if h==1
                cost=weatherUse(todayWeather,p,'move',r)+(d-1)*(isStorm*p.stormMove(r)+(1-isStorm)*p.normalMove(r));
            else
                cost=d*(isStorm*p.stormMove(r)+(1-isStorm)*p.normalMove(r));
            end
            cols(end+1,1)=travelIdx(a,h);vals(end+1,1)=cost;
        end
        if h==1
            idleCost=weatherUse(todayWeather,p,'idle',r);
            workCost=weatherUse(todayWeather,p,'work',r);
        else
            isStorm=weatherScen(kScen,h);
            idleCost=isStorm*p.stormIdle(r)+(1-isStorm)*p.normalIdle(r);
            workCost=isStorm*p.stormWork(r)+(1-isStorm)*p.normalWork(r);
        end
        nonE=setdiff(1:nNode,Eidx);
        cols=[cols;stayIdx(nonE,h);workIdx(:,h)];
        vals=[vals;idleCost*ones(numel(nonE),1);workCost*ones(nW,1)];
        addEq(cols,vals,0);
    end
    addEq([stateIdx{kScen}(4,h+1);stateIdx{kScen}(4,h);allBuy], ...
        [1;-1;moneyCost],0);
end  % for h=1:H
end  % for kScen=1:nScen
% 连续作业限制及历史衔接
for w=1:nW
    L=p.maxWork(w);
    for h0=1:H-L,addLe(workIdx(w,h0:h0+L)',ones(L+1,1),L);end
end
if initial.workPoint>=1&&initial.consecutiveDays>0
    w=initial.workPoint;allowed=p.maxWork(w)-initial.consecutiveDays;
    window=min(H,allowed+1);
    if window==allowed+1,addLe(workIdx(w,1:window)',ones(window,1),allowed);end
end

% 窗口终端：最后窗口必须到E（每场景独立验证）
for kScen=1:nScen
if H==fullRemaining
    addEq(xIdx(Eidx,H+1),1,1);
else
    stepsLeft=fullRemaining-H;
    for i=1:nNode
        if sum(abs(nodes(i,:)-p.E))>stepsLeft,ub(xIdx(i,H+1))=0;end
    end
    safeDist=sum(abs(nodes-p.E),2);
    for r=1:3
        chanceNeed=zeros(nNode,1);
        for i=1:nNode
            chanceNeed(i)=chanceMoveRequirement(safeDist(i),r,p,stageAlpha);
        end
        addLe([xIdx(:,H+1);stateIdx{kScen}(r,H+1)], ...
            [chanceNeed;-1],0);
    end
end
end  % for kScen terminal
eqLen=cellfun(@numel,eqCols);eqRows=repelem((1:nEq)',eqLen);
Aeq=sparse(eqRows,vertcat(eqCols{:}),vertcat(eqVals{:}),nEq,nVar);
inLen=cellfun(@numel,inCols);inRows=repelem((1:nIn)',inLen);
A=sparse(inRows,vertcat(inCols{:}),vertcat(inVals{:}),nIn,nVar);

rewardCoef=zeros(nVar,1);
for w=1:nW,rewardCoef(workIdx(w,:))=p.reward(w);end
options=optimoptions('intlinprog','Display','off','RelativeGapTolerance', ...
    p.relativeGap,'AbsoluteGapTolerance',0,'MaxTime',p.maxSolveTime);

% 第一阶段：只最大化teamZ
% Terminal Z potential: estimate remaining work value beyond lookahead window
% For each terminal node, compute extra days after reaching E
% Discount factor 0.08 reflects uncertainty in future weather and resources
terminalDist=sum(abs(nodes-p.E),2);
remainingAfterWindow=fullRemaining-H;
terminalZPotential=max(0,remainingAfterWindow-terminalDist)*max(p.reward)*0.08;
fZ=-rewardCoef;
fZ(xIdx(:,H+1))=fZ(xIdx(:,H+1))-terminalZPotential;
[xZ,~,flagZ,outZ]=intlinprog(fZ,intcon,A,b,Aeq,beq,lb,ub,options);
if isempty(xZ)||flagZ<0
    if flagZ==0
        reason=sprintf('第一阶段达到%.1f秒上限且未找到整数可行解',p.maxSolveTime);
    else
        reason=sprintf('第一阶段求解失败或模型不可行（exitflag=%d）',flagZ);
    end
    plan=struct('feasible',false,'reason',string(reason));return;
end
zStar=round(rewardCoef'*xZ);

% 第二阶段：固定第一阶段收益，再最大化teamM
zRow=sparse(1,nVar);zRow(1,find(rewardCoef))=rewardCoef(rewardCoef~=0);
Aeq2=[Aeq;zRow];beq2=[beq;zStar];
fM=zeros(nVar,1);
for kScen=1:nScen
    fM(stateIdx{kScen}(4,H+1))=fM(stateIdx{kScen}(4,H+1))-1/nScen;
end
terminalDistance=sum(abs(nodes-p.E),2);
distanceEpsilon=0.01/(1+max(terminalDistance));
stayEpsilon=0.001;
fM(xIdx(:,H+1))=fM(xIdx(:,H+1))+distanceEpsilon*terminalDistance;
nonE=setdiff(1:nNode,Eidx);
fM(stayIdx(nonE,1))=fM(stayIdx(nonE,1))+stayEpsilon;
[x,~,flagM,outM]=intlinprog(fM,intcon,A,b,Aeq2,beq2,lb,ub,xZ,options);
if isempty(x)||flagM<0
    if flagM==0
        reason=sprintf('第二阶段达到%.1f秒上限且未找到整数可行解',p.maxSolveTime);
    else
        reason=sprintf('第二阶段求解失败或模型不可行（exitflag=%d）',flagM);
    end
    plan=struct('feasible',false,'reason',string(reason));return;
end
stage1Gap=solverRelativeGap(outZ);stage2Gap=solverRelativeGap(outM);

% 验证求解器返回向量的可行性
ineqViolation=0;if ~isempty(A),ineqViolation=max([0;A*x-b]);end
eqViolation=0;if ~isempty(Aeq2),eqViolation=max(abs(Aeq2*x-beq2));end
boundViolation=max([0;lb-x;x-ub]);
integerViolation=max(abs(x(intcon)-round(x(intcon))));
solverViolation=max([ineqViolation;eqViolation;boundViolation;integerViolation]);
if solverViolation>1e-5
    plan=struct('feasible',false,'reason', ...
        sprintf("求解器返回向量违反MILP约束，最大误差=%.3g",solverViolation));return;
end

% 提取今日补给和实际行动
buy=zeros(1,3);
for r=1:3,buy(r)=round(sum(x(buyIdx(r,:,1))));end

nextState=initial;gain=0;actionType=uint8(0);
% 先执行补给
vec=[nextState.Water,nextState.Fuel,nextState.Food,nextState.M];
vec(1:3)=vec(1:3)+buy;vec(4)=vec(4)-sum(buy.*p.price);
if nextState.arrived
    action="已退出";
    nextState.Water=round(vec(1));nextState.Fuel=round(vec(2));
    nextState.Food=round(vec(3));nextState.M=round(vec(4));
else
    chosen=find(x(travelIdx(:,1))>0.5,1);wi=find(x(workIdx(:,1))>0.5,1);
    if ~isempty(chosen)
        target=nodes(to(chosen),:);newPos=oneStep(initial.position,target);
        action=sprintf('向(%d,%d)移动至(%d,%d)',target(1),target(2),newPos(1),newPos(2));
        actionType=uint8(1);
        use=[weatherUse(todayWeather,p,'move',1),weatherUse(todayWeather,p,'move',2), ...
            weatherUse(todayWeather,p,'move',3)];
        nextState.position=newPos;nextState.target=target;
        nextState.workPoint=0;nextState.consecutiveDays=0;
        if isequal(newPos,target),nextState.target=[NaN,NaN];end
    elseif ~isempty(wi)
        action=sprintf('在W%d作业',wi);gain=p.reward(wi);
        actionType=uint8(3);
        use=[weatherUse(todayWeather,p,'work',1),weatherUse(todayWeather,p,'work',2), ...
            weatherUse(todayWeather,p,'work',3)];
        if nextState.workPoint==wi,nextState.consecutiveDays=nextState.consecutiveDays+1;
        else,nextState.workPoint=wi;nextState.consecutiveDays=1;end
    else
        action="停泊";
        actionType=uint8(2);
        use=[weatherUse(todayWeather,p,'idle',1),weatherUse(todayWeather,p,'idle',2), ...
            weatherUse(todayWeather,p,'idle',3)];
        nextState.workPoint=0;nextState.consecutiveDays=0;
    end
    vec(1:3)=vec(1:3)-use;
    nextState.Water=round(vec(1));nextState.Fuel=round(vec(2));
    nextState.Food=round(vec(3));nextState.M=round(vec(4));
    nextState.Z=nextState.Z+gain;
    if isequal(nextState.position,p.E),nextState.arrived=true;end
end
if nextState.Water<0||nextState.Fuel<0||nextState.Food<0||nextState.M<0|| ...
        nextState.Water+nextState.Fuel+nextState.Food>p.capacity
    plan=struct('feasible',false,'reason',sprintf( ...
        "执行后状态违规：淡水/燃料/食物/资金=%g/%g/%g/%g，载重=%g", ...
        nextState.Water,nextState.Fuel,nextState.Food,nextState.M, ...
        nextState.Water+nextState.Fuel+nextState.Food));return;
end
provenWithinTolerance=flagZ>0 && flagM>0;
plan=struct('feasible',true,'next',nextState,'action',action,'buy',buy, ...
    'actionType',actionType,'gain',gain,'predictedTeamZ',initial.Z+round(rewardCoef'*x), ...
    'chanceLevel',p.arrivalProbability,'stage1Gap',stage1Gap,'stage2Gap',stage2Gap, ...
    'provenWithinTolerance',provenWithinTolerance, ...
    'stage1Exitflag',flagZ,'stage2Exitflag',flagM);

    function addEq(cols,vals,rhs)
        nEq=nEq+1;eqCols{nEq,1}=cols(:);eqVals{nEq,1}=vals(:);beq(nEq,1)=rhs;
    end
    function addLe(cols,vals,rhs)
        nIn=nIn+1;inCols{nIn,1}=cols(:);inVals{nIn,1}=vals(:);b(nIn,1)=rhs;
    end
end

function reserve=chanceDeviationReserve(n,pStorm,maxDelta,alpha)
% Fixed: independent binomial quantile for each n days
% reserve = (quantile - expected_storms) * (storm - normal)
if n<=0,reserve=zeros(size(maxDelta));return;end
q=binomialQuantile(n,pStorm,alpha);
reserve=max(0,(q-pStorm*n).*maxDelta);
end

function need=chanceMoveRequirement(distance,r,p,alpha)
if distance<=0,need=0;return;end
q=binomialQuantile(distance,p.pStorm,alpha);
need=distance*p.normalMove(r)+q*(p.stormMove(r)-p.normalMove(r));
end

function q=binomialQuantile(n,probability,alpha)
if n<=0,q=0;return;end
q=0;term=(1-probability)^n;cdf=term;
while cdf+1e-15<alpha && q<n
    term=term*(n-q)/(q+1)*probability/(1-probability);
    q=q+1;cdf=cdf+term;
end
end

function gap=solverRelativeGap(output)
gap=NaN;
if isstruct(output)
    if isfield(output,'relativegap'),gap=output.relativegap;
    elseif isfield(output,'absolutegap')&&isfield(output,'bestbound')
        gap=output.absolutegap/max(1,abs(output.bestbound));
    end
end
end

function mc=validateFixedPolicyMonteCarlo(simulation,p,nSamples,seed)
mc=struct('samples',nSamples,'successes',0,'successRate',NaN,'lower95',NaN, ...
    'method',"独立固定策略蒙特卡洛复演");
if ~simulation.feasible || simulation.controls.days<=0,return;end
rng(seed,'twister');
nDays=simulation.controls.days;
stormSample=rand(nSamples,nDays)>=p.pNormal;
success=false(nSamples,1);
for sample=1:nSamples
    resources=[100,150,100];money=750;ok=true;
    for t=1:nDays
        dailyBuy=simulation.controls.buy(t,:);
        cost=sum(dailyBuy.*p.price);
        if cost>money+1e-9 || sum(resources+dailyBuy)>p.capacity+1e-9
            ok=false;break;
        end
        resources=resources+dailyBuy;money=money-cost;
        isStorm=stormSample(sample,t);
        kind=simulation.controls.actionType(t);
        if kind==0,continue;
        elseif kind==1
            if isStorm,use=p.stormMove;else,use=p.normalMove;end
        elseif kind==2
            if isStorm,use=p.stormIdle;else,use=p.normalIdle;end
        else
            if isStorm,use=p.stormWork;else,use=p.normalWork;end
        end
        resources=resources-use;
        if any(resources<-1e-9),ok=false;break;end
    end
    success(sample)=ok;
end
mc.successes=sum(success);mc.successRate=mc.successes/nSamples;
alpha=1-p.confidenceLevel;z=sqrt(2)*erfcinv(alpha);
phat=mc.successRate;den=1+z^2/nSamples;
mc.lower95=max(0,(phat+z^2/(2*nSamples)-z*sqrt(phat*(1-phat)/nSamples+ ...
    z^2/(4*nSamples^2)))/den);
end

function v=weatherUse(weather,p,kind,r)
if weather=="正常",prefix='normal';else,prefix='storm';end
field=[prefix,upper(kind(1)),kind(2:end)];v=p.(field)(r);
end

function q=oneStep(p0,target)
q=p0;
if p0(1)<target(1),q(1)=q(1)+1;
elseif p0(1)>target(1),q(1)=q(1)-1;
elseif p0(2)<target(2),q(2)=q(2)+1;
elseif p0(2)>target(2),q(2)=q(2)-1;
end
end

function s=posText(p),s=sprintf('(%d,%d)',p(1),p(2));end

function weather=normalizeWeather(weather,T)
if isnumeric(weather)||islogical(weather)
    assert(numel(weather)==T,'天气序列必须包含90天。');
    raw=weather(:);weather=repmat("正常",T,1);weather(raw==1)="雷暴";return;
end
weather=string(weather(:));assert(numel(weather)==T,'天气序列必须包含90天。');
for k=1:T
    token=lower(strtrim(weather(k)));
    if any(token==["正常","正常天气","normal","n","0"])
        weather(k)="正常";
    elseif any(token==["雷暴","雷暴天气","storm","thunderstorm","r","1"])
        weather(k)="雷暴";
    else
        error('无法识别第%d天天气：%s',k,weather(k));
    end
end
end
