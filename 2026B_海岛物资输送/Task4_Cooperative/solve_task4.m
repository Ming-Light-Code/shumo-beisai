function result = solve_task4(weather, randomSeed)

%% ========================================================================
%% solve_task4.m - Task 4: Cooperative Dual-Ship Macro MPC MILP
%% ========================================================================
%% 场景: 30x30网格, 90天, P(正常)=0.8, P(雷暴)=0.2
%% 方法: 双船联合MILP + 滚动时域 + 宏观网络 + 5种团队策略比较
%% 优化: 字典序 - 先团队Z, 固定Z后再团队M
%% 依赖: Optimization Toolbox (intlinprog)
%% 输出: task4_result.xlsx, task4_best_route.png, 结构体result
%% ========================================================================

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
p.reward=[20,15,28]; p.maxWork=[4,5,3]; p.price=[2,1,2];
p.normalMove=[2,3,2]; p.normalIdle=[1,1,1]; p.normalWork=[5,4,3];
p.stormMove=[8,4,3]; p.stormIdle=[3,3,2]; p.stormWork=[8,6,6];
    p.pNormal=pn; p.pStorm=ps; p.arrivalProbability=0.99;
    p.expectedMove=pn*p.normalMove+ps*p.stormMove;
p.expectedIdle=pn*p.normalIdle+ps*p.stormIdle;
p.expectedWork=pn*p.normalWork+ps*p.stormWork;
    p.lookahead=15; p.maxSolveTime=5; p.relativeGap=0.02;
    p.evacuationDays=40;
    p.monteCarloSamples=5000; p.confidenceLevel=0.95;

strategies=makeStrategies();
nStrategy=numel(strategies); simulations=cell(nStrategy,1);
fprintf('开始比较%d种团队策略；所有策略使用同一条天气序列。\n',nStrategy);
parfor s=1:nStrategy
    fprintf('\n========== 策略%d/%d：%s ==========\n',s,nStrategy,strategies(s).name);
    q=simulateStrategy(strategies(s),weather,p);
    q.monteCarlo=validateFixedPolicyMonteCarlo(q,p,p.monteCarloSamples,randomSeed+s*100000);
    simulations{s}=q;
    if q.feasible
        fprintf('独立蒙特卡洛：%d次，成功率=%.4f，95%% Wilson下界=%.4f。\n', ...
            q.monteCarlo.samples,q.monteCarlo.successRate,q.monteCarlo.lower95);
    end
end

% 字典序选优：先团队总Z，后团队总M；不可行方案排在最后。
comparison=table(strings(nStrategy,1),false(nStrategy,1),false(nStrategy,1), ...
    zeros(nStrategy,1),zeros(nStrategy,1),zeros(nStrategy,1),zeros(nStrategy,1), ...
    nan(nStrategy,1),nan(nStrategy,1),zeros(nStrategy,1),zeros(nStrategy,1), ...
    zeros(nStrategy,1),strings(nStrategy,1),zeros(nStrategy,1), ...
    'VariableNames',{'Strategy','Feasible','ReliabilityPassed','TeamZ','TeamM', ...
    'ArrivalDay1','ArrivalDay2','MCSuccessRate','MCWilsonLower95','TotalTransfer', ...
    'SolveSeconds','TimedOutWindows','WindowStatus','ZGainVsBaseline'});
parfor s=1:nStrategy
    q=simulations{s}; comparison.Strategy(s)=strategies(s).name;
    comparison.Feasible(s)=q.feasible; comparison.TeamZ(s)=q.teamZ;
    comparison.TeamM(s)=q.teamM; comparison.ArrivalDay1(s)=q.arrivalDay(1);
    comparison.ArrivalDay2(s)=q.arrivalDay(2);
    comparison.MCSuccessRate(s)=q.monteCarlo.successRate;
    comparison.MCWilsonLower95(s)=q.monteCarlo.lower95;
    comparison.ReliabilityPassed(s)=q.feasible && ...
        q.monteCarlo.lower95>=p.arrivalProbability;
    comparison.TotalTransfer(s)=q.totalTransfer;
    comparison.SolveSeconds(s)=q.solveSeconds;
    comparison.TimedOutWindows(s)=q.timedOutWindows;
    comparison.WindowStatus(s)=q.windowStatus;
end
if simulations{1}.feasible
    comparison.ZGainVsBaseline=comparison.TeamZ-simulations{1}.teamZ;
else
    comparison.ZGainVsBaseline(:)=NaN;
end

% 先在通过可靠性检验的方案中按(Z,M)字典序选优；若均未通过，则退化为
% 选择最好可行方案，并在输出中明确说明可靠性尚未得到统计验证。
best=selectBestStrategy(simulations,comparison.ReliabilityPassed);
selectionReliable=best>0;
if best==0
    best=selectBestStrategy(simulations,comparison.Feasible);
end
if best==0
    outputFile='task4_result.xlsx';
    if isfile(outputFile), delete(outputFile); end
    writetable(comparison,outputFile,'Sheet','StrategyComparison','Range','A1');
parfor s=1:nStrategy
        writetable(simulations{s}.schedule,outputFile,'Sheet',sprintf('Plan_%d',s),'Range','A1');
    end
    warning('本次天气实现下未能完成双船安全抵达；已保存失败日与诊断记录。');
    result=struct('comparison',comparison,'bestStrategy',[], ...
        'bestResult',[],'allResults',{simulations},'weather',weather);
    return;
end
bestResult=simulations{best};
if ~selectionReliable
    warning('没有方案的95%% Wilson下置信界达到99%%；当前仅选择团队Z最大的可行方案。');
end

outputFile='task4_result.xlsx';
if isfile(outputFile), delete(outputFile); end
writetable(comparison,outputFile,'Sheet','StrategyComparison','Range','A1');
writetable(bestResult.schedule,outputFile,'Sheet','BestDailyPlan','Range','A1');
parfor s=1:nStrategy
    sheetName=sprintf('Plan_%d',s);
    writetable(simulations{s}.schedule,outputFile,'Sheet',sheetName,'Range','A1');
end

fig=figure('Color','w'); hold on;
plot(bestResult.route1(:,1),bestResult.route1(:,2),'-o','LineWidth',1.4,'MarkerSize',3);
plot(bestResult.route2(:,1),bestResult.route2(:,2),'-s','LineWidth',1.4,'MarkerSize',3);
plot(p.B(1),p.B(2),'gp','MarkerSize',14,'MarkerFaceColor','g');
plot(p.E(1),p.E(2),'rp','MarkerSize',14,'MarkerFaceColor','r');
plot(p.S(:,1),p.S(:,2),'bd','MarkerSize',8,'MarkerFaceColor','b');
plot(p.W(:,1),p.W(:,2),'m^','MarkerSize',8,'MarkerFaceColor','m');
grid on; axis equal; xlim([0.5,30.5]); ylim([0.5,30.5]);
xlabel('x'); ylabel('y'); title("任务4选定团队策略："+strategies(best).name);
legend('船1','船2','B','E','补给平台','作业点','Location','bestoutside');
saveas(fig,'task4_best_route.png');

fprintf('\n按可靠性、团队Z和团队M排序后的选定策略：%s\n',strategies(best).name);
fprintf('团队总Z=%d，总M=%d；到达日=(%d,%d)。\n',bestResult.teamZ, ...
    bestResult.teamM,bestResult.arrivalDay(1),bestResult.arrivalDay(2));
fprintf('相对第三问不合作基准，团队总Z变化=%+d。\n', ...
    bestResult.teamZ-simulations{1}.teamZ);
fprintf('固定策略蒙特卡洛成功率=%.4f，95%% Wilson下界=%.4f；%s。\n', ...
    bestResult.monteCarlo.successRate,bestResult.monteCarlo.lower95,bestResult.windowStatus);
result=struct('comparison',comparison,'bestStrategy',strategies(best), ...
    'bestResult',bestResult,'allResults',{simulations},'weather',weather);
end

function strategies=makeStrategies()
% allowedWork(k,w)=1表示船k允许在Ww作业。
strategies(1)=struct('name',"第三问不合作基准",'allowTransfer',false, ...
    'allowedWork',true(2,3),'meetingNodes',zeros(0,2));
strategies(2)=struct('name',"自由同点协同",'allowTransfer',true, ...
    'allowedWork',true(2,3),'meetingNodes',zeros(0,2));
strategies(3)=struct('name',"船1主作业-船2支援",'allowTransfer',true, ...
    'allowedWork',[true(1,3);false(1,3)],'meetingNodes',[12,16;21,16;24,24]);
strategies(4)=struct('name',"分工作业-S2会合",'allowTransfer',true, ...
    'allowedWork',[1,1,0;0,0,1]>0,'meetingNodes',[21,16]);
strategies(5)=struct('name',"双船集中W3",'allowTransfer',true, ...
    'allowedWork',[0,0,1;0,0,1]>0,'meetingNodes',[21,16;24,24]);
end

function out=simulateStrategy(strategy,weather,p)
init=struct('position',p.B,'Water',100,'Fuel',150,'Food',100,'M',750,'Z',200, ...
    'workPoint',0,'consecutiveDays',0,'arrived',false,'target',[NaN,NaN]);
ship=[init,init]; T=p.T; route1=zeros(T+1,2); route2=zeros(T+1,2);
route1(1,:)=p.B; route2(1,:)=p.B; arrival=[0,0]; nRoute=1;
rows=cell(T,32); nRow=0; feasible=true; totalTransfer=0; solveTotal=0;
timedOutWindows=0; allWindowsWithinTolerance=true;
buyHistory=zeros(T,2,3); transfer12History=zeros(T,4);transfer21History=zeros(T,4);
actionTypeHistory=zeros(T,2,'uint8');
failureReason="";
for t=1:T
    if all([ship.arrived]), break; end
    H=min(p.lookahead,T-t+1); timer=tic;
    plan=solveJointMacroMILP(ship,weather(t),H,T-t+1,p,strategy);
    solveTotal=solveTotal+toc(timer);
    if ~plan.feasible
        feasible=false;
        if isfield(plan,'reason')
            failureReason=plan.reason;
            fprintf('%s：第%d天规划不可行（%s）。\n',strategy.name,t,plan.reason);
        end
        break;
    end
    if ~plan.provenWithinTolerance
        timedOutWindows=timedOutWindows+1;allWindowsWithinTolerance=false;
    end
    old=ship; ship=plan.next;
    for k=1:2
        if ~old(k).arrived && ship(k).arrived && arrival(k)==0, arrival(k)=t; end
    end
    nRoute=nRoute+1;
    route1(nRoute,:)=ship(1).position; route2(nRoute,:)=ship(2).position;
    totalTransfer=totalTransfer+sum(plan.transfer12)+sum(plan.transfer21);
    buyHistory(t,:,:)=reshape(plan.buy,1,2,3);
    transfer12History(t,:)=plan.transfer12;transfer21History(t,:)=plan.transfer21;
    actionTypeHistory(t,:)=plan.actionType;
    nRow=nRow+1;
    rows(nRow,:)={t,char(weather(t)),strategy.name, ...
        posText(old(1).position),plan.action(1),posText(ship(1).position), ...
        plan.buy(1,1),plan.buy(1,2),plan.buy(1,3),plan.gain(1), ...
        ship(1).Water,ship(1).Fuel,ship(1).Food,ship(1).M,ship(1).Z, ...
        posText(old(2).position),plan.action(2),posText(ship(2).position), ...
        plan.buy(2,1),plan.buy(2,2),plan.buy(2,3),plan.gain(2), ...
        ship(2).Water,ship(2).Fuel,ship(2).Food,ship(2).M,ship(2).Z, ...
        transferText(plan.transfer12,plan.transfer21),plan.predictedTeamZ, ...
        plan.chanceLevel,plan.stage1Gap,plan.stage2Gap};
    if mod(t,10)==0 || all([ship.arrived])
        fprintf('%s：第%d天，团队Z=%d，M=%d。\n',strategy.name,t, ...
            ship(1).Z+ship(2).Z,ship(1).M+ship(2).M);
    end
end
if any(arrival==0), feasible=false; end
names={'Day','Weather','Strategy','Ship1Start','Ship1Action','Ship1End', ...
    'Ship1BuyWater','Ship1BuyFuel','Ship1BuyFood','Ship1Gain','Ship1Water','Ship1Fuel', ...
    'Ship1Food','Ship1M','Ship1Z','Ship2Start','Ship2Action','Ship2End', ...
    'Ship2BuyWater','Ship2BuyFuel','Ship2BuyFood','Ship2Gain','Ship2Water','Ship2Fuel', ...
    'Ship2Food','Ship2M','Ship2Z','Transfer','PredictedTeamZ', ...
        'PlanningChanceTarget','Stage1Gap','Stage2Gap'};
route1=route1(1:nRoute,:); route2=route2(1:nRoute,:);
schedule=cell2table(rows(1:nRow,:),'VariableNames',names);
if allWindowsWithinTolerance
    windowStatus="所有窗口达到设定求解容差";
else
    windowStatus="含超时的当前最好可行解";
end
controls=struct('days',nRow,'buy',buyHistory(1:nRow,:,:), ...
    'transfer12',transfer12History(1:nRow,:),'transfer21',transfer21History(1:nRow,:), ...
    'actionType',actionTypeHistory(1:nRow,:));
out=struct('feasible',feasible,'schedule',schedule,'arrivalDay',arrival, ...
    'teamZ',ship(1).Z+ship(2).Z,'teamM',ship(1).M+ship(2).M, ...
    'totalTransfer',totalTransfer,'solveSeconds',solveTotal, ...
    'timedOutWindows',timedOutWindows,'windowStatus',windowStatus, ...
    'failureReason',failureReason, ...
    'route1',route1,'route2',route2,'finalShips',ship,'controls',controls);
end

function plan=solveJointMacroMILP(initial,todayWeather,H,fullRemaining,p,strategy)
% 动态宏观节点：两船当前位置、尚未完成的承诺目标及B/S/W/E。
% 承诺目标必须保留；若目标恰好是另一艘船昨天的位置，不能因对方移动而消失。
committedTargets=zeros(0,2);
for targetK=1:2
    if all(isfinite(initial(targetK).target))
        committedTargets(end+1,:)=initial(targetK).target; %#ok<AGROW>
    end
end
nodes=unique([initial(1).position;initial(2).position;committedTargets;p.B;p.S;p.W;p.E], ...
    'rows','stable'); nNode=size(nodes,1); nShip=2; nR=3; nS=2; nW=3;
Eidx=find(ismember(nodes,p.E,'rows'),1);
Sidx=zeros(1,nS); Widx=zeros(1,nW); startIdx=zeros(1,2);
for s=1:nS, Sidx(s)=find(ismember(nodes,p.S(s,:),'rows'),1); end
for w=1:nW, Widx(w)=find(ismember(nodes,p.W(w,:),'rows'),1); end
for k=1:2, startIdx(k)=find(ismember(nodes,initial(k).position,'rows'),1); end

from=[];to=[];duration=[];
functionalOrCommitted=ismember(nodes,[p.B;p.S;p.W;p.E;committedTargets],'rows');
for i=1:nNode
    if i==Eidx, continue; end
    for j=1:nNode
        % 当前中间格只作为起点，不作为新的追逐目标；新航段只能驶向
        % 功能点或已经承诺的目标，避免追逐另一艘船上一日的临时位置。
        if i==j || ~functionalOrCommitted(j), continue; end
        d=sum(abs(nodes(i,:)-nodes(j,:)));
        if d>=1 && d<=H
            from(end+1,1)=i;to(end+1,1)=j;duration(end+1,1)=d; %#ok<AGROW>
        end
    end
end
nArc=numel(from);
cap=p.capacity;pr=p.price;expM=p.expectedMove;
expI=p.expectedIdle;expW=p.expectedWork;maxW=p.maxWork;

next=0; xIdx=cell(2,1); travelIdx=cell(2,1); stayIdx=cell(2,1);
workIdx=cell(2,1); buyIdx=cell(2,1); stateIdx=cell(2,1);
for k=1:2
    xIdx{k}=reshape(next+(1:nNode*(H+1)),nNode,H+1);next=next+nNode*(H+1);
    travelIdx{k}=reshape(next+(1:nArc*H),nArc,H);next=next+nArc*H;
    stayIdx{k}=reshape(next+(1:nNode*H),nNode,H);next=next+nNode*H;
    workIdx{k}=reshape(next+(1:nW*H),nW,H);next=next+nW*H;
    buyIdx{k}=reshape(next+(1:nR*nS*H),nR,nS,H);next=next+nR*nS*H;
    stateIdx{k}=reshape(next+(1:4*(H+1)),4,H+1);next=next+4*(H+1);
end
meetIdx=reshape(next+(1:nNode*H),nNode,H);next=next+nNode*H;
trNetIdx=reshape(next+(1:4*nNode*H),4,nNode,H);next=next+4*nNode*H;
trAbsIdx=reshape(next+(1:4*nNode*H),4,nNode,H);next=next+4*nNode*H;
nVar=next; lb=zeros(nVar,1);ub=inf(nVar,1);

binary=[];integerTransfer=[];
for k=1:2
    binary=[binary;xIdx{k}(:);travelIdx{k}(:);stayIdx{k}(:);workIdx{k}(:)]; %#ok<AGROW>
    ub(buyIdx{k}(:))=cap;
    ub(stateIdx{k}(1:3,:))=cap;
    ub(stateIdx{k}(4,:))=initial(k).M+initial(3-k).M;
    % 一般状态上界必须先设置；随后再把第1列严格固定为当天真实状态。
    % 若顺序相反，容量上界会覆盖初始状态上界，使资源和资金凭空增加。
    init=[initial(k).Water;initial(k).Fuel;initial(k).Food;initial(k).M];
    lb(stateIdx{k}(:,1))=init;ub(stateIdx{k}(:,1))=init;
end
binary=[binary;meetIdx(:)];ub(binary)=1;

% 必须在ub(binary)=1之后施加固定/禁止条件，否则零上界会被重新覆盖。
for k=1:2
    lb(xIdx{k}(startIdx(k),1))=1;ub(xIdx{k}(startIdx(k),1))=1;
    for i=1:nNode
        if i~=startIdx(k),ub(xIdx{k}(i,1))=0;end
    end
    if initial(k).arrived
        for i=1:nNode
            if i~=Eidx,ub(xIdx{k}(i,:))=0;end
        end
    end
    % 宏观弧承诺：尚未抵达既定目标时必须继续驶向该目标。
    if all(isfinite(initial(k).target))
        % 一旦开始一条宏观航段，就在模型内部连续完成该航段，避免滚动窗口
        % 每天把移动推迟到窗口末端。此约束取代求解后的强制移动。
        ub(stayIdx{k}(startIdx(k),1))=0;
        for a=find(from==startIdx(k))'
            if ~isequal(nodes(to(a),:),initial(k).target)
                ub(travelIdx{k}(a,1))=0;
            end
        end
    end
end
lb(trNetIdx(:))=-1500;ub(trNetIdx(:))=1500;ub(trAbsIdx(:))=1500;
integerTransfer=trNetIdx(:);
if ~strategy.allowTransfer
    lb(integerTransfer)=0;ub(integerTransfer)=0;
end
intcon=[binary;buyIdx{1}(:);buyIdx{2}(:);integerTransfer];

% 机会约束规划参数。未来仍以Q3概率的期望消耗为预测中心；安全裕度由
% Binomial(n,0.2)分位数给出。这里的风险预算只用于构造保守规划裕量，
% 不作为整个滚动策略的99%解析证书；最终可靠性由独立蒙特卡洛复演报告。
riskPieces=max(1,2*ceil(fullRemaining/max(1,H)));
stageAlpha=1-(1-p.arrivalProbability)/riskPieces;
futureRandomDays=max(0,H-1);
qWindow=binomialQuantile(futureRandomDays,p.pStorm,stageAlpha);
deltaActions=[p.stormMove-p.normalMove; ...
    p.stormIdle-p.normalIdle;p.stormWork-p.normalWork];
maxDelta=max(deltaActions,[],1);minDelta=min(deltaActions,[],1);
reserveTable=zeros(3,H+1);
for nr=0:H,reserveTable(:,nr+1)=chanceDeviationReserve(nr,qWindow,p.pStorm,maxDelta,minDelta);end
for k=1:2
    for h=1:H
        nRandom=max(0,h-1);
        reserve=reserveTable(:,nRandom+1);
        lb(stateIdx{k}(1:3,h+1))=max(lb(stateIdx{k}(1:3,h+1)),reserve(:));
    end
end

% 禁止无效宏观弧、终点普通停泊消耗、以及角色不允许的作业。
for k=1:2
    for h=1:H
        ub(travelIdx{k}(duration>H-h+1,h))=0;
    end
    for w=1:nW
        if ~strategy.allowedWork(k,w) || fullRemaining<=p.evacuationDays
            ub(workIdx{k}(w,:))=0;
        end
    end
end

eqCols=cell(0,1);eqVals=cell(0,1);beq=[];nEq=0;
inCols=cell(0,1);inVals=cell(0,1);b=[];nIn=0;

% 每艘船的宏观时间扩展流。
for k=1:2
    for h=1:H
        for i=1:nNode
            outArcs=travelIdx{k}(from==i,h);
            cols=[outArcs;stayIdx{k}(i,h)];vals=ones(numel(cols),1);
            wi=find(Widx==i,1);
            if ~isempty(wi),cols(end+1,1)=workIdx{k}(wi,h);vals(end+1,1)=1;end
            addEq([cols;xIdx{k}(i,h)],[vals;-1],0);

            cols=[xIdx{k}(i,h+1);stayIdx{k}(i,h)];vals=[1;-1];
            if ~isempty(wi),cols(end+1,1)=workIdx{k}(wi,h);vals(end+1,1)=-1;end
            for a=find(to==i)'
                startTime=h-duration(a)+1;
                if startTime>=1
                    cols(end+1,1)=travelIdx{k}(a,startTime);vals(end+1,1)=-1; %#ok<AGROW>
                end
            end
            addEq(cols,vals,0);
        end
    end
end

% 终点吸收：stay变量保留流但资源消耗为0。
for k=1:2
    for h=1:H
        ub(travelIdx{k}(from==Eidx,h))=0;
    end
end

% 会合与交换门控。
for h=1:H
    for i=1:nNode
        addLe([meetIdx(i,h);xIdx{1}(i,h)],[1;-1],0);
        addLe([meetIdx(i,h);xIdx{2}(i,h)],[1;-1],0);
        addLe([xIdx{1}(i,h);xIdx{2}(i,h);meetIdx(i,h)],[1;1;-1],1);
        meetingAllowed=strategy.allowTransfer && i~=Eidx && ...
            (isempty(strategy.meetingNodes) || ...
            any(ismember(strategy.meetingNodes,nodes(i,:),'rows')));
        if ~meetingAllowed
            % meetIdx表示物理同点，不能因为该点禁止交换就把同点本身禁掉；
            % 这里只关闭交换变量。否则两船同在起点B会立刻造成模型不可行。
            lb(trNetIdx(:,i,h))=0;ub(trNetIdx(:,i,h))=0;
            ub(trAbsIdx(:,i,h))=0;
        else
            for g=1:4
                U=400;if g==4,U=1500;end
                lb(trNetIdx(g,i,h))=-U;ub(trNetIdx(g,i,h))=U;
                ub(trAbsIdx(g,i,h))=U;
                addLe([trNetIdx(g,i,h);meetIdx(i,h)],[1;-U],0);
                addLe([trNetIdx(g,i,h);meetIdx(i,h)],[-1;-U],0);
                addLe([trNetIdx(g,i,h);trAbsIdx(g,i,h)],[1;-1],0);
                addLe([trNetIdx(g,i,h);trAbsIdx(g,i,h)],[-1;-1],0);
            end
        end
    end
end

% 补给、交换、资源和资金平衡。
for k=1:2
    transferSign=1;if k==2,transferSign=-1;end
    moveCost=[weatherUse(todayWeather,p,'move',1),weatherUse(todayWeather,p,'move',2),weatherUse(todayWeather,p,'move',3)];
    idleCostCache=[weatherUse(todayWeather,p,'idle',1),weatherUse(todayWeather,p,'idle',2),weatherUse(todayWeather,p,'idle',3)];
    workCostCache=[weatherUse(todayWeather,p,'work',1),weatherUse(todayWeather,p,'work',2),weatherUse(todayWeather,p,'work',3)];
    for h=1:H
        for s=1:nS
            for r=1:3
                addLe([buyIdx{k}(r,s,h);xIdx{k}(Sidx(s),h)],[1;-cap],0);
            end
        end
        netTr=reshape(trNetIdx(:,:,h),4,[]);

        % 交换发生在采购之前。正值表示1->2，负值表示2->1；单一净变量
        % 从变量定义上消除了同一资源的双向对倒。
        for g=1:4
            reserveBefore=0;
            if g<=3
                nRandomBefore=max(0,h-2);
                tmp=reserveTable(:,nRandomBefore+1);
                reserveBefore=tmp(g);
            end
            addLe([netTr(g,:)';stateIdx{k}(g,h)], ...
                [transferSign*ones(nNode,1);-1],-reserveBefore);
        end

        % 交换后采购，约束资金和载重。
        allBuy=reshape(buyIdx{k}(:,:,h),[],1);
        moneyCost=repmat(pr(:),nS,1);
        addLe([allBuy;netTr(4,:)';stateIdx{k}(4,h)], ...
            [moneyCost;transferSign*ones(nNode,1);-1],0);
        capCols=[stateIdx{k}(1:3,h);allBuy;reshape(netTr(1:3,:),[],1)];
        capVals=[ones(3,1);ones(3*nS,1);-transferSign*ones(3*nNode,1)];
        addLe(capCols,capVals,cap);

        for r=1:3
            purchases=reshape(buyIdx{k}(r,:,h),[],1);
            cols=[stateIdx{k}(r,h+1);stateIdx{k}(r,h);purchases;netTr(r,:)'];
            vals=[1;-1;-ones(nS,1);transferSign*ones(nNode,1)];
            % 宏观移动弧在起点一次扣除整段期望消耗。
            for a=1:nArc
                d=duration(a);cost=d*p.expectedMove(r);
                if h==1,cost=moveCost(r)+(d-1)*p.expectedMove(r);end
                cols(end+1,1)=travelIdx{k}(a,h);vals(end+1,1)=cost; %#ok<AGROW>
            end
            idleCost=p.expectedIdle(r);workCost=p.expectedWork(r);
            if h==1
                idleCost=idleCostCache(r);
                workCost=workCostCache(r);
            end
            nonERef=setdiff(1:nNode,Eidx);
            nonE=nonERef;
            cols=[cols;stayIdx{k}(nonE,h);workIdx{k}(:,h)];
            vals=[vals;idleCost*ones(numel(nonE),1);workCost*ones(nW,1)];
            addEq(cols,vals,0);
        end
        addEq([stateIdx{k}(4,h+1);stateIdx{k}(4,h);allBuy;netTr(4,:)'], ...
            [1;-1;moneyCost;transferSign*ones(nNode,1)],0);
    end
end

% 连续作业限制及历史衔接。
for k=1:2
    for w=1:nW
        L=maxW(w);
        for h0=1:H-L,addLe(workIdx{k}(w,h0:h0+L)',ones(L+1,1),L);end
    end
    if initial(k).workPoint>=1&&initial(k).consecutiveDays>0
    allowed=maxW(w)-initial(k).consecutiveDays;
        window=min(H,allowed+1);
        if window==allowed+1,addLe(workIdx{k}(w,1:window)',ones(window,1),allowed);end
    end
end

% 窗口终端：最后窗口必须到E；此前保证剩余时间仍可达E并留安全资源。
for k=1:2
    if H==fullRemaining
        addEq(xIdx{k}(Eidx,H+1),1,1);
    else
        stepsLeft=fullRemaining-H;
        for i=1:nNode
            if sum(abs(nodes(i,:)-p.E))>stepsLeft,ub(xIdx{k}(i,H+1))=0;end
        end
        % 窗口末端保留按stageAlpha分位数直接抵达E的安全撤离能力；
        % 最终可靠性由独立蒙特卡洛检验，而不是把该规划裕量当成解析证书。
        safeDist=sum(abs(nodes-p.E),2);
        for r=1:3
            chanceNeed=zeros(nNode,1);
            for i=1:nNode
                chanceNeed(i)=chanceMoveRequirement(safeDist(i),r,p,stageAlpha);
            end
            addLe([xIdx{k}(:,H+1);stateIdx{k}(r,H+1)], ...
                [chanceNeed;-1],0);
        end
    end
end

eqLen=cellfun(@numel,eqCols);eqRows=repelem((1:nEq)',eqLen);
Aeq=sparse(eqRows,vertcat(eqCols{:}),vertcat(eqVals{:}),nEq,nVar);
inLen=cellfun(@numel,inCols);inRows=repelem((1:nIn)',inLen);
A=sparse(inRows,vertcat(inCols{:}),vertcat(inVals{:}),nIn,nVar);

rewardCoef=zeros(nVar,1);
for k=1:2
    for w=1:nW,rewardCoef(workIdx{k}(w,:))=p.reward(w);end
end
options=optimoptions('intlinprog','Display','off','RelativeGapTolerance', ...
    p.relativeGap,'AbsoluteGapTolerance',0,'MaxTime',p.maxSolveTime);

% 第一阶段：只最大化团队总目标物资。
fZ=-rewardCoef;
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

% 第二阶段：固定第一阶段收益，再最大化团队剩余资金。
% 极小的正交换成本只在资金完全相同时最小化总交换量；其总影响严格小于0.25，
% 因而不可能用1单位资金换取更少的交换量。
zRow=sparse(1,nVar);zRow(1,find(rewardCoef))=rewardCoef(rewardCoef~=0);
Aeq2=[Aeq;zRow];beq2=[beq;zStar];
fM=zeros(nVar,1);
fM(stateIdx{1}(4,H+1))=-1;fM(stateIdx{2}(4,H+1))=-1;
allTransferAbs=trAbsIdx(:);
transferEpsilon=0.19/(1+sum(ub(allTransferAbs)));
fM(allTransferAbs)=transferEpsilon;
terminalDistance=sum(abs(nodes-p.E),2);
distanceEpsilon=0.19/(1+2*max(terminalDistance));
stayEpsilon=0.19/2;
for k=1:2
    fM(xIdx{k}(:,H+1))=fM(xIdx{k}(:,H+1))+distanceEpsilon*terminalDistance;
    nonE=nonERef;
    fM(stayIdx{k}(nonE,1))=fM(stayIdx{k}(nonE,1))+stayEpsilon;
end
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

% 不仅检查返回向量是否非空，还验证它确实满足当前MILP。这样时间限制或
% 数值异常返回的非可行向量不会进入实际执行。
ineqViolation=0;if ~isempty(A),ineqViolation=max([0;A*x-b]);end
eqViolation=0;if ~isempty(Aeq2),eqViolation=max(abs(Aeq2*x-beq2));end
boundViolation=max([0;lb-x;x-ub]);
integerViolation=max(abs(x(intcon)-round(x(intcon))));
solverViolation=max([ineqViolation;eqViolation;boundViolation;integerViolation]);
if solverViolation>1e-5
    plan=struct('feasible',false,'reason', ...
        sprintf("求解器返回向量违反MILP约束，最大误差=%.3g",solverViolation));return;
end
% 提取今日补给、交换和第一步实际行动。实际执行必须与MILP首期决策完全一致；
% 不再在求解后增加采购量、强制移动或改写作业决策。
buy=zeros(2,3);tr12=zeros(1,4);tr21=zeros(1,4);
for k=1:2,for r=1:3,buy(k,r)=round(sum(x(buyIdx{k}(r,:,1))));end,end
netToday=round(sum(x(trNetIdx(:,:,1)),2))';
tr12=max(netToday,0);tr21=max(-netToday,0);
if (any(tr12>0)||any(tr21>0)) && ...
        ~isequal(initial(1).position,initial(2).position)
    plan=struct('feasible',false,'reason',"检测到非同点交换");return;
end
nextState=initial;actions=strings(2,1);gains=zeros(2,1);actionType=zeros(1,2,'uint8');
actMoveUse=[weatherUse(todayWeather,p,'move',1),weatherUse(todayWeather,p,'move',2),weatherUse(todayWeather,p,'move',3)];
actWorkUse=[weatherUse(todayWeather,p,'work',1),weatherUse(todayWeather,p,'work',2),weatherUse(todayWeather,p,'work',3)];
actIdleUse=[weatherUse(todayWeather,p,'idle',1),weatherUse(todayWeather,p,'idle',2),weatherUse(todayWeather,p,'idle',3)];
for k=1:2
    % 交换 -> 采购。
    vec=[nextState(k).Water,nextState(k).Fuel,nextState(k).Food,nextState(k).M];
    if k==1,vec=vec-tr12+tr21;else,vec=vec+tr12-tr21;end
    vec(1:3)=vec(1:3)+buy(k,:);vec(4)=vec(4)-sum(buy(k,:).*p.price);
    if nextState(k).arrived
        actions(k)="已退出";
        nextState(k).Water=round(vec(1));nextState(k).Fuel=round(vec(2));
        nextState(k).Food=round(vec(3));nextState(k).M=round(vec(4));
        continue;
    end
    chosen=find(x(travelIdx{k}(:,1))>0.5,1);wi=find(x(workIdx{k}(:,1))>0.5,1);
    if ~isempty(chosen)
        target=nodes(to(chosen),:);newPos=oneStep(initial(k).position,target);
        actions(k)=sprintf('向(%d,%d)移动至(%d,%d)',target(1),target(2),newPos(1),newPos(2));
        actionType(k)=1;
        use=actMoveUse;
        nextState(k).position=newPos;nextState(k).target=target;
        nextState(k).workPoint=0;nextState(k).consecutiveDays=0;
        if isequal(newPos,target),nextState(k).target=[NaN,NaN];end
    elseif ~isempty(wi)
        actions(k)=sprintf('在W%d作业',wi);gains(k)=p.reward(wi);
        actionType(k)=3;
        use=actWorkUse;
        if nextState(k).workPoint==wi,nextState(k).consecutiveDays=nextState(k).consecutiveDays+1;
        else,nextState(k).workPoint=wi;nextState(k).consecutiveDays=1;end
    else
        actions(k)="停泊";
        actionType(k)=2;
        use=actIdleUse;
        nextState(k).workPoint=0;nextState(k).consecutiveDays=0;
    end
    vec(1:3)=vec(1:3)-use;
    nextState(k).Water=round(vec(1));nextState(k).Fuel=round(vec(2));nextState(k).Food=round(vec(3));
    nextState(k).M=round(vec(4));nextState(k).Z=nextState(k).Z+gains(k);
    if isequal(nextState(k).position,p.E),nextState(k).arrived=true;end
end
for k=1:2
    if nextState(k).Water<0||nextState(k).Fuel<0||nextState(k).Food<0||nextState(k).M<0|| ...
            nextState(k).Water+nextState(k).Fuel+nextState(k).Food>p.capacity
        plan=struct('feasible',false,'reason',sprintf( ...
            "执行后状态违规：船%d 淡水/燃油/食物/资金=%g/%g/%g/%g，载重=%g",k, ...
            nextState(k).Water,nextState(k).Fuel,nextState(k).Food,nextState(k).M, ...
            nextState(k).Water+nextState(k).Fuel+nextState(k).Food));return;
    end
end
provenWithinTolerance=flagZ>0 && flagM>0;
plan=struct('feasible',true,'next',nextState,'action',actions,'buy',buy, ...
    'actionType',actionType,'gain',gains,'transfer12',tr12,'transfer21',tr21, ...
    'predictedTeamZ',initial(1).Z+initial(2).Z+round(rewardCoef'*x), ...
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

function reserve=chanceDeviationReserve(n,qWindow,pStorm,maxDelta,minDelta)
% 在总雷暴日数不超过qWindow的事件上，对“实际消耗-期望消耗”取线性上界。
qPrefix=min(n,qWindow);
reserve=max(0,qPrefix.*maxDelta-pStorm*n.*minDelta);
end

function need=chanceMoveRequirement(distance,r,p,alpha)
% 连续移动distance天时资源r的精确二项分布分位数需求。
if distance<=0,need=0;return;end
q=binomialQuantile(distance,p.pStorm,alpha);
need=distance*p.normalMove(r)+q*(p.stormMove(r)-p.normalMove(r));
end

function q=binomialQuantile(n,probability,alpha)
% 不依赖Statistics Toolbox的Binomial(n,p)左分位数。
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

function best=selectBestStrategy(simulations,eligible)
best=0;
for s=1:numel(simulations)
    q=simulations{s};
    if ~eligible(s) || ~q.feasible,continue;end
    if best==0 || q.teamZ>simulations{best}.teamZ || ...
            (q.teamZ==simulations{best}.teamZ && q.teamM>simulations{best}.teamM)
        best=s;
    end
end
end

function mc=validateFixedPolicyMonteCarlo(simulation,p,nSamples,seed)
% 用独立于优化天气的样本，复演已经输出的每日采购、交换和行动。
% 这是对“固定实现策略”的样本外检验，不冒充解析概率证明。
mc=struct('samples',nSamples,'successes',0,'successRate',NaN,'lower95',NaN, ...
    'method',"独立固定策略蒙特卡洛复演");
if ~simulation.feasible || simulation.controls.days<=0,return;end
rng(seed,'twister');
nDays=simulation.controls.days;
stormSample=rand(nSamples,nDays)>=p.pNormal;
success=false(nSamples,1);
for sample=1:nSamples
    resources=[100,150,100;100,150,100];money=[750;750];ok=true;
    for t=1:nDays
        tr12=simulation.controls.transfer12(t,:);
        tr21=simulation.controls.transfer21(t,:);
        v1=[resources(1,:),money(1)]-tr12+tr21;
        v2=[resources(2,:),money(2)]+tr12-tr21;
        if any(v1<-1e-9)||any(v2<-1e-9),ok=false;break;end
        resources=[v1(1:3);v2(1:3)];money=[v1(4);v2(4)];
        dailyBuy=reshape(simulation.controls.buy(t,:,:),2,3);
        for k=1:2
            cost=sum(dailyBuy(k,:).*p.price);
            if cost>money(k)+1e-9 || sum(resources(k,:)+dailyBuy(k,:))>p.capacity+1e-9
                ok=false;break;
            end
            resources(k,:)=resources(k,:)+dailyBuy(k,:);money(k)=money(k)-cost;
        end
        if ~ok,break;end
        isStorm=stormSample(sample,t);
        for k=1:2
            kind=simulation.controls.actionType(t,k);
            if kind==0,continue;
            elseif kind==1
                if isStorm,use=p.stormMove;else,use=p.normalMove;end
            elseif kind==2
                if isStorm,use=p.stormIdle;else,use=p.normalIdle;end
            else
                if isStorm,use=p.stormWork;else,use=p.normalWork;end
            end
            resources(k,:)=resources(k,:)-use;
            if any(resources(k,:)<-1e-9),ok=false;break;end
        end
        if ~ok,break;end
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
d=target-p0;
[~,idx]=max(abs(d));
q=p0;q(idx)=q(idx)+sign(d(idx));
end
end

function s=posText(p),s=sprintf('(%d,%d)',p(1),p(2));end

function s=transferText(a,b)
s=sprintf('1->2 淡水/燃油/食物/资金=%d/%d/%d/%d; 2->1=%d/%d/%d/%d',a(1),a(2),a(3),a(4), ...
    b(1),b(2),b(3),b(4));
end

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
