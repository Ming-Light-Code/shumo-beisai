function solve_q2_mixed()
% Mixed weather: Day 1 thunderstorm, Days 2-30 normal
% TS: move O=8 H=4 F=3, park O=3 H=3 F=2
% Normal: move O=2 H=3 F=2, work O=5 H=4 F=3, park O=1 H=1 F=1

fprintf('========================================\n');
fprintf('  Mixed Weather: Day1 TS + Days 2-30 Normal\n');
fprintf('========================================\n\n');

day1_opts = { 'park@B', [1,5], 3, 3, 2;
              'move->(2,5)', [2,5], 8, 4, 3;
              'move->(1,6)', [1,6], 8, 4, 3 };
bestZ=-inf; bestM=-inf; best_sol=struct();

for di=1:size(day1_opts,1)
    label=day1_opts{di,1}; pos=day1_opts{di,2};
    o_cost=cell2mat(day1_opts(di,3)); h_cost=cell2mat(day1_opts(di,4)); f_cost=cell2mat(day1_opts(di,5));
    O=35-o_cost; H=45-h_cost; F=30-f_cost; M=240;
    if O<0||H<0||F<0, continue; end
    d_to_S2=abs(pos(1)-7)+abs(pos(2)-6);
    O2=O-d_to_S2*2; H2=H-d_to_S2*3; F2=F-d_to_S2*2;
    if O2<0, continue; end
    need_O=27; need_H=30; need_F=21;
    bO=max(0,need_O-O2); bH=max(0,need_H-H2); bF=max(0,need_F-F2);
    load=O2+H2+F2;
    if bO+bH+bF>120-load, continue; end
    cost1=bO*2+bH+bF*2; M1=M-cost1;
    if M1<0, continue; end
    days_used=1+d_to_S2+9;
    for w3=1:8
        min_p=max(0,ceil(w3/3)-1);
        for ep=0:2, p=min_p+ep; stay=w3+p;
            if days_used+8+stay>30, continue; end
            nO=16+w3*5+p; nH=24+w3*4+p; nF=16+w3*3+p;
            if nO+nH+nF>120, continue; end
            cost2=nO*2+nH+nF*2;
            if M1<cost2, continue; end
            M2=M1-cost2; Z2=100+3*28+w3*28;
            if Z2>bestZ||(Z2==bestZ&&M2>bestM)
                bestZ=Z2; bestM=M2;
                best_sol.label=label; best_sol.d_to_S2=d_to_S2;
                best_sol.w3=w3; best_sol.pk=p;
                best_sol.O2=O2; best_sol.H2=H2; best_sol.F2=F2;
                best_sol.bO=bO; best_sol.bH=bH; best_sol.bF=bF;
                best_sol.cost1=cost1; best_sol.M1=M1;
                best_sol.nO=nO; best_sol.nH=nH; best_sol.nF=nF;
                best_sol.cost2=cost2;
            end
        end
    end
end

fprintf('===== OPTIMAL SOLUTION =====\n');
fprintf('Z = %d\n',bestZ);
fprintf('M = %d\n',bestM);
fprintf('Day 1: %s (thunderstorm)\n',best_sol.label);
fprintf('Normal: B->S2(%d) -> W3(3d) -> S2 -> W3(%dw+%dp) -> E\n',best_sol.d_to_S2,best_sol.w3,best_sol.pk);
fprintf('S2 buy1: +O%d +H%d +F%d cost=%d M=%d\n',best_sol.bO,best_sol.bH,best_sol.bF,best_sol.cost1,best_sol.M1);
fprintf('S2 buy2: +O%d +H%d +F%d cost=%d\n',best_sol.nO,best_sol.nH,best_sol.nF,best_sol.cost2);
fprintf('Total W3: 3+%d=%d work days. Z=100+%d*28=%d\n',best_sol.w3,3+best_sol.w3,3+best_sol.w3,100+(3+best_sol.w3)*28);
fprintf('\nDone.\n');
end
