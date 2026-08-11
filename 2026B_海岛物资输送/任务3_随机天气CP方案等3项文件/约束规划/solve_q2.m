function solve_q2()
% solve_q2.m - Task 2: Thunderstorm extreme case (30 thunderstorm days)
% Move: O=8 H=4 F=3 | Park: O=3 H=3 F=2 | Work: O=8 H=6 F=6
MAX_DAYS=30; MAX_LOAD=120;
all_xy=[1 5;10 5;2 7;5 3;8 8;3 4;7 6];
mO=8; mH=4; mF=3; pO=3; pH=3; pF=2; wO=8; wH=6; wF=6;
d_BS1=3; d_S1E=8; d_BS2=7; d_S2E=4; d_S1S2=5;
fprintf('========================================\n');
fprintf('  Task 2: 30-Day Thunderstorm Extreme Case\n');
fprintf('========================================\n\n');
fprintf('Connectivity: B->S1=%d cells (OK), B->S2=%d cells (O=%d>35, NO)\n\n',d_BS1,d_BS2,d_BS2*8);

bestM=-inf; bestC=0; bestP=0;
fprintf('Enumerating B->S1->E (c cells + p parks):\n');
for c=3:10
    for p=0:30
        O=35-mO*c-pO*p; H=45-mH*c-pH*p; F=30-mF*c-pF*p;
        if O<0||H<0||F<0, break; end
        load=O+H+F; sp=MAX_LOAD-load;
        On=max(0,mO*d_S1E-O); Hn=max(0,mH*d_S1E-H); Fn=max(0,mF*d_S1E-F);
        need=On+Hn+Fn;
        if need<=sp
            cost=2*On+Hn+2*Fn; M=240-cost;
            fprintf('  c=%d p=%d: M=%d\n',c,p,M);
            if M>bestM, bestM=M; bestC=c; bestP=p; end
        end
    end
end

fprintf('\n===== OPTIMAL (Thunderstorm Extreme) =====\n');
fprintf('Z = 100  (no work feasible)\n');
fprintf('M = %d\n',bestM);
fprintf('Route: B(%d cells+%d parks)->S1->E(8 cells)\n',bestC,bestP);
total_days=bestC+bestP+d_S1E;
fprintf('Total: %d days (Travel=%d, Park=%d)\n',total_days,bestC+d_S1E,bestP);

fprintf('\n===== DAY-BY-DAY SCHEDULE =====\n');
fprintf('Day | Pos (x,y)  | Action      |  O   H   F  Load |   Z     M\n');
fprintf('----|-------------|-------------|------------------|------------\n');
O=35; H=45; F=30; M=240; Z=100; day=0;
% Simulate B->S1 with zigzag + park
pos=[1,5];
for i=1:bestC
    if i==1, pos=[2,5]; elseif i==2, pos=[3,5]; else pos=[3,4]; end
    day=day+1; O=O-mO; H=H-mH; F=F-mF;
    fprintf('%3d | (%2d,%2d)     | move        | %3d %3d %3d %4d | %4d %5d\n',day,pos(1),pos(2),O,H,F,O+H+F,Z,M);
end
for pi=1:bestP
    day=day+1; O=O-pO; H=H-pH; F=F-pF;
    fprintf('%3d | (%2d,%2d)     | park(at sea)| %3d %3d %3d %4d | %4d %5d\n',day,pos(1),pos(2),O,H,F,O+H+F,Z,M);
end
On=max(0,mO*d_S1E-O); Hn=max(0,mH*d_S1E-H); Fn=max(0,mF*d_S1E-F);
cost=2*On+Hn+2*Fn; M=M-cost; O=O+On; H=H+Hn; F=F+Fn;
fprintf('%3d | (%2d,%2d)     | SUPPLY(S1)  | %3d %3d %3d %4d | %4d %5d  (+O%d H%d F%d)\n',day,3,4,O,H,F,O+H+F,Z,M,On,Hn,Fn);
% S1->E
path_to_E={[4,4],[5,4],[6,4],[7,4],[8,4],[9,4],[9,5],[10,5]};
for i=1:d_S1E
    p=path_to_E{i}; day=day+1; O=O-mO; H=H-mH; F=F-mF;
    fprintf('%3d | (%2d,%2d)     | move        | %3d %3d %3d %4d | %4d %5d\n',day,p(1),p(2),O,H,F,O+H+F,Z,M);
end
fprintf('----|-------------|-------------|------------------|------------\n');
fprintf('  Final at E: Z=%d M=%d Day=%d\n',Z,M,day);
fprintf('\nDone.\n');
end
