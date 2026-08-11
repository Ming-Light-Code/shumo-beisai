function solve_q2_5ts()
% Mixed weather: First 5 days thunderstorm, Days 6-30 normal

fprintf('========================================\n');
fprintf('  5-Day TS + 25-Day Normal Optimization\n');
fprintf('========================================\n\n');

% Enumerate all TS-day strategies: n_move toward S2 + n_park
bestZ=-inf; bestM=-inf; best=struct();
for nm=0:4  % TS moves (max 4 with O=35)
    np=5-nm;  % TS park days
    O=35-nm*8-np*3; H=45-nm*4-np*3; F=30-nm*3-np*2;
    if O<0||H<0||F<0, continue; end
    cx=1+nm; cy=5;  % position after TS days
    d=abs(cx-7)+abs(cy-6);  % to S2
    O2=O-d*2; H2=H-d*3; F2=F-d*2;
    if O2<0, continue; end
    bO=max(0,27-O2); bH=max(0,30-H2); bF=max(0,21-F2);
    load=O2+H2+F2;
    if bO+bH+bF>120-load, continue; end
    c1=bO*2+bH+bF*2; M1=240-c1;
    if M1<0, continue; end
    du=5+d+9;  % days used so far
    for w3=1:8
        mp=max(0,ceil(w3/3)-1);
        for ep=0:2, p=mp+ep; stay=w3+p;
            if du+8+stay>30, continue; end
            nO=16+w3*5+p; nH=24+w3*4+p; nF=16+w3*3+p;
            if nO+nH+nF>120, continue; end
            c2=nO*2+nH+nF*2;
            if M1<c2, continue; end
            M2=M1-c2; Z2=100+3*28+w3*28;
            if Z2>bestZ||(Z2==bestZ&&M2>bestM)
                bestZ=Z2; bestM=M2;
                best.nm=nm; best.np=np; best.pos=[cx,cy];
                best.w3=w3; best.pk=p;
                best.O=O; best.H=H; best.F=F;
                best.bO=bO; best.bH=bH; best.bF=bF;
                best.c1=c1; best.M1=M1;
                best.nO=nO; best.nH=nH; best.nF=nF;
                best.c2=c2;
            end
        end
    end
end

fprintf('===== OPTIMAL SOLUTION =====\n');
fprintf('Z = %d\n',bestZ);
fprintf('M = %d\n',bestM);
fprintf('TS days: %d moves + %d parks, end at (%d,%d)\n',best.nm,best.np,best.pos(1),best.pos(2));
fprintf('Normal: (%d,%d)->S2(%d cells)->W3(3d)->S2->W3(%dw+%dp)->E\n',best.pos(1),best.pos(2),abs(best.pos(1)-7)+abs(best.pos(2)-6),best.w3,best.pk);
fprintf('S2 buy1: +O%d +H%d +F%d cost=%d M=%d\n',best.bO,best.bH,best.bF,best.c1,best.M1);
fprintf('S2 buy2: +O%d +H%d +F%d cost=%d\n',best.nO,best.nH,best.nF,best.c2);
fprintf('Total W3: 3+%d=%d days -> Z=100+%d*28=%d\n',best.w3,3+best.w3,3+best.w3,100+(3+best.w3)*28);
fprintf('\nDone.\n');
end
