$pythonCode = @"
import os
lines = []
def L(s): lines.append(s)

L("function solve_q3_new()")
L("if ~exist(\"intlinprog\",\"file\"), error(\"Need Optimization Toolbox.\"); end")
L("rng(2026); tic;")
L("fprintf(\"=== TASK 3: CCASR ===\\n\");")
L("%% ---- Data ----")
L("all_xy=[1 15;30 15;6 21;15 9;24 24;12 16;21 16];")
L("names={\"B\",\"E\",\"W1\",\"W2\",\"W3\",\"S1\",\"S2\"};")
L("WY=[20,15,28]; WM=[4,5,3];")
L("LD=400; MD=90; IO=100; IH=150; IF=100; IM=750; IZ=200;")
L("CMn=[2,3,2]; CWn=[5,4,3]; CSn=[1,1,1];")
L("CMs=[8,4,3]; CWs=[8,6,6]; CSs=[3,3,2];")
L("CMe=0.8*CMn+0.2*CMs; CWe=0.8*CWn+0.2*CWs; CSe=0.8*CSn+0.2*CSs;")
L("VM=0.16*(CMs-CMn).^2; VW=0.16*(CWs-CWn).^2; VS=0.16*(CSs-CSn).^2;")
L("ZA=1.645; PR=[2,1,2];")
L("nP=7; d=zeros(nP);")
L("for i=1:nP,for j=1:nP,d(i,j)=abs(all_xy(i,1)-all_xy(j,1))+abs(all_xy(i,2)-all_xy(j,2));end;end")

with open(r"C:\Users\ming\Desktop\solve_q3_new.m", "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"Written {len(lines)} lines")
"@

$pythonCode | python -
