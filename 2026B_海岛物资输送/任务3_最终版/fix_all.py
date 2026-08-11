import os

# ===== FIX 1: B v4 - update solve_q3_mdp_v3.m to call v4 =====
path = r'C:\Users\ming\Desktop\任务3_最终版\优化B_MDP方向\solve_q3_mdp_v3.m'
with open(path, 'r', encoding='utf-8') as f:
    content = f.read()

count = content.count('mdp_solver_v3')
print(f'B: mdp_solver_v3 calls: {count}')
content = content.replace('mdp_solver_v3', 'mdp_solver_v4')
with open(path, 'w', encoding='utf-8') as f:
    f.write(content)
print('B: Updated to mdp_solver_v4')

# ===== FIX 2: C v4 - improve tail_simulate to route through supply =====
path2 = r'C:\Users\ming\Desktop\任务3_最终版\优化C_滚动随机优化\rso_solver_v4.m'
with open(path2, 'r', encoding='utf-8') as f:
    content2 = f.read()

# Find and replace the tail_simulate MOVE-toward-E section
old_move = """        % 4. MOVE toward E
        if ~action_taken
            dx=to_E(1)-sim_pos(1); dy=to_E(2)-sim_pos(2);
            if abs(dx)>0, sim_pos(1)=sim_pos(1)+sign(dx);
            elseif abs(dy)>0, sim_pos(2)=sim_pos(2)+sign(dy);
            end
            sim_O=sim_O-ca.MO; sim_H=sim_H-ca.MH; sim_F=sim_F-ca.MF; cw=0;
            % Check arrival at any node
            for nd=1:7
                if sim_pos(1)==cfg.xy(nd,1)&&sim_pos(2)==cfg.xy(nd,2)
                    sim_pt=nd; break;
                end
            end
        end"""

new_move = """        % 4. MOVE: toward E, or toward nearest supply if resources low
        if ~action_taken
            dE_chk=cfg.dist(sim_pt,2);
            need_supply=(sim_O<dE_chk*cT.MO||sim_H<dE_chk*cT.MH||sim_F<dE_chk*cT.MF);
            if need_supply && sim_pt~=6 && sim_pt~=7
                % Head to nearest supply point
                dS1=cfg.dist(sim_pt,6); dS2=cfg.dist(sim_pt,7);
                if dS1<=dS2, tgt=cfg.xy(6,:); else, tgt=cfg.xy(7,:); end
            else
                tgt=to_E;
            end
            dx=tgt(1)-sim_pos(1); dy=tgt(2)-sim_pos(2);
            if abs(dx)>0, sim_pos(1)=sim_pos(1)+sign(dx);
            elseif abs(dy)>0, sim_pos(2)=sim_pos(2)+sign(dy);
            end
            sim_O=sim_O-ca.MO; sim_H=sim_H-ca.MH; sim_F=sim_F-ca.MF; cw=0;
            for nd=1:7
                if sim_pos(1)==cfg.xy(nd,1)&&sim_pos(2)==cfg.xy(nd,2)
                    sim_pt=nd; break;
                end
            end
        end"""

if old_move in content2:
    content2 = content2.replace(old_move, new_move)
    print('C: tail_simulate fixed - supply routing added')
else:
    print('C: old_move pattern NOT FOUND - manual check needed')
    # Print surrounding context
    idx = content2.find('4. MOVE toward E')
    if idx >= 0:
        print(f'  Found at position {idx}')
        print(content2[idx:idx+300])

with open(path2, 'w', encoding='utf-8') as f:
    f.write(content2)
print('C: rso_solver_v4.m updated')
