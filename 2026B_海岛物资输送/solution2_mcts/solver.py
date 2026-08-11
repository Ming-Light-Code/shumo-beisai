"""Solution 2: Monte Carlo Tree Search"""
import sys, os, csv, json, random, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from environment import (Environment, ShipState, GRID_SIZE, START, END,
                         SUPPLY, WORK, WORK_YIELD, WORK_MAX_CONSECUTIVE,
                         LOAD_CAP, TIME_LIMIT, PRICE_O, PRICE_H, PRICE_F,
                         P_NORMAL, P_STORM, CONSUME)

MOVE_DIRS = {"move_up":(-1,0),"move_down":(1,0),"move_left":(0,-1),"move_right":(0,1)}

def _md(p1,p2): return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

class MCTSNode:
    __slots__ = ["state","parent","action","children","visits","value","untried"]
    def __init__(self, state, parent=None, action=None):
        self.state=state;self.parent=parent;self.action=action
        self.children=[];self.visits=0;self.value=0.0;self.untried=None
    def ucb1(self, C=1.414):
        if self.visits==0: return float("inf")
        return self.value/self.visits + C*math.sqrt(math.log(self.parent.visits)/self.visits)

class MCTSSolver:
    def __init__(self, env, iterations=150, max_depth=30):
        self.env=env;self.iterations=iterations;self.max_depth=max_depth

    def _get_actions(self, state):
        x,y=state.x,state.y;wd=state.work_days
        acts=[]
        for an,(dx,dy) in MOVE_DIRS.items():
            nx,ny=x+dx,y+dy
            if 1<=nx<=GRID_SIZE and 1<=ny<=GRID_SIZE: acts.append(an)
        acts.append("stay")
        for wn,wp in WORK.items():
            if (x,y)==wp and wd.get(wn,0)<WORK_MAX_CONSECUTIVE[wn]:
                acts.append("work");break
        sn=self.env.get_supply_name(x,y)
        if sn: acts.append("replenish")
        return acts

    def _heuristic_rollout(self, state, env_copy, rng):
        s=state.copy();depth=0
        while not s.terminated and s.day<=TIME_LIMIT and depth<self.max_depth:
            acts=self._get_actions(s);weather="normal" if rng.random()<P_NORMAL else "storm"
            chosen="stay"
            for a in acts:
                if a=="work": chosen="work";break
            if chosen=="stay":
                for a in acts:
                    if a=="replenish" and s.load<LOAD_CAP*0.5: chosen="replenish";break
            if chosen=="stay":
                best_d=999;best_a="stay"
                targets=[END]+[wp for wp in WORK.values()]
                for tx,ty in targets:
                    for a in acts:
                        if a.startswith("move_"):
                            dx,dy=MOVE_DIRS[a];nx,ny=s.x+dx,s.y+dy
                            d=_md((nx,ny),(tx,ty))
                            if d<best_d: best_d=d;best_a=a
                chosen=best_a
            repl=None
            if chosen=="replenish":
                sp=LOAD_CAP-s.load;nd=max(0,20*7-s.load);am=min(nd,sp)
                o=min(am*2//7,sp,s.M//2);h=min(am*3//7,sp-o,s.M);f=min(am*2//7,sp-o-h,s.M//2)
                repl=(max(0,o),max(0,h),max(0,f))
            s,reward,done=env_copy.step(s,chosen,weather,repl);depth+=1
            if done: break
        if s.terminated and (s.x,s.y)==END: return s.Z+s.M*1e-4
        r=TIME_LIMIT-s.day+1;de=_md(s.pos,END)
        if de>r: return -99999
        pot=s.Z;dl=r-de;cur=s.pos;used={w:False for w in WORK}
        for _ in range(3):
            bw,bs=None,-1.0
            for wn,wp in WORK.items():
                if used[wn]: continue
                d=_md(cur,wp)
                if d<dl:
                    sc=float(WORK_YIELD[wn]*WORK_MAX_CONSECUTIVE[wn])/(d+1)
                    if sc>bs:bs=sc;bw=(wn,wp,WORK_YIELD[wn],WORK_MAX_CONSECUTIVE[wn])
            if bw:
                wn,wp,yld,md=bw;d=_md(cur,wp);wd=min(dl-d,md)
                pot+=wd*yld;dl-=(d+wd);cur=wp;used[wn]=True
            else: break
        return float(pot)+float(s.M)*1e-4

    def plan(self, root_state):
        root=MCTSNode(root_state)
        env_copy=Environment(seed=self.env.rng.randint(0,999999))
        for _ in range(self.iterations):
            node=root
            while node.untried is None and node.children:
                node=max(node.children,key=lambda n:n.ucb1())
            if node.untried is None:
                acts=self._get_actions(node.state)
                node.untried=acts[:]
            if node.untried and not node.state.terminated:
                a=node.untried.pop()
                weather="normal" if self.env.rng.random()<P_NORMAL else "storm"
                repl=None
                if a=="replenish":
                    sp=LOAD_CAP-node.state.load;nd=max(0,20*7-node.state.load);am=min(nd,sp)
                    o=min(am*2//7,sp,node.state.M//2);h=min(am*3//7,sp-o,node.state.M)
                    f=min(am*2//7,sp-o-h,node.state.M//2)
                    repl=(max(0,o),max(0,h),max(0,f))
                ns,reward,done=env_copy.step(node.state.copy(),a,weather,repl)
                child=MCTSNode(ns,node,a)
                node.children.append(child)
                val=self._heuristic_rollout(ns,env_copy,self.env.rng)
                child.visits=1;child.value=val
            else:
                child=random.choice(node.children) if node.children else node
                val=self._heuristic_rollout(child.state,env_copy,self.env.rng) if not child.state.terminated else 0
            cur=child
            while cur is not None:
                cur.visits+=1;cur.value+=val;cur=cur.parent
        if not root.children: return "stay",None
        best=max(root.children,key=lambda n:n.visits)
        if best.action=="replenish":
            sp=LOAD_CAP-root_state.load;nd=max(0,20*7-root_state.load);am=min(nd,sp)
            o=min(am*2//7,sp,root_state.M//2);h=min(am*3//7,sp-o,root_state.M)
            f=min(am*2//7,sp-o-h,root_state.M//2)
            return best.action,(max(0,o),max(0,h),max(0,f))
        return best.action,None

def main():
    env=Environment(seed=42)
    solver=MCTSSolver(env,iterations=150,max_depth=25)
    print("="*60);print("Solution 2: Monte Carlo Tree Search");print("="*60)
    state=env.reset(42);log=[];total=0
    while not state.terminated and state.day<=TIME_LIMIT:
        weather=env.sample_weather();act,repl=solver.plan(state)
        log.append(dict(day=state.day,x=state.x,y=state.y,weather=weather,
                        action=act,replenish=repl,
                        O=state.O,H=state.H,F=state.F,M=state.M,Z=state.Z))
        state,reward,done=env.step(state,act,weather,repl);total+=1
        if total%15==0 or done:
            print("  Day",state.day,"pos",state.pos,"Z",state.Z,"O",state.O,"H",state.H,"F",state.F,"M",state.M)
    success=state.terminated and (state.x,state.y)==END
    nlog=len(log)
    print("Result: Success={} Z={} M={} Day={}".format(success,state.Z,state.M,state.day))
    base=os.path.dirname(__file__)
    with open(os.path.join(base,"result_mcts.json"),"w",encoding="utf-8") as f:
        json.dump(dict(success=success,final_Z=state.Z,final_M=state.M,
                       final_day=state.day,log=log),f,ensure_ascii=False,indent=2)
    with open(os.path.join(base,"result_mcts.csv"),"w",newline="",encoding="utf-8-sig") as f:
        w=csv.writer(f)
        w.writerow(["Day","x","y","Weather","Action","Replenish","O","H","F","M","Z"])
        for r in log:
            w.writerow([r["day"],r["x"],r["y"],r["weather"],r["action"],
                       str(r["replenish"]),r["O"],r["H"],r["F"],r["M"],r["Z"]])
    print("Saved result_mcts.json and result_mcts.csv ({} days)".format(nlog))

if __name__=="__main__": main()