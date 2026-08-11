
const MD=30,ML=120;
const AXY=[[1,5],[10,5],[2,7],[5,3],[8,8],[3,4],[7,6]];
const NM=['B','E','W1','W2','W3','S1','S2'];
const WY=[20,15,28],WM=[4,5,3];
const IIDX=[2,3,4,5,6];
const DST=(()=>{const n=7,d=Array(n).fill().map(()=>Array(n));for(let i=0;i<n;i++)for(let j=0;j<n;j++)d[i][j]=Math.abs(AXY[i][0]-AXY[j][0])+Math.abs(AXY[i][1]-AXY[j][1]);return d;})();
const co={MO:2,MH:3,MF:2,PO:1,PH:1,PF:1,WO:5,WH:4,WF:3,pO:2,pH:1,pF:2};

function mwp(mc,rem){let b=0;for(let k=1;k<=rem+1;k++){const s=k*mc+(k-1);if(s>rem)break;b=k*mc;const sl=rem-s;if(sl>=1)b=Math.max(b,k*mc+Math.min(mc,sl-1));}return Math.max(b,Math.min(mc,rem));}
function epc(n,m){const c=[];const u=new Array(n).fill(0);function r(p,rem){if(p===n-1){u[p]=rem;c.push([...u]);return;}for(let i=0;i<=rem;i++){u[p]=i;r(p+1,rem-i);}}r(0,m);return c;}

let sc=0,sf=0;
function sim(pid,m,tt,df,wd,pk,O,H,F,M,Z){
sc++;const T=tt+(wd||[]).reduce((a,b)=>a+b,0)+(pk||[]).reduce((a,b)=>a+b,0)+200;
const cO=Array(T).fill(0),cH=Array(T).fill(0),cF=Array(T).fill(0),zG=Array(T).fill(0),iS=Array(T).fill(false);
const wa=[],ww=[];for(let i=1;i<pid.length;i++){const pt=pid[i];if(pt>=2&&pt<=4){wa.push(i);ww.push(pt-2);}}
let dy=0;
for(let k=0;k<=m;k++){
for(let pd=0;pd<(pk[k]||0);pd++){cO[dy]=co.PO;cH[dy]=co.PH;cF[dy]=co.PF;dy++;}
const d=df[pid[k]][pid[k+1]];
for(let dd=0;dd<d;dd++){cO[dy]=co.MO;cH[dy]=co.MH;cF[dy]=co.MF;if(dd===d-1&&(pid[k+1]===5||pid[k+1]===6))iS[dy]=true;dy++;}
const wk=wa.indexOf(k+1);
if(wk>=0&&wd&&wd.length>0&&wd[wk]>0){const wmVal=WM[ww[wk]],yld=WY[ww[wk]];let rv=wd[wk];while(rv>0){const ch=Math.min(rv,wmVal);for(let w=0;w<ch;w++){cO[dy]=co.WO;cH[dy]=co.WH;cF[dy]=co.WF;zG[dy]=yld;dy++;}rv-=ch;if(rv>0){cO[dy]=co.PO;cH[dy]=co.PH;cF[dy]=co.PF;dy++;}}}
}
const Ta=dy;
for(let t=0;t<Ta;t++){O-=cO[t];H-=cH[t];F-=cF[t];Z+=zG[t];if(O<0||H<0||F<0)return[false,Z,M];if(O+H+F>ML+1e-9)return[false,Z,M];
if(iS[t]){let ns=Ta+1;for(let tt=t+1;tt<Ta;tt++){if(iS[tt]){ns=tt;break;}}let nO=0,nH=0,nF=0;for(let tt=t+1;tt<=ns;tt++){if(tt>=Ta)break;nO+=cO[tt];nH+=cH[tt];nF+=cF[tt];}const sp=ML-(O+H+F);const bO=Math.max(0,nO-O),bH=Math.max(0,nH-H),bF=Math.max(0,nF-F);if(bO+bH+bF>sp)return[false,Z,M];if(ns>Ta&&(O+bO<nO||H+bH<nH||F+bF<nF))return[false,Z,M];const cost=bO*co.pO+bH*co.pH+bF*co.pF;if(cost>M)return[false,Z,M];O+=bO;H+=bH;F+=bF;M-=cost;}}
sf++;return[true,Z,M];
}

function evl(fp,m,tt,pr,df,wd,bZ,bM,bP,bWD,bPS,O,H,F,M,Z){
const ns=m+1;const pc=epc(ns+1,pr);
for(const c of pc){const ps=c.slice(0,ns);const[ok,Zo,Mo]=sim(fp,m,tt,df,wd,ps,O,H,F,M,Z);if(ok&&(Zo>bZ||(Zo===bZ&&Mo>bM))){bZ=Zo;bM=Mo;bP=[...fp];bWD=wd?[...wd]:[];bPS=[...ps];}}
return[bZ,bM,bP,bWD,bPS];
}

let tc=0;
function cps(path,tsf,wa,ww,bZ,bM,bP,bWD,bPS,df,O,H,F,M,Z,IZ,dp){
tc++;
const ps=path.join(',');
if(dp>20)return[bZ,bM,bP,bWD,bPS];
const lp=path[path.length-1];
if(lp!==1){const dE=df[lp][1],rem=MD-tsf;if(rem<dE)return[bZ,bM,bP,bWD,bPS];const mw=mwp(3,rem-dE);if(IZ+mw*28<=bZ&&bZ>-Infinity)return[bZ,bM,bP,bWD,bPS];}
const dE2=df[lp][1];
if(tsf+dE2<=MD){const fp=[...path,1],m=fp.length-2;let tt=0;for(let k=0;k<=m;k++)tt+=df[fp[k]][fp[k+1]];const rd=MD-tt,nw=wa.length;
if(nw===0){[bZ,bM,bP,bWD,bPS]=evl(fp,m,tt,rd,df,[],bZ,bM,bP,bWD,bPS,O,H,F,M,Z);}
else{const mwk=wa.map((_,j)=>mwp(WM[ww[j]],rd));const sz=mwk.map(v=>v+1);const nc=sz.reduce((a,b)=>a*b,1);
for(let ci=0;ci<nc;ci++){const wd=new Array(nw).fill(0);let t2=ci;for(let j=nw-1;j>=0;j--){wd[j]=t2%sz[j];t2=Math.floor(t2/sz[j]);}let ts=0;for(let j=0;j<nw;j++){if(wd[j]>0)ts+=wd[j]+Math.max(0,Math.ceil(wd[j]/WM[ww[j]])-1);}if(tt+ts>MD)continue;const pr=MD-tt-ts;[bZ,bM,bP,bWD,bPS]=evl(fp,m,tt,pr,df,wd,bZ,bM,bP,bWD,bPS,O,H,F,M,Z);}}}
for(const np of IIDX){if(np===lp)continue;const d=df[lp][np];if(tsf+d>MD)continue;const dE3=df[np][1];if(tsf+d+dE3>MD)continue;const np2=[...path,np],nt=tsf+d,nwa=[...wa],nww=[...ww];if(np>=2&&np<=4){nwa.push(np2.length-1);nww.push(np-2);}[bZ,bM,bP,bWD,bPS]=cps(np2,nt,nwa,nww,bZ,bM,bP,bWD,bPS,df,O,H,F,M,Z,IZ,dp+1);}
return[bZ,bM,bP,bWD,bPS];
}

// Direct sim test for Path B
const tp=[0,2,6,4,6,1],tw=[3,6],tk=[0,0,0,0,0];
const[r,Zo,Mo]=sim(tp,4,19,DST,tw,tk,35,45,30,240,100);
console.log('Path B direct sim: ok='+r+' Z='+Zo+' M='+Mo);

// CP Search
sc=0;sf=0;tc=0;
let bZ=-Infinity,bM=-Infinity,bP=[0,1],bWD=[],bPS=[];
[bZ,bM,bP,bWD,bPS]=cps([0],0,[],[],bZ,bM,bP,bWD,bPS,DST,35,45,30,240,100,100,0);

console.log('=== RESULT ===');
console.log('Z='+bZ+' M='+bM);
console.log('Path: '+bP.map(i=>NM[i]).join(' -> '));
console.log('Work: '+(bWD&&bWD.length?bWD.join(','):'none'));
console.log('cps calls: '+tc+' sim calls: '+sc+' feasible: '+sf);
console.log(bZ===328?'MATCH!':'GOT '+bZ+', expected 328');

