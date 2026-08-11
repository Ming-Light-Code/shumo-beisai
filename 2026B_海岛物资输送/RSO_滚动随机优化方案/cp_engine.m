function varargout = cp_engine(action, varargin)
switch action
    case 'config',       varargout{1} = get_config();
    case 'cons',         varargout{1} = get_cons(varargin{:});
    case 'weather',      varargout{1} = gen_weather(varargin{:});
    case 'skeletons',    [varargout{1},varargout{2}] = enum_skeletons();
    case 'skeleton_info',varargout{1} = skeleton_info(varargin{:});
    otherwise, error('cp_engine: unknown action "%s"', action);
end
end
function cfg = get_config()
    cfg.MAX_DAYS = 90;  cfg.MAX_LOAD = 400;  cfg.p_normal = 0.8;
    cfg.xy = [1 15; 30 15; 6 21; 15 9; 24 24; 12 16; 21 16];
    cfg.names = {'B','E','W1','W2','W3','S1','S2'};
    cfg.WY = [20 15 28];  cfg.WM = [4 5 3];
    cfg.init = struct('O',100,'H',150,'F',100,'M',750,'Z',200);
    cfg.inter = [3 4 5 6 7];
    cfg.MC_N = 300;  cfg.SAFETY_ALPHA = 0.4;  cfg.WORK_BUFFER = 5;
    n = size(cfg.xy,1);  cfg.dist = zeros(n);
    for i=1:n, for j=1:n
        cfg.dist(i,j)=abs(cfg.xy(i,1)-cfg.xy(j,1))+abs(cfg.xy(i,2)-cfg.xy(j,2));
    end; end
end
function cons = get_cons(mode)
    switch mode
        case 'normal'
            cons.MO=2;cons.MH=3;cons.MF=2;cons.PO=1;cons.PH=1;cons.PF=1;cons.WO=5;cons.WH=4;cons.WF=3;
        case 'thunder'
            cons.MO=8;cons.MH=4;cons.MF=3;cons.PO=3;cons.PH=3;cons.PF=2;cons.WO=8;cons.WH=6;cons.WF=6;
        case 'expected'
            cons.MO=3.2;cons.MH=3.2;cons.MF=2.2;cons.PO=1.4;cons.PH=1.4;cons.PF=1.2;cons.WO=5.6;cons.WH=4.4;cons.WF=3.6;
    end
    cons.pO=2; cons.pH=1; cons.pF=2;
end
function ws = gen_weather(n, pn)
    if nargin<2, pn=0.8; end
    ws = repmat('N',1,n);  ws(rand(1,n)>pn)='T';
end
function [skels, n_explored] = enum_skeletons()
    cfg = get_config();  skels = {};  n_explored = 0;
    dfs([1], 0);
    function dfs(path, tsf)
        n_explored = n_explored + 1;
        lp = path(end);  dE = cfg.dist(lp, 2);
        if lp ~= 2 && tsf + dE <= cfg.MAX_DAYS
            sk = [path, 2];  tt = 0;
            for k = 1:length(sk)-1, tt = tt + cfg.dist(sk(k), sk(k+1)); end
            if tt <= cfg.MAX_DAYS, skels{end+1} = sk; end
        end
        if lp == 2, return; end
        for ni = 1:length(cfg.inter)
            np = cfg.inter(ni);
            if np==lp, continue; end
            d = cfg.dist(lp, np);
            if tsf+d>cfg.MAX_DAYS, continue; end
            if tsf+d+cfg.dist(np,2)>cfg.MAX_DAYS, continue; end
            if (lp==6||lp==7) && (np==6||np==7), continue; end
            dfs([path, np], tsf + d);
        end
    end
end
function info = skeleton_info(skel)
    cfg = get_config();
    info.skel = skel;
    info.travel_days = 0;
    info.work_points = [];  info.supply_points = [];
    for k = 1:length(skel)-1
        info.travel_days = info.travel_days + cfg.dist(skel(k), skel(k+1));
        np = skel(k+1);
        if np>=3 && np<=5, info.work_points(end+1) = np;
        elseif np==6||np==7, info.supply_points(end+1) = np; end
    end
    info.remaining_for_work = cfg.MAX_DAYS - info.travel_days;
end
