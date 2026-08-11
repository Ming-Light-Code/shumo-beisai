function varargout = mdp_solver(action, varargin)
% MDP_SOLVER  Task3 MDP Engine (Value Iteration on Aggregated State Space)
% States: (node_id, weather_today, resource_level, day_bucket)
%   node_id: 1-7 (B,E,W1,W2,W3,S1,S2)
%   weather_today: 1=Normal, 2=Thunder
%   resource_level: 1=critical, 2=low, 3=normal
%   day_bucket: 1=early(1-30), 2=mid(31-60), 3=late(61-90)
% Total states: 7x2x3x3 = 126
% Actions: 1=Move-to-next-node, 2=Park-one-day, 3=Work(if-at-W),
%          4=Supply(if-at-S)

switch action
    case 'solve',    varargout{1} = solve_MDP(varargin{:});
    case 'policy',   [varargout{1},varargout{2},varargout{3}] = get_policy(varargin{:});
    case 'config',   varargout{1} = get_mdp_config();
    otherwise, error('mdp_solver: unknown action');
end
end

function cfg = get_mdp_config()
    cfg = cp_engine_v2('config');
    cfg.pN = 0.8;
    cfg.pT = 0.2;
    cfg.gamma = 1.0;  % undiscounted (finite horizon)
    cfg.max_iter = 200;
end

% ===== Main MDP Solver =====
function V = solve_MDP()
    cfg = get_mdp_config();
    nNodes = 7; nWeather = 2; nRes = 3; nDay = 3;
    nStates = nNodes * nWeather * nRes * nDay;
    
    % Action space: 1=move-to-next, 2=park, 3=work, 4=supply
    nActions = 4;
    
    % Initialize value function (optimistic: use max possible Z)
    V = zeros(nNodes, nWeather, nRes, nDay);
    % Terminal state: at E with any resources, V = current_Z (approximate)
    for w=1:nWeather, for r=1:nRes, for d=1:nDay
        V(2,w,r,d) = 500;  % optimistic terminal value
    end; end; end
    
    % Precompute transitions
    fprintf('MDP: %d states, running value iteration...\n', nStates);
    
    for iter = 1:cfg.max_iter
        delta = 0;
        V_old = V;
        
        for node = 1:nNodes
            if node == 2, continue; end  % E is terminal
            
            for w = 1:nWeather
                for r = 1:nRes
                    for d = 1:nDay
                        best_val = -inf;
                        
                        % Action 1: Move toward best next node
                        val_move = eval_move(node, w, r, d, V_old, cfg);
                        best_val = max(best_val, val_move);
                        
                        % Action 2: Park one day
                        val_park = eval_park(node, w, r, d, V_old, cfg);
                        best_val = max(best_val, val_park);
                        
                        % Action 3: Work (only at W nodes)
                        if node >= 3 && node <= 5
                            val_work = eval_work(node, w, r, d, V_old, cfg);
                            best_val = max(best_val, val_work);
                        end
                        
                        % Action 4: Supply (only at S nodes)
                        if node == 6 || node == 7
                            val_supply = eval_supply(node, w, r, d, V_old, cfg);
                            best_val = max(best_val, val_supply);
                        end
                        
                        V(node,w,r,d) = best_val;
                        delta = max(delta, abs(V(node,w,r,d) - V_old(node,w,r,d)));
                    end
                end
            end
        end
        
        if mod(iter,20)==0
            fprintf('  Iter %d, delta=%.2f\n', iter, delta);
        end
        if delta < 1e-3, fprintf('  Converged at iter %d\n', iter); break; end
    end
    fprintf('MDP solved.\n');
end

% ===== Evaluate Move Action =====
function val = eval_move(node, w, r, d, V, cfg)
    % Find best next node to move to
    best_val = -inf;
    for next = cfg.inter
        dist = cfg.dist(node, next);
        % Check if feasible with resource level
        if ~can_move(node, next, r, w, d, cfg), continue; end
        
        % Expected value after moving: weather changes
        next_w_n = 1; next_w_t = 2;
        next_r = resource_after_move(node, next, r, w, cfg);
        next_d = day_after_move(d, dist, cfg);
        
        if next_r == 0 || next_d == 0
            continue;  % infeasible
        end
        
        exp_val = cfg.pN * V(next, next_w_n, next_r, next_d) + ...
                  cfg.pT * V(next, next_w_t, next_r, next_d);
        
        % Subtract resource cost from value
        [cost_O, cost_H, cost_F] = move_cost(dist, w, cfg);
        exp_val = exp_val - cost_O * 0.1 - cost_H * 0.05 - cost_F * 0.1;
        
        best_val = max(best_val, exp_val);
    end
    val = best_val;
end

% ===== Evaluate Park Action =====
function val = eval_park(node, w, r, d, V, cfg)
    next_w_n = 1; next_w_t = 2;
    next_r = resource_after_park(r, w, cfg);
    next_d = day_after_move(d, 1, cfg);
    
    if next_r == 0 || next_d == 0
        val = -inf; return;
    end
    
    val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
          cfg.pT * V(node, next_w_t, next_r, next_d);
    
    % Cost of parking
    if w == 1  % normal
        val = val - 1*0.1 - 1*0.05 - 1*0.1;
    else  % thunder
        val = val - 3*0.1 - 3*0.05 - 2*0.1;
    end
end

% ===== Evaluate Work Action =====
function val = eval_work(node, w, r, d, V, cfg)
    wi = node - 2;  % work index 1..3
    yield = cfg.WY(wi);
    
    next_w_n = 1; next_w_t = 2;
    next_r = resource_after_work(r, w, cfg);
    next_d = day_after_move(d, 1, cfg);
    
    if next_r == 0 || next_d == 0
        val = -inf; return;
    end
    
    exp_val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
              cfg.pT * V(node, next_w_t, next_r, next_d);
    
    % Add yield, subtract resource cost
    if w == 1
        val = yield + exp_val - 5*0.1 - 4*0.05 - 3*0.1;
    else
        val = yield + exp_val - 8*0.1 - 6*0.05 - 6*0.1;
    end
end

% ===== Evaluate Supply Action =====
function val = eval_supply(node, w, r, d, V, cfg)
    % Supply improves resource level
    next_r = 3;  % goes to normal after supply
    next_w_n = 1; next_w_t = 2;
    next_d = day_after_move(d, 1, cfg);
    
    if next_d == 0
        val = -inf; return;
    end
    
    exp_val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
              cfg.pT * V(node, next_w_t, next_r, next_d);
    
    % Cost of supply day (park consumption + supply cost)
    if w == 1
        val = exp_val - 1*0.1 - 1*0.05 - 1*0.1 - 50*0.01;
    else
        val = exp_val - 3*0.1 - 3*0.05 - 2*0.1 - 50*0.01;
    end
end

% ===== Helper Functions =====
function ok = can_move(node, next, r, w, d, cfg)
    dist = cfg.dist(node, next);
    day_idx = (d-1)*30 + 1 + dist;
    ok = (dist > 0) && (day_idx <= 90) && (r > 1 || (r==1 && dist < 5));
end

function next_r = resource_after_move(node, next, r, w, cfg)
    dist = cfg.dist(node, next);
    % Rough check: can resources sustain the move?
    if r == 3, next_r = max(1, r - (w==2));  % normal->ok, or normal->low if thunder
    elseif r == 2, next_r = max(1, r - 1);
    else, next_r = 1;
    end
end

function next_d = day_after_move(d, dist, cfg)
    day_idx = (d-1)*30 + 1 + dist;
    if day_idx > 90, next_d = 0;
    elseif day_idx <= 30, next_d = 1;
    elseif day_idx <= 60, next_d = 2;
    else, next_d = 3;
    end
end

function next_r = resource_after_park(r, w, cfg)
    next_r = r;  % parking doesn't change resource level much
end

function next_r = resource_after_work(r, w, cfg)
    if w == 2, next_r = max(1, r-1);  % thunder work depletes
    else, next_r = r;
    end
end

function [cO, cH, cF] = move_cost(dist, w, cfg)
    if w == 1
        cO = dist * 2; cH = dist * 3; cF = dist * 2;
    else
        cO = dist * 8; cH = dist * 4; cF = dist * 3;
    end
end

% ===== Get Policy for a State =====
function [best_action, best_target, best_val] = get_policy(node, w, r, d, V, cfg)
    if nargin < 6, cfg = get_mdp_config(); end
    
    best_val = -inf;
    best_action = 2;  % default: park
    best_target = node;
    
    % Move
    for next = cfg.inter
        if ~can_move(node, next, r, w, d, cfg), continue; end
        next_w_n = 1; next_w_t = 2;
        next_r = resource_after_move(node, next, r, w, cfg);
        next_d = day_after_move(d, cfg.dist(node, next), cfg);
        if next_r == 0 || next_d == 0, continue; end
        exp_val = cfg.pN * V(next, next_w_n, next_r, next_d) + ...
                  cfg.pT * V(next, next_w_t, next_r, next_d);
        if exp_val > best_val
            best_val = exp_val;
            best_action = 1;
            best_target = next;
        end
    end
    
    % Park
    next_w_n = 1; next_w_t = 2;
    next_r = resource_after_park(r, w, cfg);
    next_d = day_after_move(d, 1, cfg);
    if next_r > 0 && next_d > 0
        exp_val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
                  cfg.pT * V(node, next_w_t, next_r, next_d);
        if exp_val > best_val
            best_val = exp_val;
            best_action = 2;
            best_target = node;
        end
    end
    
    % Work
    if node >= 3 && node <= 5
        next_r = resource_after_work(r, w, cfg);
        next_d = day_after_move(d, 1, cfg);
        wi = node - 2;
        if next_r > 0 && next_d > 0
            exp_val = cfg.WY(wi) + cfg.pN * V(node, next_w_n, next_r, next_d) + ...
                      cfg.pT * V(node, next_w_t, next_r, next_d);
            if exp_val > best_val
                best_val = exp_val;
                best_action = 3;
                best_target = node;
            end
        end
    end
    
    % Supply
    if node == 6 || node == 7
        next_r = 3;
        next_d = day_after_move(d, 1, cfg);
        if next_d > 0
            exp_val = cfg.pN * V(node, next_w_n, next_r, next_d) + ...
                      cfg.pT * V(node, next_w_t, next_r, next_d);
            if exp_val > best_val
                best_val = exp_val;
                best_action = 4;
                best_target = node;
            end
        end
    end
end
