function test_nodes()
cfg = params_q3_v2();
for i = 1:cfg.nN
    pos = cfg.xy(i,:);
    [id, found] = which_node_test(pos, cfg);
    fprintf('Pos (%d,%d) -> node=%d (%s), found=%d\n', pos(1), pos(2), id, cfg.names{id}, found);
end
fprintf('\nN_B=%d, N_S=%s\n', cfg.N_B, mat2str(cfg.N_S));
end

function [node_id, found] = which_node_test(pos, cfg)
    found = false;  node_id = 0;
    for i = 1:cfg.nN
        if isequal(pos, cfg.xy(i,:))
            node_id = i;  found = true;  return;
        end
    end
end
