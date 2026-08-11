function fig_ub_pruning()
%% 上界剪枝搜索空间压缩示意图
close all;
figure('Position', [60, 60, 1100, 680], 'Color', 'w');

% ====== 左图: 搜索树示意图 ======
subplot(1,2,1); hold on; axis off;
title('(a) 路径骨架搜索树与上界剪枝', 'FontSize', 13, 'FontWeight', 'bold');

root = [0, 8];
plot(root(1), root(2), 's', 'MarkerSize', 14, 'MarkerFaceColor', [0.2 0.7 0.2], 'MarkerEdgeColor', 'k');
text(root(1)-0.3, root(2)+0.4, 'B', 'FontSize', 11, 'FontWeight', 'bold');

layer1_x = [-4, -2, 0, 2, 4]; layer1_y = 6;
layer1_cols = {[0 0.6 0], [0.5 0.5 0.5], [0.5 0.5 0.5], [0.5 0.5 0.5], [0 0.6 0]};
for i = 1:5
    plot([root(1), layer1_x(i)], [root(2), layer1_y], '-', 'Color', layer1_cols{i}, 'LineWidth', 2);
    ms = 10; if i == 1 || i == 5, ms = 12; end
    plot(layer1_x(i), layer1_y, 'o', 'MarkerSize', ms, 'MarkerFaceColor', [1 0.8 0.2], 'MarkerEdgeColor', 'k');
end
text(-4, 6.4, 'W_1', 'FontSize', 10, 'HorizontalAlignment', 'center');
text(-2, 6.4, 'W_2', 'FontSize', 10, 'HorizontalAlignment', 'center', 'Color', [0.5 0.5 0.5]);
text(0, 6.4, 'W_3', 'FontSize', 10, 'HorizontalAlignment', 'center', 'Color', [0.5 0.5 0.5]);
text(2, 6.4, 'S_1', 'FontSize', 10, 'HorizontalAlignment', 'center', 'Color', [0.5 0.5 0.5]);
text(4, 6.4, 'S_2', 'FontSize', 10, 'HorizontalAlignment', 'center');

lx = -4 + [-1.5, 0, 1.5]; ly = 4;
for i = 1:3
    plot([layer1_x(1), lx(i)], [layer1_y, ly], '-', 'Color', [0 0.6 0], 'LineWidth', 1.5);
end
text(lx(1), ly-0.4, 'S_1', 'FontSize', 9, 'HorizontalAlignment', 'center');
text(lx(2), ly-0.4, 'S_2', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0 0.4 0], 'HorizontalAlignment', 'center');
text(lx(3), ly-0.4, 'E', 'FontSize', 9, 'HorizontalAlignment', 'center');

lx2 = lx(2) + [-1, 0, 1]; ly2 = 2;
for i = 1:3
    plot([lx(2), lx2(i)], [ly, ly2], '-', 'Color', [0 0.4 0], 'LineWidth', 1.5);
end
text(lx2(1), ly2-0.4, 'W_2', 'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', [0.5 0.5 0.5]);
text(lx2(2), ly2-0.4, 'W_3', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0 0.4 0], 'HorizontalAlignment', 'center');
text(lx2(3), ly2-0.4, 'E', 'FontSize', 9, 'HorizontalAlignment', 'center');

ef_x = lx2(2); ef_y = 0.4;
plot([lx2(2), ef_x], [ly2, ef_y], '-', 'Color', [0 0.4 0], 'LineWidth', 2);
plot(ef_x, ef_y, 'd', 'MarkerSize', 12, 'MarkerFaceColor', [1 0.4 0.4], 'MarkerEdgeColor', 'k');
text(ef_x, ef_y-0.4, 'E', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% 剪枝标记
px = [0, 2]; py = [4, 4];
for i = 1:2
    plot(px(i), py(i), 'x', 'MarkerSize', 12, 'LineWidth', 2.5, 'Color', 'r');
end
text(px(1)+0.5, py(1)+0.3, 'x 剪枝', 'FontSize', 9, 'Color', 'r');

text(5.5, 7, 'UB 剪枝条件:', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.3 0.3 0.3]);
text(5.5, 6.2, 'UB_Z(p,t) = Z_t + w_max(D) * Y_3', 'FontSize', 9, 'Color', [0.3 0.3 0.3]);
text(5.5, 5.5, '若 UB_Z <= Z* 则剪除子树', 'FontSize', 9, 'Color', [0.3 0.3 0.3]);
xlim([-6, 7]); ylim([-0.8, 9.2]);

% ====== 右图: 空间压缩效果 ======
subplot(1,2,2); hold on;
log_vals = [log10(5^30), 5, 2.3];
bar(1, log_vals(1), 0.5, 'FaceColor', [1 0.5 0.5], 'EdgeColor', 'k');
bar(2, log_vals(2), 0.5, 'FaceColor', [1 0.85 0.5], 'EdgeColor', 'k');
bar(3, log_vals(3), 0.5, 'FaceColor', [0.5 0.9 0.5], 'EdgeColor', 'k');

set(gca, 'XTick', 1:3, 'XTickLabel', {'原始搜索空间', 'UB剪枝后(C层)', '最终枚举节点'}, 'FontSize', 10);
set(gca, 'YTick', 0:5:20);
yticklabels({'10^0', '10^5', '10^{10}', '10^{15}', '10^{20}'});
ylabel('搜索空间规模 (log10)', 'FontSize', 11, 'FontWeight', 'bold');
title('(b) 搜索空间压缩效果', 'FontSize', 13, 'FontWeight', 'bold');

text(1, log_vals(1)+1, '5^{30} ~= 9.3 x 10^{20}', 'FontSize', 10, 'FontWeight', 'bold', ...
     'Color', [0.7 0.1 0.1], 'HorizontalAlignment', 'center');
text(2, log_vals(2)+1, '~= 5 x 10^4', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(3, log_vals(3)+1, '~= 200', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.5 0.1], 'HorizontalAlignment', 'center');
text(1.5, 18.5, '| 剪枝压缩率 ~= 99.9999999% |', 'FontSize', 11, 'Color', [0.6 0.1 0.1], ...
     'HorizontalAlignment', 'center', 'BackgroundColor', [1 0.9 0.9], 'Margin', 5);

grid on; box on; ylim([0, 22]);

sgtitle('问题一  上界剪枝 (UB Pruning) 策略与搜索空间压缩', 'FontSize', 14, 'FontWeight', 'bold');

saveas(gcf, 'q_ub_pruning.pdf');
fprintf('Figure 5 saved: q_ub_pruning.pdf\n');
end
