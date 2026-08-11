function fig_q1_optimal_path()
%% 问题一最优路径详图
close all;
figure('Position', [100, 100, 850, 800], 'Color', 'w');
hold on;

N = 10;
for i = 0:N
    plot([0 N], [i i], 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
    plot([i i], [0 N], 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
end

B = [1, 5];   E = [10, 5];
S1 = [3, 4];  S2 = [7, 6];
W1 = [2, 7];  W2 = [5, 3];  W3 = [8, 8];

path_nodes = {B, W1, S2, W3, S2, E};
path_colors = lines(length(path_nodes)-1);
for i = 1:length(path_nodes)-1
    p1 = path_nodes{i}; p2 = path_nodes{i+1};
    quiver(p1(1), p1(2), p2(1)-p1(1), p2(2)-p1(2), 0, ...
           'Color', path_colors(i,:), 'LineWidth', 3.5, 'MaxHeadSize', 0.7);
end

plot(B(1), B(2), 's', 'MarkerSize', 22, 'MarkerFaceColor', [0.2 0.7 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
plot(E(1), E(2), 'd', 'MarkerSize', 22, 'MarkerFaceColor', [1 0.4 0.4], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
plot([S1(1), S2(1)], [S1(2), S2(2)], '^', 'MarkerSize', 16, 'MarkerFaceColor', [0.3 0.6 1], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot([W1(1), W2(1), W3(1)], [W1(2), W2(2), W3(2)], 'o', 'MarkerSize', 16, 'MarkerFaceColor', [1 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

text(B(1)-0.6, B(2)+0.55, 'B(1,5)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.15 0.5 0.15]);
text(E(1)+0.5, E(2)+0.55, 'E(10,5)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.7 0.1 0.1]);
text(S1(1)-0.9, S1(2)-0.65, 'S_1(3,4)', 'FontSize', 10, 'Color', [0.1 0.3 0.7]);
text(S2(1)+0.5, S2(2)-0.65, 'S_2(7,6)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.3 0.7]);
text(W1(1)+0.5, W1(2)+0.55, 'W_1(2,7)', 'FontSize', 10, 'Color', [0.7 0.5 0]);
text(W2(1)+0.5, W2(2)-0.55, 'W_2(5,3)', 'FontSize', 10, 'Color', [0.5 0.5 0.5]);
text(W3(1)+0.5, W3(2)+0.55, 'W_3(8,8)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.7 0.5 0]);

path_labels = {'3', '6', '3', '3', '4'};
offsets = {[-0.2, 0.55], [0.55, -0.2], [0.55, 0.55], [-0.55, -0.55], [0.55, -0.55]};
for i = 1:5
    p1 = path_nodes{i}; p2 = path_nodes{i+1};
    mp = (p1 + p2) / 2;
    text(mp(1)+offsets{i}(1), mp(2)+offsets{i}(2), ['d=', path_labels{i}], ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', path_colors(i,:), ...
         'BackgroundColor', [1 1 1 0.7]);
end

text(1.3, 2.0, '最优结果: Z* = 328 (目标物资)  |  M* = 21 (剩余资金)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.97 0.97 0.85], ...
     'EdgeColor', [0.5 0.5 0.3], 'Margin', 6, 'LineWidth', 1.2);
text(1.3, 1.3, '总旅行 19 天  |  总工作 11 天', ...
     'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.97 0.97 0.85], ...
     'EdgeColor', [0.5 0.5 0.3], 'Margin', 6, 'LineWidth', 1.2);

axis([-0.8, N+0.8, -0.8, N+0.8]); axis equal; box on;
set(gca, 'XTick', 0:N, 'YTick', 0:N, 'FontSize', 9);
xlabel('x', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('y', 'FontSize', 12, 'FontWeight', 'bold');
title('问题一 最优路径示意图', 'FontSize', 15, 'FontWeight', 'bold');

saveas(gcf, 'q1_optimal_path.pdf');
fprintf('Figure 1 saved: q1_optimal_path.pdf\n');
end
