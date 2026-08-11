function fig_q3q4_grid()
%% 问题三/四 30x30 大网格功能点分布图
close all;
figure('Position', [100, 100, 820, 820], 'Color', 'w');
hold on;

N = 30;
for i = 0:5:N
    plot([0 N], [i i], 'Color', [0.88 0.88 0.88], 'LineWidth', 0.4);
    plot([i i], [0 N], 'Color', [0.88 0.88 0.88], 'LineWidth', 0.4);
end
plot([0 N], [15 15], ':', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);

B  = [1, 15];   E  = [30, 15];
S1 = [12, 16];  S2 = [21, 16];
W1 = [6, 21];   W2 = [15, 9];   W3 = [24, 24];

plot(B(1), B(2), 's', 'MarkerSize', 18, 'MarkerFaceColor', [0.2 0.7 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 2.5);
plot(E(1), E(2), 'd', 'MarkerSize', 18, 'MarkerFaceColor', [1 0.4 0.4], 'MarkerEdgeColor', 'k', 'LineWidth', 2.5);
plot([S1(1), S2(1)], [S1(2), S2(2)], '^', 'MarkerSize', 16, 'MarkerFaceColor', [0.3 0.6 1], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
plot([W1(1), W2(1), W3(1)], [W1(2), W2(2), W3(2)], 'o', 'MarkerSize', 15, 'MarkerFaceColor', [1 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 2);

text(B(1)-1.5, B(2)+0.8, 'B(1,15)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.15 0.5 0.15]);
text(E(1)+1, E(2)+0.8, 'E(30,15)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.7 0.1 0.1]);
text(S1(1), S1(2)+1.2, 'S_1(12,16)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.1 0.3 0.7], 'HorizontalAlignment', 'center');
text(S2(1), S2(2)+1.2, 'S_2(21,16)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.1 0.3 0.7], 'HorizontalAlignment', 'center');
text(W1(1), W1(2)+1.2, 'W_1(6,21)', 'FontSize', 11, 'Color', [0.7 0.5 0], 'HorizontalAlignment', 'center');
text(W2(1), W2(2)-1.5, 'W_2(15,9)', 'FontSize', 11, 'Color', [0.7 0.5 0], 'HorizontalAlignment', 'center');
text(W3(1), W3(2)+1.2, 'W_3(24,24)', 'FontSize', 11, 'Color', [0.7 0.5 0], 'HorizontalAlignment', 'center');

text(28, 16.5, '上半区', 'FontSize', 8, 'Color', [0.5 0.5 0.5]);
text(28, 13.5, '下半区', 'FontSize', 8, 'Color', [0.5 0.5 0.5]);
text(15, 15.8, 'd(B,E) = 29', 'FontSize', 10, 'Color', [0.3 0.3 0.3], ...
     'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.7]);

text(1, 29, ...
     '参数: 网格 30x30  |  期限 Tmax=90天  |  载重 Lmax=400', ...
     'FontSize', 9, 'BackgroundColor', [0.95 0.97 1], 'EdgeColor', [0.1 0.4 0.7], 'Margin', 4);
text(1, 27.5, ...
     'O0=100 H0=150 F0=100 M0=750 Z0=200  |  天气: p(N)=0.8, p(T)=0.2', ...
     'FontSize', 9, 'BackgroundColor', [0.95 0.97 1], 'EdgeColor', [0.1 0.4 0.7], 'Margin', 4);

legend({'B (起始岛)', 'E (终点岛)', 'S_1,S_2 (补给平台)', 'W_1,W_2,W_3 (作业点)'}, ...
       'Location', 'northwest', 'FontSize', 10);

axis([-1, N+1, -1, N+1]); axis equal; box on;
set(gca, 'XTick', 0:5:N, 'YTick', 0:5:N, 'FontSize', 9);
xlabel('x', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('y', 'FontSize', 13, 'FontWeight', 'bold');
title('问题三/四  30x30 海域功能点分布图', 'FontSize', 15, 'FontWeight', 'bold');

saveas(gcf, 'q3q4_grid_map.pdf');
fprintf('Figure 3 saved: q3q4_grid_map.pdf\n');
end
