function fig_q2_extreme_path()
%% 问题二极端天气路径对比图
close all;
figure('Position', [80, 80, 1400, 620], 'Color', 'w');

N = 10;
B = [1, 5];   E = [10, 5];
S1 = [3, 4];  S2 = [7, 6];
W1 = [2, 7];  W2 = [5, 3];  W3 = [8, 8];

% ====== 左子图: 全正常天气 ======
subplot(1,2,1); hold on;
for i = 0:N
    plot([0 N], [i i], 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
    plot([i i], [0 N], 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
end
plot(B(1), B(2), 's', 'MarkerSize', 18, 'MarkerFaceColor', [0.2 0.7 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
plot(E(1), E(2), 'd', 'MarkerSize', 18, 'MarkerFaceColor', [1 0.4 0.4], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
plot(S1(1), S1(2), '^', 'MarkerSize', 14, 'MarkerFaceColor', [0.3 0.6 1], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(S2(1), S2(2), '^', 'MarkerSize', 14, 'MarkerFaceColor', [0.3 0.6 1], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(W1(1), W1(2), 'o', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(W2(1), W2(2), 'o', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(W3(1), W3(2), 'o', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

path_normal = {B, W1, S2, W3, S2, E};
for i = 1:length(path_normal)-1
    p1 = path_normal{i}; p2 = path_normal{i+1};
    quiver(p1(1), p1(2), p2(1)-p1(1), p2(2)-p1(2), 0, ...
           'Color', [0.1 0.4 0.8], 'LineWidth', 3.2, 'MaxHeadSize', 0.7);
end

text(B(1)-0.6, B(2)+0.4, 'B', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.15 0.5 0.15]);
text(E(1)+0.4, E(2)+0.4, 'E', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.7 0.1 0.1]);
text(S1(1)-0.8, S1(2)-0.5, 'S_1', 'FontSize', 10, 'Color', [0.1 0.3 0.7]);
text(S2(1)+0.4, S2(2)-0.5, 'S_2', 'FontSize', 10, 'Color', [0.1 0.3 0.7]);
text(W1(1)+0.4, W1(2)+0.4, 'W_1', 'FontSize', 10, 'Color', [0.7 0.5 0]);
text(W2(1)+0.4, W2(2)-0.4, 'W_2', 'FontSize', 10, 'Color', [0.5 0.5 0.5]);
text(W3(1)+0.4, W3(2)+0.4, 'W_3', 'FontSize', 10, 'Color', [0.7 0.5 0]);

axis([-1, N+1, -1, N+1]); axis equal; box on;
set(gca, 'XTick', 0:2:N, 'YTick', 0:2:N, 'FontSize', 9);
xlabel('x', 'FontSize', 12); ylabel('y', 'FontSize', 12);
title('(a) 全正常天气 (正常 = N)', 'FontSize', 12, 'FontWeight', 'bold');
text(1.5, 1.0, 'Z*=328  M*=21  B->W_1->S_2->W_3->S_2->E', ...
     'FontSize', 9, 'BackgroundColor', [0.9 0.95 1], 'EdgeColor', [0.1 0.4 0.8], 'Margin', 4);

% ====== 右子图: 全雷暴极端天气 ======
subplot(1,2,2); hold on;
for i = 0:N
    plot([0 N], [i i], 'Color', [0.78 0.78 0.78], 'LineWidth', 0.5);
    plot([i i], [0 N], 'Color', [0.78 0.78 0.78], 'LineWidth', 0.5);
end

fill([0 N N 0], [0 0 N N], [1 0.92 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.4);

plot(B(1), B(2), 's', 'MarkerSize', 18, 'MarkerFaceColor', [0.2 0.7 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
plot(E(1), E(2), 'd', 'MarkerSize', 18, 'MarkerFaceColor', [1 0.4 0.4], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
plot(S1(1), S1(2), '^', 'MarkerSize', 14, 'MarkerFaceColor', [0.3 0.6 1], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(S2(1), S2(2), '^', 'MarkerSize', 14, 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'k', 'LineWidth', 1);
plot(W1(1), W1(2), 'o', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(W2(1), W2(2), 'o', 'MarkerSize', 14, 'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerEdgeColor', 'k', 'LineWidth', 1);
plot(W3(1), W3(2), 'o', 'MarkerSize', 14, 'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerEdgeColor', 'k', 'LineWidth', 1);

park_point = [2, 6];
quiver(B(1), B(2), park_point(1)-B(1), park_point(2)-B(2), 0, ...
       'Color', [0.85 0.15 0.15], 'LineWidth', 3.2, 'MaxHeadSize', 0.7);
quiver(park_point(1), park_point(2), S1(1)-park_point(1), S1(2)-park_point(2), 0, ...
       'Color', [0.85 0.15 0.15], 'LineWidth', 3.2, 'MaxHeadSize', 0.7);
quiver(S1(1), S1(2), E(1)-S1(1), E(2)-S1(2), 0, ...
       'Color', [0.85 0.15 0.15], 'LineWidth', 3.2, 'MaxHeadSize', 0.7);

plot(park_point(1), park_point(2), 'x', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', [0.8 0.1 0.1]);
text(park_point(1)+0.6, park_point(2)-0.2, '停泊等待', 'FontSize', 9, 'Color', [0.8 0.1 0.1], 'FontWeight', 'bold');

text(B(1)-0.6, B(2)+0.4, 'B', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.15 0.5 0.15]);
text(E(1)+0.4, E(2)+0.4, 'E', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.7 0.1 0.1]);
text(S1(1)-0.8, S1(2)-0.5, 'S_1', 'FontSize', 10, 'Color', [0.1 0.3 0.7]);
text(S2(1)+0.4, S2(2)-0.5, 'S_2', 'FontSize', 10, 'Color', [0.5 0.5 0.5]);
text(W1(1)+0.4, W1(2)+0.4, 'W_1', 'FontSize', 10, 'Color', [0.7 0.5 0]);

axis([-1, N+1, -1, N+1]); axis equal; box on;
set(gca, 'XTick', 0:2:N, 'YTick', 0:2:N, 'FontSize', 9);
xlabel('x', 'FontSize', 12); ylabel('y', 'FontSize', 12);
title('(b) 全雷暴极端天气 (天气 = T)', 'FontSize', 12, 'FontWeight', 'bold');
text(1.5, 1.0, 'Z*=100  M*=116  B->停泊->S_1->E', ...
     'FontSize', 9, 'BackgroundColor', [1 0.92 0.92], 'EdgeColor', [0.85 0.2 0.2], 'Margin', 4);
text(8, 9, ' 仅 S_1, W_1 可达 ', 'FontSize', 8, 'Color', [0.5 0.5 0.5], ...
     'EdgeColor', [0.5 0.5 0.5], 'Margin', 2);

sgtitle('问题二 正常天气与极端雷暴天气路径对比', 'FontSize', 15, 'FontWeight', 'bold');

saveas(gcf, 'q2_extreme_path.pdf');
fprintf('Figure 2 saved: q2_extreme_path.pdf\n');
end
