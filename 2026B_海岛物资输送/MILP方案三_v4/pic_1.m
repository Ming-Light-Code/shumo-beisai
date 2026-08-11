function pic_1()
% pic_1.m
% 海岛物资输送问题 —— 任务1：海域地图（美化版）
% 10×10 整数网格，标注起点 B、终点 E、补给平台 S1/S2、作业点 W1/W2/W3

%% 初始化画布与颜色方案
figure('Color', 'w', 'Position', [100 100 840 720]);

% 配色：起点绿  终点珊瑚红  补给平台青蓝  作业点琥珀橙
cB  = [0.15 0.72 0.25];   % 起点绿
cE  = [0.85 0.20 0.25];   % 终点珊瑚红
cS  = [0.15 0.50 0.80];   % 补给平台蓝
cW  = [0.90 0.55 0.10];   % 作业点琥珀橙
cGrid = [0.78 0.82 0.88];  % 网格线灰蓝
cBG   = [0.965 0.97 0.975]; % 绘图区淡底

hold on;

%% 绘制淡色背景填充（10×10 区域）
fill([0.5 10.5 10.5 0.5], [0.5 0.5 10.5 10.5], cBG, ...
    'EdgeColor', 'none', 'HandleVisibility', 'off');

%% 轴与范围
axis([0.5 10.5 0.5 10.5]);
axis equal;
set(gca, 'XTick', 1:10, 'YTick', 1:10, ...
         'FontName', 'Helvetica', 'FontSize', 10, ...
         'LineWidth', 0.8, 'TickLength', [0.004 0.004], ...
         'XColor', [0.3 0.3 0.3], 'YColor', [0.3 0.3 0.3]);

%% 细网格（先画，置于底层）
for x = 0.5:10.5
    plot([x x], [0.5 10.5], '-', 'Color', cGrid, 'LineWidth', 0.4, ...
         'HandleVisibility', 'off');
end
for y = 0.5:10.5
    plot([0.5 10.5], [y y], '-', 'Color', cGrid, 'LineWidth', 0.4, ...
         'HandleVisibility', 'off');
end

%% 轴线加粗强调
plot([0.5 10.5], [0.5 0.5], 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot([0.5 0.5], [0.5 10.5], 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');

%% 坐标轴标签
xlabel('{\it x}  (网格横坐标)', 'FontName', 'Helvetica', ...
       'FontSize', 12, 'FontWeight', 'normal', 'Color', [0.25 0.25 0.25]);
ylabel('{\it y}  (网格纵坐标)', 'FontName', 'Helvetica', ...
       'FontSize', 12, 'FontWeight', 'normal', 'Color', [0.25 0.25 0.25]);

%% 标题
title({'海岛物资输送问题  ·  任务 1  海域地图'; ...
       '10×10 整数网格  |  起点 B(1,5) → 终点 E(10,5)'}, ...
      'FontName', 'Helvetica', 'FontSize', 13, 'FontWeight', 'bold', ...
      'Color', [0.15 0.15 0.15]);

%% 定义坐标
B  = [1, 5];
E  = [10, 5];
S1 = [3, 4];
S2 = [7, 6];
W1 = [2, 7];
W2 = [5, 3];
W3 = [8, 8];

%% 绘制点位
ms = 180;   % 散点面积基数

% 1. 补给平台（先画，使其在下层）
hS = scatter([S1(1), S2(1)], [S1(2), S2(2)], ms, ...
    's', 'filled', 'MarkerFaceColor', cS, ...
    'MarkerEdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.2);

% 2. 作业点
hW = scatter([W1(1), W2(1), W3(1)], [W1(2), W2(2), W3(2)], ms, ...
    'd', 'filled', 'MarkerFaceColor', cW, ...
    'MarkerEdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.2);

% 3. 起点
hB = scatter(B(1), B(2), ms * 1.2, ...
    'o', 'filled', 'MarkerFaceColor', cB, ...
    'MarkerEdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.5);

% 4. 终点
hE = scatter(E(1), E(2), ms * 1.4, ...
    'p', 'filled', 'MarkerFaceColor', cE, ...
    'MarkerEdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.8);

%% 文字标注（加半透明白底，不遮挡图形元素）
offset = 0.55;
txtOpts = {'HorizontalAlignment', 'center', 'FontName', 'Helvetica', ...
           'FontSize', 10, 'FontWeight', 'bold', ...
           'BackgroundColor', [1 1 1 0.75], 'Margin', 1};

text(B(1),  B(2)  - offset, 'B (1,5)',  txtOpts{:}, 'Color', cB * 0.7);
text(E(1),  E(2)  - offset, 'E (10,5)', txtOpts{:}, 'Color', cE * 0.7);
text(S1(1), S1(2) + offset, 'S_1 (3,4)',  txtOpts{:}, 'Color', cS * 0.7);
text(S2(1), S2(2) + offset, 'S_2 (7,6)',  txtOpts{:}, 'Color', cS * 0.7);
text(W1(1), W1(2) + offset, 'W_1 (2,7)',  txtOpts{:}, 'Color', cW * 0.7);
text(W2(1), W2(2) - offset, 'W_2 (5,3)',  txtOpts{:}, 'Color', cW * 0.7);
text(W3(1), W3(2) + offset, 'W_3 (8,8)',  txtOpts{:}, 'Color', cW * 0.7);

%% 终点加一个外圈光环强调
th = linspace(0, 2*pi, 60);
r = 0.55;
plot(E(1) + r*cos(th), E(2) + r*sin(th), '--', ...
     'Color', cE, 'LineWidth', 1.5, 'HandleVisibility', 'off');

%% 图例
lgd = legend([hB, hE, hS, hW], ...
    {'起点 B', '终点 E', '补给平台 S', '作业点 W'}, ...
    'Location', 'northeastoutside', 'FontName', 'Helvetica', ...
    'FontSize', 10, 'FontWeight', 'normal', 'Box', 'on');
title(lgd, '图例', 'FontName', 'Helvetica', 'FontSize', 10, ...
      'FontWeight', 'bold', 'Color', [0.25 0.25 0.25]);

%% 收尾
box off;
set(gca, 'Layer', 'top');
hold off;

end
