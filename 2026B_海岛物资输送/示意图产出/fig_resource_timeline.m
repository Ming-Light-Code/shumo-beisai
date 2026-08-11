function fig_resource_timeline()
%% 资源时序变化与停泊重置策略示意图
close all;

cons_move = [2, 3, 2]; cons_park = [1, 1, 1]; cons_work = [5, 4, 3];

action = [1 1 1, 2 2 2 2, 3, 2 2 2 2, 4 4 4 4 4 4, 5, 6 6 6, 7 7 7, 4 4 4, 5, 8 8 8 8];
action = action(1:min(30, length(action)));
n_days_act = length(action);

O = zeros(1, n_days_act+1); O(1) = 35;
H = zeros(1, n_days_act+1); H(1) = 45;
F = zeros(1, n_days_act+1); F(1) = 30;
M = zeros(1, n_days_act+1); M(1) = 240;
Z = zeros(1, n_days_act+1); Z(1) = 100;

for t = 1:n_days_act
    a = action(t); dO = 0; dH = 0; dF = 0; dM = 0; dZ = 0;
    if ismember(a, [1, 4, 6, 8]), dO = cons_move(1); dH = cons_move(2); dF = cons_move(3);
    elseif a == 2, dO = cons_work(1); dH = cons_work(2); dF = cons_work(3); dZ = 20;
    elseif a == 3, dO = cons_park(1); dH = cons_park(2); dF = cons_park(3);
    elseif a == 5
        dO = cons_park(1); dH = cons_park(2); dF = cons_park(3);
        buy = [30, 35, 25];
        dO = dO - buy(1); dH = dH - buy(2); dF = dF - buy(3);
        dM = -(2*buy(1)+1*buy(2)+2*buy(3));
    elseif a == 7, dO = cons_work(1); dH = cons_work(2); dF = cons_work(3); dZ = 28;
    end
    O(t+1) = max(0, O(t)-dO); H(t+1) = max(0, H(t)-dH);
    F(t+1) = max(0, F(t)-dF); M(t+1) = M(t)+dM; Z(t+1) = Z(t)+dZ;
end

figure('Position', [80, 80, 1200, 880], 'Color', 'w');

% 子图1: O,H,F
subplot(3,2,[1 2]); hold on;
stairs(0:n_days_act, O, 'r-', 'LineWidth', 2);
stairs(0:n_days_act, H, 'b-', 'LineWidth', 2);
stairs(0:n_days_act, F, 'Color', [0 0.6 0], 'LineWidth', 2);
xline(3.5, '-.', 'Color', [0.4 0.4 0.4], 'LineWidth', 1);
xline(7.5, '--', 'Color', [0.5 0.1 0.5], 'LineWidth', 1.5);
xline(12.5, '-.', 'Color', [0.4 0.4 0.4], 'LineWidth', 1);
xline(18.5, ':', 'Color', [0 0.4 0.8], 'LineWidth', 2);
xline(22.5, '-.', 'Color', [0.4 0.4 0.4], 'LineWidth', 1);
xline(25.5, '-.', 'Color', [0.4 0.4 0.4], 'LineWidth', 1);
xline(28.5, ':', 'Color', [0 0.4 0.8], 'LineWidth', 2);
yl = ylim;
text(5.5, yl(2)*0.85, 'W1 作业区', 'FontSize', 10, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center', 'BackgroundColor', [1 0.92 0.8], 'Margin', 3);
text(24, yl(2)*0.85, 'W3 作业', 'FontSize', 10, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center', 'BackgroundColor', [1 0.92 0.8], 'Margin', 3);
text(7.5, yl(2)*0.6, '停泊重置', 'FontSize', 9, 'Color', [0.5 0.1 0.5], ...
     'HorizontalAlignment', 'center', 'BackgroundColor', 'w', 'Margin', 2);
text(18.5, yl(2)*0.5, 'S2 补给', 'FontSize', 9, 'Color', [0 0.4 0.8], ...
     'HorizontalAlignment', 'center', 'BackgroundColor', 'w', 'Margin', 2);
text(28.5, yl(2)*0.5, 'S2 补给', 'FontSize', 9, 'Color', [0 0.4 0.8], ...
     'HorizontalAlignment', 'center', 'BackgroundColor', 'w', 'Margin', 2);
xlabel('天数 t', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('资源余量', 'FontSize', 12, 'FontWeight', 'bold');
title('自持资源 O, H, F 时序变化', 'FontSize', 13, 'FontWeight', 'bold');
legend({'燃油 O', '淡水 H', '食物 F'}, 'FontSize', 10, 'Location', 'best');
grid on; box on;

% 子图2: Z
subplot(3,2,3); hold on;
stairs(0:n_days_act, Z, '-', 'Color', [0.8 0.3 0], 'LineWidth', 2.5);
xlabel('天数 t', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('目标物资 Z', 'FontSize', 11, 'FontWeight', 'bold');
title('目标物资 Z 累积过程', 'FontSize', 12, 'FontWeight', 'bold');
grid on; box on;

% 子图3: M
subplot(3,2,4); hold on;
stairs(0:n_days_act, M, '-', 'Color', [0.2 0.5 0.2], 'LineWidth', 2.5);
xlabel('天数 t', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('资金 M', 'FontSize', 11, 'FontWeight', 'bold');
title('资金 M 变化 (补给点采购消耗)', 'FontSize', 12, 'FontWeight', 'bold');
grid on; box on;

% 子图4: O+H+F
subplot(3,2,5); hold on;
load_total = O + H + F;
stairs(0:n_days_act, load_total, 'k-', 'LineWidth', 2);
yline(120, 'r--', 'LineWidth', 1.5);
text(15, 125, 'Lmax = 120', 'FontSize', 10, 'Color', 'r');
xlabel('天数 t', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('总自持资源', 'FontSize', 11, 'FontWeight', 'bold');
title('载重 O+H+F 变化 (上限 120)', 'FontSize', 12, 'FontWeight', 'bold');
grid on; box on;

% 子图5: 连续工作计数器
subplot(3,2,6); hold on;
w_cnt = zeros(1, n_days_act+1);
for t = 1:n_days_act
    if ismember(action(t), [2, 7]), w_cnt(t+1) = w_cnt(t) + 1;
    else, w_cnt(t+1) = 0; end
end
for t = 1:n_days_act
    clr = [0.3 0.5 0.3]; if w_cnt(t+1) > 3, clr = [1 0.3 0.3]; end
    stairs(t-1:t, [w_cnt(t) w_cnt(t+1)], '-', 'Color', clr, 'LineWidth', 1.5);
end
yline(4, 'r--', 'LineWidth', 1.2);
text(20, 4.3, 'W1 max = 4', 'FontSize', 9, 'Color', 'r');
yline(3, 'm--', 'LineWidth', 1.2);
text(20, 3.3, 'W3 max = 3', 'FontSize', 9, 'Color', 'm');
xlabel('天数 t', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('连续工作天数', 'FontSize', 11, 'FontWeight', 'bold');
title('连续工作计数器 (停泊重置策略)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0 5.5]); grid on; box on;

sgtitle('问题一最优方案资源时序变化  (路径: B -> W_1 -> S_2 -> W_3 -> S_2 -> E)', ...
        'FontSize', 14, 'FontWeight', 'bold');

saveas(gcf, 'q_resource_timeline.pdf');
fprintf('Figure 4 saved: q_resource_timeline.pdf\n');
end
