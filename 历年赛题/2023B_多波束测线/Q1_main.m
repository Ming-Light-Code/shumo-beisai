 %% 问题1：多波束测深覆盖宽度及重叠率计算
 % 多波束换能器开角 120°，坡度 1.5°，中心点水深 70 m
 % 重叠率定义：eta = 1 - d/W （适用于各测线位置）
 
 clear; clc;
 
 %% 参数设定
 theta = 120;                % 换能器开角（度）
 alpha = 1.5;                % 海底坡度（度）
 D0 = 70;                    % 海域中心点水深 (m)
 d = 200;                    % 相邻测线间距 (m)
 
 theta_rad = deg2rad(theta);
 alpha_rad = deg2rad(alpha);
 half_theta = theta_rad / 2;
 
 %% 测线位置
 x = [-800, -600, -400, -200, 0, 200, 400, 600, 800];
 n = length(x);
 
 %% 预分配
 D  = zeros(1, n);    % 海水深度
 WL = zeros(1, n);    % 左侧（上坡）半宽度
 WR = zeros(1, n);    % 右侧（下坡）半宽度
 W  = zeros(1, n);    % 总覆盖宽度
 eta = zeros(1, n);   % 重叠率
 
 %% 模型计算
 tan_half = tan(half_theta);           % tan(60 deg)
 tan_alpha = tan(alpha_rad);           % tan(1.5 deg)
 
 for i = 1:n
     % 1) 海水深度（斜坡线性变化）
     D(i) = D0 + x(i) * tan_alpha;
     
     % 2) 左、右半宽度（考虑坡度不对称性）
     WL(i) = D(i) * tan_half / (1 + tan_alpha * tan_half);
     WR(i) = D(i) * tan_half / (1 - tan_alpha * tan_half);
     
     % 3) 总覆盖宽度
     W(i) = WL(i) + WR(i);
 end
 
 %% 重叠率计算
 % 定义：eta = 1 - d/W
 % d 为相邻测线间距，W 为该测线位置处的覆盖宽度
 for i = 1:n
     if i == 1
         eta(i) = NaN;          % 第一条测线无前一条
     else
         eta(i) = (1 - d / W(i)) * 100;   % 转换为百分比，三位小数
     end
 end
 
 %% 输出结果
 fprintf('多波束测深问题1计算结果\n');
 fprintf('====================================================================\n');
 fprintf('测线距中心点处的距离/m  ');
 fprintf('%6d  ', x);
 fprintf('\n');
 fprintf('海水深度/m              ');
 fprintf('%.3f  ', D);
 fprintf('\n');
 fprintf('左半宽度/m               ');
 fprintf('%.3f  ', WL);
 fprintf('\n');
 fprintf('右半宽度/m               ');
 fprintf('%.3f  ', WR);
 fprintf('\n');
 fprintf('覆盖宽度/m               ');
 fprintf('%.3f  ', W);
 fprintf('\n');
 fprintf('与前一条测线的重叠率/%%    ');
 for i = 1:n
     if i == 1
         fprintf('   ——   ');
     else
         fprintf('%.3f  ', eta(i));
     end
 end
 fprintf('\n');
 fprintf('====================================================================\n');
 
 %% 保存到Excel（三位小数）
 T = table(x', round(D',3), round(W',3), [NaN; round(eta(2:end)',3)], ...
     'VariableNames', {'距离_m','深度_m','覆盖宽度_m','重叠率_pct'});
 writetable(T, 'result1.xlsx');
 fprintf('\n结果已保存至 result1.xlsx\n');
