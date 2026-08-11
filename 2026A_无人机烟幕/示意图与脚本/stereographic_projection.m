%% 单位球面投影法 (Stereographic Projection) 原理示意图
%  防遮挡版本 — 透明球面 + 多视角 + 光照增强
clear; close all;

%% ========== 参数设置 ==========
N_meridian = 36;       % 经线密度
N_latitude = 18;       % 纬线密度
N_proj_pts = 12;       % 球面上投影采样点个数
alpha_sphere = 0.22;   % 球面透明度
proj_center = [0, 0, 1];  % 投影中心 (北极)

%% ========== 生成单位球面网格 ==========
[theta, phi] = meshgrid(linspace(0, 2*pi, N_meridian+1), ...
                         linspace(0, pi, N_latitude+1));
X = sin(phi) .* cos(theta);
Y = sin(phi) .* sin(theta);
Z = cos(phi);

%% ========== 生成经线 (子午线) ==========
theta_lines = linspace(0, 2*pi, N_meridian+1);
phi_fine    = linspace(0, pi, 200);

%% ========== 生成纬线 ==========
phi_lines = linspace(0.1, pi-0.1, N_latitude);
theta_fine = linspace(0, 2*pi, 300);

%% ========== 球面上待投影的点 (上半球面 + 下半球面各若干) ==========
rng(42);
% 上半球面 8 个点
n_upper = 8;
phi_upper   = linspace(0.15, pi/2 - 0.15, n_upper);
theta_upper = linspace(0, 1.85*pi, n_upper);
% 下半球面 4 个点
n_lower = 4;
phi_lower   = linspace(pi/2 + 0.2, pi - 0.25, n_lower);
theta_lower = linspace(0.3*pi, 1.6*pi, n_lower);

phi_pts   = [phi_upper, phi_lower];
theta_pts = [theta_upper, theta_lower];

pts_sphere = [sin(phi_pts') .* cos(theta_pts'), ...
              sin(phi_pts') .* sin(theta_pts'), ...
              cos(phi_pts')];

%% ========== 计算投影点 (北极 → 球面点 → 投影平面 z=0) ==========
% 参数方程: P(t) = N + t*(S - N),  N=[0,0,1]
% 令 z=0:  1 + t*(sz - 1) = 0  =>  t = 1/(1 - sz)
pts_proj = zeros(size(pts_sphere));
for k = 1:size(pts_sphere, 1)
    sx = pts_sphere(k, 1); sy = pts_sphere(k, 2); sz = pts_sphere(k, 3);
    if abs(sz - 1) < 1e-9
        pts_proj(k, :) = [inf, inf, 0];  % 北极点自身 → 无穷远
    else
        t = 1 / (1 - sz);
        pts_proj(k, :) = [1 + t*(sx - 1), 0 + t*(sy - 0), 0];
        % 即: px = sx/(1-sz), py = sy/(1-sz)
    end
end

%% ========== 绘图 ==========
fig = figure('Color', 'w', 'Position', [80, 80, 1400, 650]);
colormap(jet(256));

% ===== 子图 1: 侧视图 =====
subplot(1,2,1);
hold on; axis equal; grid on;
xlim([-2.8, 2.8]); ylim([-2.8, 2.8]); zlim([-1.5, 1.8]);

% 透明球面
surf(X, Y, Z, 'FaceAlpha', alpha_sphere, 'EdgeColor', 'none', ...
     'FaceColor', [0.45, 0.65, 0.85]);

% 经线 (半透明)
for i = 1:4:N_meridian+1
    t = theta_lines(i);
    x_line = sin(phi_fine) * cos(t);
    y_line = sin(phi_fine) * sin(t);
    z_line = cos(phi_fine);
    plot3(x_line, y_line, z_line, 'Color', [0.3 0.3 0.3 0.35], 'LineWidth', 0.6);
end

% 纬线 (半透明)
for i = 1:length(phi_lines)
    p = phi_lines(i);
    r = sin(p);
    x_circ = r * cos(theta_fine);
    y_circ = r * sin(theta_fine);
    z_circ = cos(p) * ones(size(theta_fine));
    plot3(x_circ, y_circ, z_circ, 'Color', [0.3 0.3 0.3 0.25], 'LineWidth', 0.5);
end

% 赤道 (加粗)
r_eq = 1;
x_eq = r_eq * cos(theta_fine);
y_eq = r_eq * sin(theta_fine);
z_eq = zeros(size(theta_fine));
plot3(x_eq, y_eq, z_eq, 'k-', 'LineWidth', 1.5);

% 投影平面 (z=0 处的圆盘)
[Xp, Yp] = meshgrid(linspace(-2.6, 2.6, 60));
Zp = zeros(size(Xp));
mask = (Xp.^2 + Yp.^2) <= 2.6^2;
Zp(~mask) = NaN;
surf(Xp, Yp, Zp, 'FaceAlpha', 0.18, 'EdgeColor', 'none', ...
     'FaceColor', [0.85, 0.75, 0.55]);
% 投影平面边框
plot3(2.6*cos(theta_fine), 2.6*sin(theta_fine), zeros(size(theta_fine)), ...
      'Color', [0.5 0.4 0.2], 'LineWidth', 1.2);

% 北极点 (投影中心)
plot3(0, 0, 1, 'ro', 'MarkerSize', 11, 'MarkerFaceColor', 'r', ...
      'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

% 球面上的点 + 投影点 + 连线
for k = 1:size(pts_sphere, 1)
    ps = pts_sphere(k, :);
    pp = pts_proj(k, :);
    % 判断上半球面还是下半球面 → 颜色区分
    if ps(3) >= 0
        clr_sphere = [0.2, 0.6, 0.2];   % 上半球面: 绿
        clr_proj   = [0.1, 0.7, 0.1];
        clr_line   = [0.2, 0.65, 0.2];
    else
        clr_sphere = [0.75, 0.25, 0.25]; % 下半球面: 红
        clr_proj   = [0.85, 0.2, 0.2];
        clr_line   = [0.7, 0.3, 0.3];
    end
    % 球面上的点
    plot3(ps(1), ps(2), ps(3), 'o', 'MarkerSize', 8, ...
          'MarkerFaceColor', clr_sphere, 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
    % 投影点
    plot3(pp(1), pp(2), pp(3), 's', 'MarkerSize', 7, ...
          'MarkerFaceColor', clr_proj, 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
    % 连线: 北极 → 球面点 → 投影点
    line_pts = [proj_center; ps; pp];
    plot3(line_pts(:,1), line_pts(:,2), line_pts(:,3), ...
          '-', 'Color', [clr_line, 0.55], 'LineWidth', 1.0);
end

% 坐标轴
quiver3(-2.6, -2.6, -1.3, 1.1, 0, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
quiver3(-2.6, -2.6, -1.3, 0, 1.1, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
quiver3(-2.6, -2.6, -1.3, 0, 0, 1.1, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
text(-1.5, -2.7, -1.3, 'X', 'FontSize', 12, 'FontWeight', 'bold');
text(-2.7, -1.5, -1.3, 'Y', 'FontSize', 12, 'FontWeight', 'bold');
text(-2.7, -2.7, -0.2, 'Z', 'FontSize', 12, 'FontWeight', 'bold');

% 标注
text(0, 0, 1.2, 'N (北极 / 投影中心)', 'FontSize', 10, ...
     'FontWeight', 'bold', 'Color', 'r', 'HorizontalAlignment', 'center');
text(1.15, 0, -0.12, '赤道 (z=0)', 'FontSize', 9, 'Color', [0.2 0.2 0.2]);
text(1.8, 1.2, 0.08, '投影平面 \Pi', 'FontSize', 11, ...
     'Color', [0.5 0.4 0.15], 'FontWeight', 'bold');

title('侧面视图 — 透明球体 + 投影关系', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
view([-35, 22]);
lighting gouraud;
camlight('headlight');
material dull;

% ===== 子图 2: 俯视图 (从北极上方看) =====
subplot(1,2,2);
hold on; axis equal; grid on;
xlim([-2.8, 2.8]); ylim([-2.8, 2.8]); zlim([-1.5, 1.8]);

% 透明球面
surf(X, Y, Z, 'FaceAlpha', alpha_sphere, 'EdgeColor', 'none', ...
     'FaceColor', [0.45, 0.65, 0.85]);

% 纬线
for i = 1:length(phi_lines)
    p = phi_lines(i);
    r = sin(p);
    x_circ = r * cos(theta_fine);
    y_circ = r * sin(theta_fine);
    z_circ = cos(p) * ones(size(theta_fine));
    plot3(x_circ, y_circ, z_circ, 'Color', [0.3 0.3 0.3 0.25], 'LineWidth', 0.5);
end

% 赤道
plot3(x_eq, y_eq, z_eq, 'k-', 'LineWidth', 1.5);

% 投影平面
surf(Xp, Yp, Zp, 'FaceAlpha', 0.15, 'EdgeColor', 'none', ...
     'FaceColor', [0.85, 0.75, 0.55]);
plot3(2.6*cos(theta_fine), 2.6*sin(theta_fine), zeros(size(theta_fine)), ...
      'Color', [0.5 0.4 0.2], 'LineWidth', 1.2);

% 北极
plot3(0, 0, 1, 'ro', 'MarkerSize', 11, 'MarkerFaceColor', 'r', ...
      'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

% 球面点 + 投影点 + 连线
for k = 1:size(pts_sphere, 1)
    ps = pts_sphere(k, :);
    pp = pts_proj(k, :);
    if ps(3) >= 0
        clr_sphere = [0.2, 0.6, 0.2];
        clr_proj   = [0.1, 0.7, 0.1];
        clr_line   = [0.2, 0.65, 0.2];
    else
        clr_sphere = [0.75, 0.25, 0.25];
        clr_proj   = [0.85, 0.2, 0.2];
        clr_line   = [0.7, 0.3, 0.3];
    end
    plot3(ps(1), ps(2), ps(3), 'o', 'MarkerSize', 8, ...
          'MarkerFaceColor', clr_sphere, 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
    plot3(pp(1), pp(2), pp(3), 's', 'MarkerSize', 7, ...
          'MarkerFaceColor', clr_proj, 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
    line_pts = [proj_center; ps; pp];
    plot3(line_pts(:,1), line_pts(:,2), line_pts(:,3), ...
          '-', 'Color', [clr_line, 0.45], 'LineWidth', 1.0);
end

% 投影映射弧线示例 (可选: 展示保圆性)
% 选上半球面几个点画虚线弧
for k = 1:4
    ps = pts_sphere(k, :);
    pp = pts_proj(k, :);
    plot3([ps(1), pp(1)], [ps(2), pp(2)], [ps(3), pp(3)], ...
          ':', 'Color', [0.3 0.3 0.3 0.5], 'LineWidth', 0.8);
end

% 坐标轴
quiver3(-2.6, -2.6, -1.3, 1.1, 0, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
quiver3(-2.6, -2.6, -1.3, 0, 1.1, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
quiver3(-2.6, -2.6, -1.3, 0, 0, 1.1, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
text(-1.5, -2.7, -1.3, 'X', 'FontSize', 12, 'FontWeight', 'bold');
text(-2.7, -1.5, -1.3, 'Y', 'FontSize', 12, 'FontWeight', 'bold');
text(-2.7, -2.7, -0.2, 'Z', 'FontSize', 12, 'FontWeight', 'bold');

% 标注
text(0, 0, 1.25, 'N', 'FontSize', 12, 'FontWeight', 'bold', ...
     'Color', 'r', 'HorizontalAlignment', 'center');
text(1.8, 1.2, 0.08, '投影平面 \Pi', 'FontSize', 11, ...
     'Color', [0.5 0.4 0.15], 'FontWeight', 'bold');

% 图例
h_green = plot3(nan, nan, nan, 'go', 'MarkerFaceColor', [0.2 0.6 0.2], ...
                'MarkerSize', 8, 'MarkerEdgeColor', 'k');
h_red   = plot3(nan, nan, nan, 'ro', 'MarkerFaceColor', [0.75 0.25 0.25], ...
                'MarkerSize', 8, 'MarkerEdgeColor', 'k');
h_sq    = plot3(nan, nan, nan, 'ks', 'MarkerFaceColor', [0.5 0.5 0.5], ...
                'MarkerSize', 7, 'MarkerEdgeColor', 'k');
legend([h_green, h_red, h_sq], ...
       {'上半球面点 (z>0)', '下半球面点 (z<0)', '投影像点 (\Pi 上)'}, ...
       'Location', 'northeast', 'FontSize', 9);

title('俯视图 (从 N 上方) — 投影映射到平面 \Pi', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
view([0, 85]);
lighting gouraud;
camlight('headlight');
material dull;

%% ========== 保存图像 ==========
sgtitle('单位球面投影法 (Stereographic Projection) — 防遮挡原理图', ...
        'FontSize', 16, 'FontWeight', 'bold');
% exportgraphics(fig, 'stereographic_projection.png', 'Resolution', 300);
disp('>> 绘图完成。取消最后一行注释即可保存 PNG。');
