function fig4_effective_masking_3d()
% 图 4：有效遮挡三维示意图（立体版，中文标注，无乱码）
% 运行后直接鼠标左键旋转查看空间关系

%% ==================== 1. 字体预设（防乱码核心） ====================
set(groot, 'defaultAxesFontName',  'SimHei');
set(groot, 'defaultTextFontName',  'SimHei');
set(groot, 'defaultLegendFontName','SimHei');

%% ==================== 2. 三维场景数据 ====================
P1  = [-6.0;  0.0;  1.5];   % 导弹（观察点）位置
C   = [ 0.0;  0.0;  0.0];   % 烟幕云团中心
r   = 1.4;                  % 有效遮蔽半径

% 真目标：圆柱体，下底面圆心，半径，高度
P2c  = [ 5.0; -1.0; -0.8];
Rcyl = 0.35;
Hcyl = 1.0;

%% ==================== 3. 计算切点（在 P1-C-P2 平面内） ====================
u  = P1 - C;                % 向量 C -> P1
d  = norm(u);
e1 = u / d;                 % 沿 CP1 方向的单位向量

% 构造平面内垂直于 e1 的 e2（以 P2 为参考方向投影）
v_ref    = P2c - C;
e2_temp  = v_ref - dot(v_ref, e1) * e1;
if norm(e2_temp) < 1e-6     % 若共线则另取一个垂直方向
    e2_temp = [0; 1; 0] - dot([0; 1; 0], e1) * e1;
end
e2 = e2_temp / norm(e2_temp);

% 切点：从 C 到切点的向量与 e1 夹角为 acos(r/d)
theta = acos(r / d);
T1 = C + r * (cos(theta)*e1 + sin(theta)*e2);
T2 = C + r * (cos(theta)*e1 - sin(theta)*e2);

%% ==================== 4. 创建画布 ====================
figure('Name','图4 有效遮挡三维示意图', 'Color','w', ...
       'Position',[300 150 1100 800]);
ax = axes('Position',[0.08 0.08 0.74 0.88]);
hold(ax,'on'); grid(ax,'on'); box(ax,'on');
ax.GridColor = [0.8 0.8 0.8]; ax.GridAlpha = 0.4;
ax.Color = [1 1 1];

% 开启光照，增强立体感
light('Position',[-10 8 10], 'Style','infinite');
lighting gouraud;
material dull;

%% ==================== 5. 绘制烟幕球体 ====================
[Xs, Ys, Zs] = sphere(50);
surf(ax, C(1)+r*Xs, C(2)+r*Ys, C(3)+r*Zs, ...
     'FaceColor',[0.82 0.82 0.82], 'FaceAlpha',0.45, ...
     'EdgeColor',[0.4 0.4 0.4], 'EdgeAlpha',0.25, ...
     'FaceLighting','gouraud');

%% ==================== 6. 绘制真目标圆柱体（轴线竖直）====================
draw_cylinder_3d(ax, P2c, Rcyl, Hcyl, [0.95 0.93 0.88], 0.6);

%% ==================== 7. 绘制连线 ====================
% 贯穿虚线 P1 — C — P2
plot3(ax, [P1(1), C(1), P2c(1)], ...
          [P1(2), C(2), P2c(2)], ...
          [P1(3), C(3), P2c(3)], ...
      'k--', 'LineWidth', 1.3);

% 切线（实线）：从 P1 经过切点并向远处延伸
for T = {T1, T2}
    Tp  = T{1};
    dir = (Tp - P1) / norm(Tp - P1);
    far = P1 + 7.5 * dir;   % 向远方拉长，示意遮挡边界
    plot3(ax, [P1(1), far(1)], [P1(2), far(2)], [P1(3), far(3)], ...
          'k-', 'LineWidth', 1.0);
end

% 半径标注线（球心到球面，45°方向）
r_pt = C + [r*0.7; r*0.5; r*0.35];
plot3(ax, [C(1), r_pt(1)], [C(2), r_pt(2)], [C(3), r_pt(3)], ...
      'k-', 'LineWidth', 1);

%% ==================== 8. 绘制关键点 ====================
scatter3(ax, P1(1),  P1(2),  P1(3),  80, 'k', 'filled', ...
         'MarkerEdgeColor','w','LineWidth',1.2);   % P1
scatter3(ax, C(1),   C(2),   C(3),   50, 'k', 'filled');   % C
scatter3(ax, P2c(1), P2c(2), P2c(3), 40, 'k', 'filled');   % P2 中心

%% ==================== 9. 三维空间标注（分散防遮挡）====================
% 所有标注关闭 LaTeX，使用 SimHei，确保零乱码

% P1：偏左上方
text(ax, P1(1)-0.6, P1(2)+0.3, P1(3)+0.5, 'P1', ...
     'FontSize',15, 'FontWeight','bold', 'Color','k', ...
     'HorizontalAlignment','right');

% C：偏右下方
text(ax, C(1)+0.25, C(2)-0.4, C(3)-0.35, 'C', ...
     'FontSize',15, 'FontWeight','bold', 'Color','k');

% P2：偏右下方
text(ax, P2c(1)+0.4, P2c(2)-0.5, P2c(3)-0.3, 'P2', ...
     'FontSize',15, 'FontWeight','bold', 'Color','k');

% r：半径线旁
text(ax, (C(1)+r_pt(1))/2+0.15, (C(2)+r_pt(2))/2+0.1, ...
     (C(3)+r_pt(3))/2+0.15, 'r', ...
     'FontSize',14, 'FontWeight','bold', 'Color','k');

%% ==================== 10. 中文汇总说明框（右上角固定，不随旋转）====================
annotation('textbox', [0.66 0.68 0.28 0.24], 'String', ...
    {'【符号说明】', ...
     '', ...
     'P1：导弹（观察点）', ...
     'C：烟幕云团中心', ...
     'r：有效遮蔽半径', ...
     'P2：真目标中心', ...
     '', ...
     '实线：切线（遮挡边界）', ...
     '虚线：P1 → C → P2'}, ...
    'FontName','SimHei', 'Interpreter','none', ...
    'FontSize',12, 'EdgeColor',[0.3 0.3 0.3], ...
    'BackgroundColor',[1 1 1 0.96], 'FitBoxToText','off', ...
    'Margin', 6, 'HorizontalAlignment','left', ...
    'VerticalAlignment','top');

%% ==================== 11. 视角与美化 ====================
xlabel(ax,'X / m', 'FontSize',12);
ylabel(ax,'Y / m', 'FontSize',12);
zlabel(ax,'Z / m', 'FontSize',12);
title(ax,'图 4：有效遮挡三维示意图', 'FontSize',16, 'FontWeight','bold');

% 保持三维比例真实，不压扁
axis(ax,'equal');
daspect(ax,[1 1 1]);

% 初始视角：斜向俯视，看清切线与圆柱的前后关系
view(ax, 40, 18);

% 坐标轴范围
xlim(ax, [-9 9]);
ylim(ax, [-5 5]);
zlim(ax, [-3 4]);

% 开启交互旋转
rotate3d(ax,'on');

% 底部操作提示
annotation('textbox',[0.02 0.01 0.35 0.04],'String', ...
    {'提示：鼠标左键旋转视角 | 滚轮缩放 | 右键平移'}, ...
    'FontName','SimHei','Interpreter','none','FontSize',10, ...
    'Color',[0.4 0.4 0.4],'EdgeColor','none', ...
    'BackgroundColor',[1 1 1 0.75]);

disp('三维模型已生成，按住鼠标左键即可旋转查看空间遮挡关系。');
end

%% ==================== 辅助函数：绘制三维圆柱体 ====================
function draw_cylinder_3d(ax, center, radius, height, color, alpha)
% 轴线平行于 Z 轴的半透明圆柱体
    [theta_cyl, z_cyl] = meshgrid(linspace(0,2*pi,50), linspace(0,height,25));
    x_surf = center(1) + radius * cos(theta_cyl);
    y_surf = center(2) + radius * sin(theta_cyl);
    z_surf = center(3) + z_cyl;
    
    surf(ax, x_surf, y_surf, z_surf, ...
         'FaceColor',color, 'FaceAlpha',alpha, ...
         'EdgeColor','k', 'EdgeAlpha',0.25, ...
         'FaceLighting','gouraud');
    
    % 顶面与底面
    [xd, yd] = meshgrid(linspace(-radius,radius,35));
    mask = (xd.^2 + yd.^2) <= radius^2;
    xd = xd .* mask + center(1);
    yd = yd .* mask + center(2);
    
    surf(ax, xd, yd, center(3)*ones(size(xd)), ...
         'FaceColor',color, 'FaceAlpha',alpha, 'EdgeColor','none');
    surf(ax, xd, yd, (center(3)+height)*ones(size(xd)), ...
         'FaceColor',color, 'FaceAlpha',alpha, 'EdgeColor','none');
end