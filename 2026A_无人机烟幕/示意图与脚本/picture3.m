%% 遮蔽判定算法原理示意图 (防遮挡版) v3
% 基于单位球面投影法的有效遮蔽判别模型
% 修复文字溢出问题

clear; clc; close all;

%% ==================== 全局设置 ====================
set(0, 'DefaultAxesFontSize', 9);
set(0, 'DefaultTextFontSize', 9);
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'DefaultAxesBox', 'off');

%% ==================== 场景参数 ====================
M      = [5000, 0, 1500];       % 导弹 M_i
T_base = [0, 200, 0];          % 真目标下底面圆心
r_T    = 7;                    % 圆柱底面半径 [m]
h_T    = 10;                   % 圆柱高度 [m]
C      = [1800, 100, 800];     % 烟幕云团中心
R_s    = 650;                  % 烟幕有效半径 [m]
R_vis  = 1.8;                  % 单位球面可视化放大系数

%% ==================== 预计算 ====================
v_CM      = C - M;
d_cm      = norm(v_CM);
v_CM_unit = v_CM / d_cm;
theta_s   = asin(min(R_s/d_cm, 1));

if abs(v_CM_unit(1)) < 0.9
    v_perp1 = cross(v_CM_unit, [1,0,0]);
else
    v_perp1 = cross(v_CM_unit, [0,1,0]);
end
v_perp1 = v_perp1 / norm(v_perp1);
v_perp2 = cross(v_CM_unit, v_perp1);
v_perp2 = v_perp2 / norm(v_perp2);

%% ==================== 创建图形窗口 ====================
fig = figure('Position', [50, 50, 1700, 950], ...
             'Color', 'w', ...
             'Name', '遮蔽判定算法原理示意图', ...
             'NumberTitle', 'off');

%% ========================================================================
%%  Panel 1: 三维场景总览 (左半部 2/3)
%% ========================================================================
ax1 = subplot(2, 3, [1, 2, 4, 5]);
hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on');
xlabel(ax1, 'X (m)', 'FontSize', 10);
ylabel(ax1, 'Y (m)', 'FontSize', 10);
zlabel(ax1, 'Z (m)', 'FontSize', 10);
title(ax1, '(a)  三维场景几何关系', 'FontSize', 12, 'FontWeight', 'bold');
view(ax1, -42, 22);

% ---------- 真目标圆柱 (半透明红) ----------
[Xc, Yc, Zc] = cylinder(r_T, 50);
Zc = Zc * h_T + T_base(3);
surf(ax1, Xc+T_base(1), Yc+T_base(2), Zc, ...
    'FaceColor',[0.85 0.30 0.30],'FaceAlpha',0.30, ...
    'EdgeColor',[0.60 0.30 0.30],'EdgeAlpha',0.20);
fill3(ax1, Xc(1,:)+T_base(1), Yc(1,:)+T_base(2), Zc(1,:), ...
    [0.85 0.30 0.30],'FaceAlpha',0.30,'EdgeColor',[0.60 0.30 0.30],'EdgeAlpha',0.20);
fill3(ax1, Xc(2,:)+T_base(1), Yc(2,:)+T_base(2), Zc(2,:), ...
    [0.85 0.30 0.30],'FaceAlpha',0.30,'EdgeColor',[0.60 0.30 0.30],'EdgeAlpha',0.20);

% ---------- 导弹 ----------
scatter3(ax1, M(1), M(2), M(3), 220, 'b', 'filled', ...
    'MarkerEdgeColor','k','LineWidth',1.5);
text(ax1, M(1)+300, M(2)-600, M(3)+250, ...
    '导弹 M_i','FontSize',11,'Color','b','FontWeight','bold');

% ---------- 烟幕球体 (半透明蓝) ----------
[sx, sy, sz] = sphere(60);
sx = sx*R_s + C(1); sy = sy*R_s + C(2); sz = sz*R_s + C(3);
surf(ax1, sx, sy, sz, ...
    'FaceColor',[0.30 0.50 0.85],'FaceAlpha',0.18, ...
    'EdgeColor',[0.35 0.55 0.90],'EdgeAlpha',0.12);
text(ax1, C(1)-250, C(2)-1050, C(3)+350, ...
    '烟幕球体 (R_s)','FontSize',11,'Color',[0.10 0.30 0.70],'FontWeight','bold');

% ---------- 导弹->烟幕连线 ----------
plot3(ax1, [M(1) C(1)], [M(2) C(2)], [M(3) C(3)], ...
    '--','Color',[0.15 0.15 0.65],'LineWidth',2.2);

% ---------- 导弹->目标采样视线 (淡粉) ----------
ths = linspace(0,2*pi,10);
for zi = 1:3
    zz = T_base(3) + [0, h_T*0.5, h_T];
    for ti = 1:10
        px = T_base(1)+r_T*cos(ths(ti));
        py = T_base(2)+r_T*sin(ths(ti));
        plot3(ax1, [M(1) px],[M(2) py],[M(3) zz(zi)], ...
            '-','Color',[1 0.55 0.55 0.35],'LineWidth',0.5);
    end
end

% ---------- 单位球面 (浅灰, 极低不透明度) ----------
[ux, uy, uz] = sphere(45);
urad = R_vis*500;
ux = ux*urad + M(1); uy = uy*urad + M(2); uz = uz*urad + M(3);
surf(ax1, ux, uy, uz, ...
    'FaceColor',[0.92 0.92 0.92],'FaceAlpha',0.07, ...
    'EdgeColor',[0.55 0.55 0.55],'EdgeAlpha',0.08);
text(ax1, M(1)+700, M(2)+1250, M(3)+300, ...
    '单位球面 S^2','FontSize',11,'Color',[0.45 0.45 0.45]);

% ---------- 烟幕球冠边界圆 (橙色粗线) ----------
nc = 80; cap3 = zeros(3,nc);
for k = 1:nc
    phi = 2*pi*(k-1)/nc;
    dv = cos(theta_s)*v_CM_unit + sin(theta_s)*(cos(phi)*v_perp1 + sin(phi)*v_perp2);
    cap3(:,k) = M + dv*urad;
end
plot3(ax1, cap3(1,:), cap3(2,:), cap3(3,:), ...
    '-','Color',[1.00 0.45 0.00],'LineWidth',2.8);

% ---------- 目标投影采样点 (红色方块) ----------
np = 30; proj3 = zeros(3,np);
for k = 1:np
    th = 2*pi*(k-1)/np;
    if mod(k,2)==0
        pk = [T_base(1)+r_T*cos(th), T_base(2)+r_T*sin(th), T_base(3)+h_T];
    else
        pk = [T_base(1)+r_T*cos(th), T_base(2)+r_T*sin(th), T_base(3)];
    end
    vk = (pk-M)/norm(pk-M);
    proj3(:,k) = M + vk*urad;
end
plot3(ax1, proj3(1,:), proj3(2,:), proj3(3,:), ...
    's-','Color',[1.00 0.15 0.15],'LineWidth',2, ...
    'MarkerSize',7,'MarkerFaceColor',[1.00 0.35 0.35]);

% ---------- 手动标注 (放在空旷区域) ----------
text(ax1, cap3(1,32)+400, cap3(2,32)+250, cap3(3,32), ...
    '烟幕球冠 \partialS','FontSize',10,'Color',[1.00 0.45 0.00],'FontWeight','bold');
text(ax1, proj3(1,15)+250, proj3(2,15)+320, proj3(3,15), ...
    '目标投影 T','FontSize',10,'Color',[1.00 0.15 0.15],'FontWeight','bold');
text(ax1, T_base(1)+400, T_base(2)+600, T_base(3)+h_T+200, ...
    '真目标 (圆柱)','FontSize',11,'Color',[0.65 0.15 0.15],'FontWeight','bold');

% ---------- 坐标轴范围 (留足余量防止文字溢出) ----------
axis(ax1, [M(1)-500, M(1)+6200, M(2)-3000, M(2)+3000, M(3)-1000, M(3)+2400]);

%% ========================================================================
%%  Panel 2: 单位球面投影原理 (右上)
%% ========================================================================
ax2 = subplot(2,3,3);
hold(ax2,'on'); axis(ax2,'equal');
title(ax2, '(b)  单位球面投影 (S^2)','FontSize',11,'FontWeight','bold');

% 单位圆
nci = 200;
plot(ax2, cos(linspace(0,2*pi,nci)), sin(linspace(0,2*pi,nci)), ...
    'k-','LineWidth',1.8);
fill(ax2, cos(linspace(0,2*pi,nci))*0.98, sin(linspace(0,2*pi,nci))*0.98, ...
    [0.94 0.97 1.00],'FaceAlpha',0.45,'EdgeColor','none');

% 烟幕球冠投影 (蓝色圆盘)
tv = theta_s*0.85;
nc2 = 100;
cx_d = tv*cos(linspace(0,2*pi,nc2));
cy_d = tv*sin(linspace(0,2*pi,nc2));
fill(ax2, cx_d, cy_d, [0.30 0.50 0.85], ...
    'FaceAlpha',0.28,'EdgeColor',[0.20 0.35 0.70],'LineWidth',2.2);

% 烟幕中心 +
plot(ax2, 0, 0, '+','Color',[0.15 0.25 0.60],'MarkerSize',12,'LineWidth',2);

% 目标投影2D映射
lz = v_CM_unit;
if abs(lz(1))<0.9
    lx = cross(lz,[1 0 0]);
else
    lx = cross(lz,[0 1 0]);
end
lx = lx/norm(lx); ly = cross(lz,lx);
nt = 80; t2d = zeros(2,nt);
for k = 1:nt
    th = 2*pi*(k-1)/nt;
    if mod(k,3)==0
        p3 = [T_base(1)+r_T*cos(th), T_base(2)+r_T*sin(th), T_base(3)+h_T];
    elseif mod(k,3)==1
        p3 = [T_base(1)+r_T*cos(th), T_base(2)+r_T*sin(th), T_base(3)];
    else
        p3 = [T_base(1)+r_T*cos(th), T_base(2)+r_T*sin(th), T_base(3)+h_T/2];
    end
    v3 = (p3-M)/norm(p3-M);
    zen = acos(dot(v3,lz)); azi = atan2(dot(v3,ly),dot(v3,lx));
    t2d(1,k) = zen*cos(azi); t2d(2,k) = zen*sin(azi);
end
plot(ax2, t2d(1,:), t2d(2,:), ...
    's-','Color',[1.00 0.15 0.15],'LineWidth',1.8, ...
    'MarkerSize',5,'MarkerFaceColor',[1.00 0.35 0.35]);

% 标注 (文字全部在数据范围内)
text(ax2, 0.02, -0.10, 'c-hat (烟幕中心)','FontSize',10, ...
    'Color',[0.15 0.25 0.60],'FontWeight','bold');
text(ax2, mean(t2d(1,:))+0.04, mean(t2d(2,:))+0.06, ...
    'T (目标投影)','FontSize',10,'Color',[0.80 0.15 0.15],'FontWeight','bold');
text(ax2, -0.30, 0.35, 'S (烟幕球冠)','FontSize',10, ...
    'Color',[0.15 0.25 0.60],'FontWeight','bold');
plot(ax2, [0 tv*cos(pi/6)],[0 tv*sin(pi/6)],'k-','LineWidth',1.8);
text(ax2, tv*cos(pi/6)/2-0.02, tv*sin(pi/6)/2+0.05, ...
    '\theta_s','FontSize',12,'FontWeight','bold');

xlabel(ax2, '投影 x (rad)','FontSize',9);
ylabel(ax2, '投影 y (rad)','FontSize',9);

% 轴范围严格控制文字不溢出
axis(ax2, [-0.58 0.58 -0.50 0.50]);
grid(ax2,'on'); set(ax2,'GridAlpha',0.25);

%% ========================================================================
%%  Panel 3: 方位-俯仰展开对比图 (右下)
%% ========================================================================
ax3 = subplot(2,3,6);
hold(ax3,'on');
title(ax3, '(c)  方位-俯仰展开','FontSize',11,'FontWeight','bold');

% 背景网格
for g = -2.5:0.5:2.5
    plot(ax3, [-pi pi],[g g],'-','Color',[0.88 0.88 0.88],'LineWidth',0.3);
    plot(ax3, [g g],[-pi/2 pi/2],'-','Color',[0.88 0.88 0.88],'LineWidth',0.3);
end

% 烟幕中心球面坐标
cpsi = asin(v_CM_unit(3));
cphi = atan2(v_CM_unit(2), v_CM_unit(1));

% 烟幕球冠展开
ca = linspace(0,2*pi,120);
ce = cphi + theta_s*cos(ca);
cp = cpsi + theta_s*sin(ca);
fill(ax3, ce, cp, [0.30 0.50 0.85], ...
    'FaceAlpha',0.28,'EdgeColor',[0.20 0.35 0.70],'LineWidth',2.2);
plot(ax3, cphi, cpsi, '+','Color',[0.15 0.25 0.60], ...
    'MarkerSize',14,'LineWidth',2.5);

% 目标投影展开
ne = 120; tphi = zeros(1,ne); tpsi = zeros(1,ne);
for k = 1:ne
    th = 2*pi*(k-1)/ne;
    if mod(k,3)==0
        p3t = [T_base(1)+r_T*cos(th), T_base(2)+r_T*sin(th), T_base(3)+h_T];
    elseif mod(k,3)==1
        p3t = [T_base(1)+r_T*cos(th), T_base(2)+r_T*sin(th), T_base(3)];
    else
        p3t = [T_base(1)+r_T*cos(th), T_base(2)+r_T*sin(th), T_base(3)+h_T/2];
    end
    vt = (p3t-M)/norm(p3t-M);
    tpsi(k) = asin(vt(3)); tphi(k) = atan2(vt(2), vt(1));
end
plot(ax3, tphi, tpsi, 'o-','Color',[1.00 0.15 0.15], ...
    'LineWidth',1.8,'MarkerSize',4,'MarkerFaceColor',[1.00 0.35 0.35]);
fill(ax3, tphi, tpsi, [1.00 0.15 0.15],'FaceAlpha',0.12,'EdgeColor','none');

% 标注 (全部在轴范围内)
text(ax3, cphi+theta_s*cos(pi/4)+0.12, cpsi+theta_s*sin(pi/4), ...
    'S (球冠)','FontSize',10,'Color',[0.15 0.25 0.60],'FontWeight','bold');
text(ax3, mean(tphi)+0.14, mean(tpsi)+0.08, ...
    'T (目标)','FontSize',10,'Color',[0.70 0.15 0.15],'FontWeight','bold');
text(ax3, cphi+0.04, cpsi+0.05, 'c-hat','FontSize',10, ...
    'Color',[0.15 0.25 0.60],'FontWeight','bold');
plot(ax3, [cphi cphi+theta_s*cos(pi/3)],[cpsi cpsi+theta_s*sin(pi/3)], ...
    'k-','LineWidth',1.6);
text(ax3, cphi+theta_s*cos(pi/3)/2, cpsi+theta_s*sin(pi/3)/2+0.045, ...
    '\theta_s','FontSize',11,'FontWeight','bold');

xlabel(ax3, '方位角 \phi (rad)','FontSize',9);
ylabel(ax3, '俯仰角 \psi (rad)','FontSize',9);

% 轴范围严格控制
axis(ax3, [-0.80 1.00 -0.65 0.75]);
grid(ax3,'on'); set(ax3,'GridAlpha',0.25);

%% ========================================================================
%%  底部判别式 (使用 textbox annotation, 不绑定数据坐标)
%%  只放一行浓缩公式, 确保不溢出
%% ========================================================================
annotation('textbox', [0.06, 0.02, 0.88, 0.06], ...
    'String', [...
    '遮蔽判别准则:  \forall v_k-hat \in \partialT,  ' ...
    'arccos(v_k-hat \cdot c-hat) \leq \theta_s,  ' ...
    '\theta_s = arcsin(R_s / ||r_c - r_{Mi}||)  |  ' ...
    '特殊情形: 导弹位于烟幕内部 (||r_c - r_{Mi}|| \leq R_s) 时恒有效'], ...
    'FontSize', 10, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 1.8, ...
    'BackgroundColor', [0.97 0.97 0.97], ...
    'FitBoxToText', 'on');   % 关键: 自动适配文本大小

%% ========================================================================
%%  右上角判定流程框 (FitBoxToText 防止溢出)
%% ========================================================================
annotation('textbox', [0.12, 0.75, 0.26, 0.16], ...
    'String', {...
    '【判定流程】', ...
    '1. 烟幕球体和真目标投影至', ...
    '   导弹为球心的单位球面 S^2', ...
    '2. 烟幕投影为球冠 S (角半径 \theta_s)', ...
    '3. 检查目标边界全部投影点', ...
    '   是否落入球冠内部', ...
    '4. 若 max \Delta\theta_k \leq \theta_s,', ...
    '   则判定为有效遮蔽'}, ...
    'FontSize', 9, ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
    'EdgeColor', [0.40 0.40 0.40], 'LineWidth', 1.2, ...
    'BackgroundColor', [1.00 1.00 0.92], ...
    'FitBoxToText', 'on');   % 关键: 自动适配

%% ========================================================================
%%  导出
%% ========================================================================
drawnow;

exportgraphics(fig, 'C:\Users\ming\Desktop\occlusion_algorithm_diagram.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'white');
exportgraphics(fig, 'C:\Users\ming\Desktop\occlusion_algorithm_diagram.png', ...
    'Resolution', 300, 'BackgroundColor', 'white');
savefig(fig, 'C:\Users\ming\Desktop\occlusion_algorithm_diagram.fig');

fprintf('图形已保存至:\n');
fprintf('  PDF: C:\\Users\\ming\\Desktop\\occlusion_algorithm_diagram.pdf\n');
fprintf('  PNG: C:\\Users\\ming\\Desktop\\occlusion_algorithm_diagram.png\n');
fprintf('  FIG: C:\\Users\\ming\\Desktop\\occlusion_algorithm_diagram.fig\n');
