clc; clear; close all;

%% === 用户输入区域 ===
input_angle_1_deg = 180;  % 左电机角度 (相对于X轴正方向)
input_angle_4_deg = 0;   % 右电机角度 (相对于X轴正方向)

%% === 计算过程 ===
rad1 = deg2rad(input_angle_1_deg);
rad4 = deg2rad(input_angle_4_deg);

[L0, phi0, C_pos] = five_bar_fwd_kinematics_centered(rad1, rad4);

fprintf('--------------------------------\n');
fprintf('输入角度: phi1 = %.2f°, phi4 = %.2f°\n', input_angle_1_deg, input_angle_4_deg);
fprintf('--------------------------------\n');

if isnan(L0)
    fprintf('计算失败：位置不可达\n');
else
    fprintf('计算结果 (以机身中心为原点):\n');
    fprintf('C点坐标     = [%.2f, %.2f] mm\n', C_pos(1), C_pos(2));
    fprintf('虚拟腿长 L0   = %.4f mm\n', L0);
    fprintf('虚拟腿角 phi0 = %.4f rad (%.2f°)\n', phi0, rad2deg(phi0));
end
fprintf('--------------------------------\n');

%% === 绘图验证 ===
if ~isnan(L0)
    draw_robot_centered(rad1, rad4, C_pos);
end

%% === 辅助函数：正解 (中心原点版) ===
function [L0, phi0, C] = five_bar_fwd_kinematics_centered(phi1, phi4)
    l1 = 61; l2 = 90; l3 = 90; l4 = 61; l5 = 38;
    offset = l5 / 2;
    
    % 1. 计算主动臂末端 (相对于中心)
    xB = -offset + l1 * cos(phi1); 
    yB = l1 * sin(phi1);
    xD =  offset + l4 * cos(phi4); 
    yD = l4 * sin(phi4);
    
    % 2. 闭环矢量计算
    diff_x = xD - xB;
    diff_y = yD - yB;
    lBD_sq = diff_x^2 + diff_y^2;
    
    A0 = 2 * l2 * diff_x;
    B0 = 2 * l2 * diff_y;
    C0 = l2^2 + lBD_sq - l3^2;
    
    delta = A0^2 + B0^2 - C0^2;
    if delta < 0, L0 = NaN; phi0 = NaN; C = [NaN, NaN]; return; end
    
    % 3. 计算从动臂角度及C点坐标
    phi2 = 2 * atan2((B0 + sqrt(delta)), (A0 + C0));
    xC = xB + l2 * cos(phi2);
    yC = yB + l2 * sin(phi2);
    
    C = [xC, yC];
    L0 = sqrt(xC^2 + yC^2);
    phi0 = atan2(yC, xC);
end

%% === 辅助函数：绘图 (中心原点版) ===
function draw_robot_centered(phi1, phi4, C)
    l1 = 61; l5 = 38;
    offset = l5 / 2;
    
    A = [-offset, 0];
    E = [offset, 0];
    B = [-offset + l1*cos(phi1), l1*sin(phi1)];
    D = [offset + l1*cos(phi4), l1*sin(phi4)];
    
    figure(1); clf; hold on; axis equal; grid on;
    xlim([-150, 150]); ylim([-150, 150]);
    xlabel('X (mm)'); ylabel('Y (mm)');
    title('五连杆正运动学验证 (中心原点)');
    
    % 画图组件
    plot([A(1), E(1)], [A(2), E(2)], 'k-s', 'LineWidth', 3); % 机身
    plot([A(1), B(1)], [A(2), B(2)], 'r-o', 'LineWidth', 2); % 左连杆1
    plot([B(1), C(1)], [B(2), C(2)], 'r-o', 'LineWidth', 2); % 左连杆2
    plot([E(1), D(1)], [E(2), D(2)], 'b-o', 'LineWidth', 2); % 右连杆1
    plot([D(1), C(1)], [D(2), C(2)], 'b-o', 'LineWidth', 2); % 右连杆2
    plot([0, C(1)], [0, C(2)], 'g--', 'LineWidth', 1.5);    % 虚拟腿
    
    text(A(1), A(2)-10, 'A(-w,0)'); text(E(1), E(2)-10, 'E(w,0)');
    text(0, 0, ' O(0,0)'); text(C(1), C(2)+10, ' C');
    legend('机身', '左腿', '', '右腿', '', '虚拟腿');
end