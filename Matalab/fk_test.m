clc; clear; close all;

%% === 用户输入区域 ===
% 这里输入角度 (单位：度)
input_angle_1_deg = 100;  % 左电机角度
input_angle_4_deg = 40;   % 右电机角度 (对称的话通常左右互补)

%% === 计算过程 ===
% 1. 角度转弧度
rad1 = deg2rad(input_angle_1_deg);
rad4 = deg2rad(input_angle_4_deg);

% 2. 调用正解函数
[L0, phi0] = five_bar_fwd_kinematics(rad1, rad4);

% 3. 输出结果
fprintf('--------------------------------\n');
fprintf('输入角度: phi1 = %.2f°, phi4 = %.2f°\n', input_angle_1_deg, input_angle_4_deg);
fprintf('--------------------------------\n');
if isnan(L0)
    fprintf('计算失败：位置不可达\n');
else
    fprintf('计算结果:\n');
    fprintf('虚拟腿长 L0   = %.4f mm\n', L0);
    fprintf('虚拟腿角 phi0 = %.4f rad (%.2f°)\n', phi0, rad2deg(phi0));
end
fprintf('--------------------------------\n');

%% === 绘图验证 (Visual Check) ===
if ~isnan(L0)
    draw_robot(rad1, rad4, L0, phi0);
end

%% === 辅助函数：绘图 ===
function draw_robot(phi1, phi4, L0, phi0)
    % 参数重定义用于绘图
    l1 = 61; l2 = 90; l5 = 38;
    
    % 关键点坐标重算
    A = [0, 0];
    E = [l5, 0];
    B = [l1*cos(phi1), l1*sin(phi1)];
    D = [l5 + l1*cos(phi4), l1*sin(phi4)];
    
    % 逆推C点用于画图
    xC_virt = L0 * cos(phi0);
    yC_virt = L0 * sin(phi0);
    C = [xC_virt + l5/2, yC_virt]; % 转回全局坐标

    figure(1); clf; hold on; axis equal; grid on;
    % 设置画图范围
    xlim([-100, 150]); ylim([-150, 100]);
    xlabel('X (mm)'); ylabel('Y (mm)');
    title('五连杆正运动学验证');

    % 画机身
    plot([A(1), E(1)], [A(2), E(2)], 'k-', 'LineWidth', 3); 
    % 画左腿
    plot([A(1), B(1)], [A(2), B(2)], 'r-o', 'LineWidth', 2); % l1
    plot([B(1), C(1)], [B(2), C(2)], 'r-o', 'LineWidth', 2); % l2
    % 画右腿
    plot([E(1), D(1)], [E(2), D(2)], 'b-o', 'LineWidth', 2); % l4
    plot([D(1), C(1)], [D(2), C(2)], 'b-o', 'LineWidth', 2); % l3
    % 画虚拟腿
    plot([l5/2, C(1)], [0, C(2)], 'g--', 'LineWidth', 1.5);

    % 标注
    text(A(1), A(2), ' A'); text(E(1), E(2), ' E');
    text(B(1), B(2), ' B'); text(D(1), D(2), ' D');
    text(C(1), C(2), ' C');
    legend('机身', '左腿(l1,l2)', '', '右腿(l4,l3)', '', '虚拟腿(L0)');
end

%% === 粘贴上面的正解函数 ===
function [L0, phi0] = five_bar_fwd_kinematics(phi1, phi4)
    l1 = 61; l2 = 90; l3 = 90; l4 = 61; l5 = 38;
    xB = l1 * cos(phi1); yB = l1 * sin(phi1);
    xD = l5 + l4 * cos(phi4); yD = l4 * sin(phi4);
    
    lBD_sq = (xD - xB)^2 + (yD - yB)^2;
    A0 = 2 * l2 * (xD - xB);
    B0 = 2 * l2 * (yD - yB);
    C0 = l2^2 + lBD_sq - l3^2;
    
    delta = A0^2 + B0^2 - C0^2;
    if delta < 0, L0 = NaN; phi0 = NaN; return; end
    
    phi2 = 2 * atan2((B0 + sqrt(delta)), (A0 + C0));
    xC = xB + l2 * cos(phi2);
    yC = yB + l2 * sin(phi2);
    
    x_center = xC - l5/2;
    y_center = yC;
    L0 = sqrt(x_center^2 + y_center^2);
    phi0 = atan2(y_center, x_center);
end