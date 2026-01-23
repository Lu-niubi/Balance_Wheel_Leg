function five_bar_xy_ik()
    % 五连杆 XY 逆解主程序
    clc; clear; close all;

    %% === 1. 用户输入 (在此修改目标 XY 坐标) ===
    % 坐标系定义：
    % 原点 (0,0) 在两个电机连线的中点。
    % X轴：向右为正
    % Y轴：向上为正 (如果是站立机器人，通常 Y 为负)
    
    target_x = 0;      % 目标 X 坐标 (mm)
    target_y = 32;    % 目标 Y 坐标 (mm)

    %% === 2. 调用 XY 逆解函数 ===
    [phi1, phi4, is_valid] = calculate_xy_ik(target_x, target_y);

    %% === 3. 输出结果 ===
    fprintf('========================================\n');
    fprintf('   五连杆 XY 逆解 (输入 X,Y -> 输出电机角度)\n');
    fprintf('========================================\n');
    fprintf('目标坐标:\n');
    fprintf('  X = %.2f mm\n', target_x);
    fprintf('  Y = %.2f mm\n', target_y);
    fprintf('----------------------------------------\n');

    if is_valid
        fprintf('计算结果 (电机角度):\n');
        fprintf('  Phi1 (左) = %.4f rad (%.2f°)\n', phi1, rad2deg(phi1));
        fprintf('  Phi4 (右) = %.4f rad (%.2f°)\n', phi4, rad2deg(phi4));
        
        % 绘图验证
        draw_robot_xy(phi1, phi4, target_x, target_y);
    else
        fprintf('ERROR: 目标点 (%.1f, %.1f) 超出工作空间！\n', target_x, target_y);
    end
    fprintf('========================================\n');
end

%% === 核心 XY 逆解算法 ===
function [phi1, phi4, valid] = calculate_xy_ik(x, y)
    % 机械参数 (mm)
    l1 = 61.0;  l4 = 61.0;
    l2 = 90.0;  l3 = 90.0;
    l5 = 38.0;
    
    w = l5 / 2.0; % 半个机身宽

    %% --- 1. 计算左电机角度 phi1 ---
    % 左电机 A 坐标: (-w, 0)
    % 向量 AC = (x - (-w), y - 0) = (x+w, y)
    x_AC = x + w;
    y_AC = y;
    Lac_sq = x_AC^2 + y_AC^2; % AC长度平方
    Lac = sqrt(Lac_sq);

    % 基础角度 (向量 AC 相对于 X 轴的角度)
    theta_A = atan2(y_AC, x_AC);

    % 三角形内角 (余弦定理)
    % cos(alpha) = (l1^2 + Lac^2 - l2^2) / (2 * l1 * Lac)
    cos_angle_L = (l1^2 + Lac_sq - l2^2) / (2 * l1 * Lac);

    %% --- 2. 计算右电机角度 phi4 ---
    % 右电机 E 坐标: (w, 0)
    % 向量 EC = (x - w, y - 0) = (x-w, y)
    x_EC = x - w;
    y_EC = y;
    Lec_sq = x_EC^2 + y_EC^2; % EC长度平方
    Lec = sqrt(Lec_sq);

    % 基础角度 (向量 EC 相对于 X 轴的角度)
    theta_E = atan2(y_EC, x_EC);

    % 三角形内角
    % cos(beta) = (l4^2 + Lec^2 - l3^2) / (2 * l4 * Lec)
    cos_angle_R = (l4^2 + Lec_sq - l3^2) / (2 * l4 * Lec);

    %% --- 3. 合法性检查 ---
    if abs(cos_angle_L) > 1 || abs(cos_angle_R) > 1
        phi1 = NaN; phi4 = NaN; valid = false;
        return;
    end
    valid = true;

    %% --- 4. 角度合成 (构型选择) ---
    % 这里的加减号决定了膝盖是朝外(Up)还是朝内(Down)
    % 对于大多数五连杆，左边是 +acos，右边是 -acos
    
    phi1 = theta_A + acos(cos_angle_L); % 左臂
    phi4 = theta_E - acos(cos_angle_R); % 右臂
end

%% === 绘图辅助函数 ===
function draw_robot_xy(phi1, phi4, tx, ty)
    l1 = 61; l4 = 61; l2 = 90; l3 = 90; l5 = 38;
    w = l5/2;

    % 关键点坐标 (A, E 在全局坐标系)
    A = [-w, 0];
    E = [ w, 0];
    
    % 正向推算肘部坐标 B, D
    B = A + [l1*cos(phi1), l1*sin(phi1)];
    D = E + [l4*cos(phi4), l4*sin(phi4)];
    
    % 目标点 C
    C = [tx, ty];

    figure(1); clf; hold on; axis equal; grid on;
    set(gcf, 'Color', 'w');

    % 绘图
    plot([A(1), E(1)], [A(2), E(2)], 'k-', 'LineWidth', 4); % 机身
    plot([A(1), B(1)], [A(2), B(2)], 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r'); % 左主动臂
    plot([B(1), C(1)], [B(2), C(2)], 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r'); % 左从动臂
    plot([E(1), D(1)], [E(2), D(2)], 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b'); % 右主动臂
    plot([D(1), C(1)], [D(2), C(2)], 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b'); % 右从动臂

    % 标记原点
    plot(0, 0, 'kx', 'MarkerSize', 10);
    text(0, -10, 'Origin(0,0)', 'HorizontalAlignment', 'center');
    
    title(sprintf('目标坐标 X=%.1f, Y=%.1f', tx, ty));
    xlabel('X (mm)'); ylabel('Y (mm)');
    axis([-100, 100, -50, 150]); % 根据需要调整视野
end