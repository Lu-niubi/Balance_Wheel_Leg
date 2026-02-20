% =========================================================================
% LQR 参数计算器 - BDS3620 & 34mm半径修正版
% =========================================================================
clear; clc;

% --- 用户设置 ---
V_batt = 11.0;      % 电池电压
PWM_Max = 10000;    % PWM 满载值

% --- 腿长/重心高度范围 ---
leg_range = 0.03 : 0.005 : 0.08; 
len_num = length(leg_range);

K_data = zeros(8, len_num); 

fprintf('正在计算 LQR 增益 (R=34mm, Wheel=85g)...\n');

for i = 1:len_num
    K_curr = LQR_k_calc(leg_range(i), V_batt, PWM_Max);
    K_data(1:4, i) = K_curr(1, :); 
    K_data(5:8, i) = K_curr(2, :); 
end

% --- 多项式拟合 (3次) ---
Poly_Coeffs = zeros(8, 4); 
for row = 1:8
    Poly_Coeffs(row, :) = polyfit(leg_range, K_data(row, :), 3);
end

% --- C代码生成 ---
fprintf('\n// ===== 复制到单片机 (Radius 34mm) =====\n');
var_names = {'k_x', 'k_v', 'k_theta', 'k_omega'}; 
for i = 1:4
    coeffs = Poly_Coeffs(i, :);
    fprintf('float %s_poly[4] = {%.6f, %.6f, %.6f, %.6f};\n', ...
        var_names{i}, coeffs(1), coeffs(2), coeffs(3), coeffs(4));
end
fprintf('// ===========================================\n');

% 绘图检查
figure(1); sgtitle('LQR Gains (R=34mm)');
subplot(2,2,1); plot(leg_range, K_data(3,:), 'r-o'); title('Angle Gain (K_{\theta})'); grid on;
subplot(2,2,2); plot(leg_range, K_data(4,:), 'm-o'); title('Gyro Gain (K_{\omega})'); grid on;
subplot(2,2,3); plot(leg_range, K_data(1,:), 'b-o'); title('Pos Gain (K_{x})'); grid on;
subplot(2,2,4); plot(leg_range, K_data(2,:), 'k-o'); title('Speed Gain (K_{v})'); grid on;

% =========================================================================
% 核心计算函数
% =========================================================================
function K_pwm = LQR_k_calc(leg_length, V_batt, PWM_Max)
    % 1. 物理参数
    m_total = 0.4113;       % 总质量 (kg)
    m_wheel = 0.085;        % 单个轮组质量 (85g)
    M_body = m_total - 2 * m_wheel; 
    
    % [修正] 车轮半径 34mm
    r = 0.034;              
    
    I_wheel = 0.5 * m_wheel * r^2;  % 转动惯量
    l = leg_length;         % 重心高度
    g = 9.81;
    J_body = 0.000671;      % 车身Pitch惯量
    
    % 2. 电机参数 (BDS3620)
    Kt = 0.0191;    % N.m/A
    Rm = 1.0;       % Ohm
    
    % 力转换系数 (半径变大，同样的力矩产生的推力变小)
    force_per_volt = Kt / (Rm * r);
    
    % 3. 动力学 A 矩阵
    a = M_body + 2*m_wheel + 2*I_wheel/(r^2); 
    b = M_body * l;
    c = J_body + M_body * l^2;
    d = M_body * g * l;
    det = a*c - b*b;
    
    A23 = -b * d / det;
    A43 = a * d / det;
    
    A = [0 1  0   0;
         0 0 A23  0;
         0 0  0   1;
         0 0 A43  0];
     
    % 4. 输入矩阵 B
    scale_x = c + b*r; 
    scale_th = b + a*r;
    
    B_x = scale_x / det;
    B_th = scale_th / det;
    
    B_volt = [0; 
              2 * B_x * force_per_volt; 
              0; 
              2 * B_th * force_per_volt];
          
    % 5. 权重设置
    % R=34mm 比 R=33.75mm 稍微大一点，意味着同样转速下线速度更快，
    % 但力矩杠杆变长，电机稍微“吃力”一点点。
    %Q = diag([ 25,  0.75,  220,   1.2]); 
    % 降低角度刚性，增加角速度阻尼，略微降低速度敏感度
    Q = diag([ 40,  095,  90,   1.2]);
    % 位置：8速度：0.1角度：60角速度：0.8
    %Q = diag([8, 0.1,60,  0.8, 20]);  
    R = 1.2;          
    %R = 1.0;
    %Q = diag([ 25,  0.5,  220,   1.2 ]); 
    K_volts = lqr(A, B_volt, Q, R);
    K_pwm = K_volts * (PWM_Max / V_batt);
    K_pwm = [K_pwm; K_pwm]; 
end