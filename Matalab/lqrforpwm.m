% =========================================================================
% LQR 参数计算器 - 针对 BDS3620 电机 & PWM 10000 定制版
% =========================================================================
clear; clc;

% --- 用户设置 ---
% 电池电压 (非常重要！如果你的电池是3S充满(12.6V)或标称(11.1V)，请修改这里)
V_batt = 11.3; 
% PWM 最大值 (定时器ARR值)
PWM_Max = 10000; 

% --- 腿长范围 (根据你的机械结构调整) ---
leg_range = 0.06 : 0.01 : 0.16; 
len_num = length(leg_range);
K_data = zeros(8, len_num); 

fprintf('正在针对 BDS3620 电机计算 LQR 增益...\n');

for i = 1:len_num
    % 调用计算函数
    K_curr = LQR_k_calc(leg_range(i), V_batt, PWM_Max);
    
    % 存储数据
    K_data(1:4, i) = K_curr(1, :); % 第一行: k_x, k_v, k_theta, k_omega
    K_data(5:8, i) = K_curr(2, :); % 第二行 (通常左右轮对称，取一组即可)
end

% --- 多项式拟合 ---
Poly_Coeffs = zeros(8, 4); 
for row = 1:8
    Poly_Coeffs(row, :) = polyfit(leg_range, K_data(row, :), 3);
end

% --- 生成 C 代码 ---
fprintf('\n// ===== 请将以下参数复制到单片机代码中 (BDS3620定制) =====\n');
fprintf('// 电池电压: %.1fV, PWM范围: 0-%d\n', V_batt, PWM_Max);
var_names = {'k_x', 'k_v', 'k_theta', 'k_omega'}; % 对应状态变量

% 注意：LQR标准形式 u = -Kx。
% 我们通常只取第一行（假设左右轮一致），这里输出前4个参数
for i = 1:4
    coeffs = Poly_Coeffs(i, :);
    fprintf('float %s_poly[4] = {%.6f, %.6f, %.6f, %.6f};\n', ...
        var_names{i}, coeffs(1), coeffs(2), coeffs(3), coeffs(4));
end
fprintf('// ========================================================\n');

% 绘图验证
figure(1);
subplot(2,2,1); plot(leg_range, K_data(3,:), 'o-'); title('角度增益 K\_theta'); grid on;
subplot(2,2,2); plot(leg_range, K_data(4,:), 'o-'); title('角速度增益 K\_omega'); grid on;
subplot(2,2,3); plot(leg_range, K_data(1,:), 'o-'); title('位移增益 K\_x'); grid on;
subplot(2,2,4); plot(leg_range, K_data(2,:), 'o-'); title('速度增益 K\_v'); grid on;


% =========================================================================
% 核心计算函数
% =========================================================================
function K_pwm = LQR_k_calc(leg_length, V_batt, PWM_Max)
    % 1. 物理参数
    m = 0.178;      % 车轮质量 kg
    r = 0.03375;    % 车轮半径 m
    M = 0.875;      % 车体质量 kg (包含电机)
    I = 0.5*m*r^2;  % 车轮转动惯量
    l = leg_length; % 质心高度
    g = 9.8;
    Jz = 0.0015 + M * (l - 0.05)^2; % 估算的转动惯量

    % 2. BDS3620 电机参数 (基于图片推导)
    % KV = 500 -> Ke = 0.0191
    % Stall 12A @ 12V -> R = 1.0 Ohm
    Kt = 0.0191;    % N.m/A
    Rm = 1.0;       % Ohm (估计值，如果有万用表实测更佳)
    
    % 电压到力的增益系数 (Force / Voltage)
    % F = T/r = (Kt/R * V) / r
    force_per_volt = Kt / (Rm * r);

    % 3. 状态空间矩阵 (A矩阵-动力学)
    a = r*(M + 2*m + 2*I/(r^2));
    b = M*r*l;
    c = Jz + M*l^2;
    d = M*g*l;
    e = M*l;
    den = a*c - b*e;

    A23 = -b*d / den;
    A43 = a*d / den;
    
    % 反电动势产生的各种阻尼项 (简化处理，实际上会增加A矩阵对角线元素)
    % 严谨的LQR通常忽略电机电感，但不能忽略反电动势阻尼
    % 阻尼力矩 T_damping = -(Kt*Ke/R) * omega_wheel
    % 这里我们暂时保留原始A矩阵，依靠B矩阵的准确性来控制
    
    A = [0 1 0 0;
         0 0 A23 0;
         0 0 0 1;
         0 0 A43 0];
     
    % 4. 输入矩阵 (B矩阵-电压控制)
    B2_force = (c+b) / den;
    B4_force = -(e+a) / den;
    
    % 将力(Force)转换为电压(Voltage)
    % 乘以2是因为有两个电机同时用力
    B_volt = [0; 
              2 * B2_force * force_per_volt; 
              0; 
              2 * B4_force * force_per_volt];
          
    % 5. LQR 权重 (针对电压控制优化)
    % Q矩阵: [x, v, theta, omega]
    % 角度(theta)权重最大(200)，位移(x)次之(10)
    Q = diag([10, 0.1, 200, 1]); 
    
    % R矩阵: 对电压的惩罚
    % 因为你的PWM很大(10000)，我们需要算出的K比较小，或者后续再乘
    % 这里我们先算对"Volts"的K，R设为1比较合理
    R = 1; 

    % 计算电压增益 K_volts = [k_x, k_v, k_theta, k_omega]
    K_volts = lqr(A, B_volt, Q, R);
    
    % 6. 转换为 PWM 增益
    % K_pwm * PWM_val = K_volts * Volts
    % PWM_val = (Volts / V_batt) * PWM_Max
    % 所以 K_pwm = K_volts * (V_batt / PWM_Max) ... 等等，反了
    % 我们想要 u_pwm = -K_pwm * x
    % u_pwm = u_volts * (PWM_Max / V_batt)
    % u_pwm = (-K_volts * x) * (PWM_Max / V_batt)
    % 所以 K_pwm = K_volts * (PWM_Max / V_batt)
    
    K_pwm = K_volts * (PWM_Max / V_batt);
    
    % 构造2行输出 (假设需要传给函数的是2x4矩阵)
    K_pwm = [K_pwm; K_pwm]; 
end