% =========================================================================
% 轮腿小车 PID 优化 - 【地狱级越野颠簸路况微调版】
% =========================================================================
clear; clc; close all;

%% 1. 初始锚点设定 (使用你实测偏柔和的参数作为起点)
x_initial = [380.0, 1.20, 0.045, 0.09, 0.10, 0.001];

% 定义微调边界 (给 D 项留出极大的上限，逼迫它学会“吸收震动”)
ub = [ 500.0,   2.00,   0.08,   0.30,   0.20,   0.002 ]; 
lb = [ 200.0,   0.80,   0.02,   0.01,   0.05,   0.000 ]; 

fprintf('==================================================\n');
fprintf('开启越野模式：持续注入[碎石高频振动]与[随机大坑冲击]\n');
fprintf('==================================================\n\n');

%% 2. 模拟退火算法核心
T_init = 100.0; T_min = 0.01; alpha = 0.95; markov_len = 20;
best_pos = x_initial; current_pos = x_initial;
current_cost = Objective_OffRoad(current_pos);
best_cost = current_cost;
T = T_init; history_cost = [];

while T > T_min
    for i = 1:markov_len
        step_size = (ub - lb) .* (T / T_init) * 0.2; 
        new_pos = current_pos + (rand(1, 6) * 2 - 1) .* step_size;
        new_pos = max(min(new_pos, ub), lb);
        
        new_cost = Objective_OffRoad(new_pos);
        
        delta = new_cost - current_cost;
        if delta < 0
            current_pos = new_pos; current_cost = new_cost;
            if current_cost < best_cost
                best_cost = current_cost; best_pos = current_pos;
            end
        else
            if rand() < exp(-delta / T)
                current_pos = new_pos; current_cost = new_cost;
            end
        end
    end
    history_cost = [history_cost, best_cost];
    fprintf('当前温度: %.2f, 最佳抗颠簸代价: %.4f\n', T, best_cost);
    T = T * alpha; 
end

%% 3. 输出越野级平滑参数
fprintf('\n// ===== 经过地狱越野路况淬炼的最优参数 =====\n');
fprintf('#define GYRO_KP   %.2ff\n', best_pos(1));
fprintf('#define GYRO_KI   0.0f\n');
fprintf('#define GYRO_KD   0.0f\n\n');

fprintf('#define ANG_KP    %.2ff\n', best_pos(2));
fprintf('#define ANG_KI    %.4ff\n', best_pos(3));
fprintf('#define ANG_KD    %.4ff\n\n', best_pos(4));

fprintf('#define SPD_KP    %.4ff\n', best_pos(5));
fprintf('#define SPD_KI    %.4ff\n', best_pos(6));
fprintf('#define SPD_KD    0.0f\n');
fprintf('// ===============================================\n');

%% 4. 绘图验证
figure('Name', '越野路况微调结果', 'Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(history_cost, 'g', 'LineWidth', 2); title('抗颠簸代价收敛曲线'); grid on;

[~, y_pitch, y_speed, t, valid_steps, real_pwm] = Objective_OffRoad(best_pos);
subplot(2,1,2);
yyaxis left;
plot(t(1:valid_steps), y_pitch(1:valid_steps), 'r', 'LineWidth', 1.5); hold on;
plot(t(1:valid_steps), y_speed(1:valid_steps), 'b', 'LineWidth', 1.5);
ylabel('状态响应 (rad & m/s)');
yyaxis right;
plot(t(1:valid_steps), real_pwm(1:valid_steps), 'k--', 'LineWidth', 1);
ylabel('电机真实输出 PWM');
title('底盘在连续碎石与大坑冲击下的真实姿态表现');
legend('Pitch 角度', '线速度', '电机 PWM'); grid on;

% =========================================================================
% 目标函数：注入连续碎石白噪声与脉冲冲击
% =========================================================================
function [cost, pitch_hist, speed_hist, time_hist, valid_steps, pwm_hist] = Objective_OffRoad(x)
    gyro_kp = x(1); ang_kp = x(2); ang_ki = x(3); ang_kd = x(4);
    spd_kp = x(5); spd_ki = x(6);
    
    m_total = 0.4113; m_wheel = 0.085; M_body = m_total - 2 * m_wheel;
    r = 0.034; l = 0.05; g = 9.81; J_body = 0.000671; I_wheel = 0.5 * m_wheel * r^2;
    Kt = 0.0191; Rm = 1.0; force_per_volt = Kt / (Rm * r);
    a = M_body + 2*m_wheel + 2*I_wheel/(r^2); b = M_body * l; c = J_body + M_body * l^2; d = M_body * g * l;
    det_val = a*c - b*b; scale_x = c + b*r; scale_th = b + a*r;
    
    Total_damping = 2 * (Kt^2) / (Rm * r^2) + 0.8; % 越野泥地摩擦力更大
    A = [0 1 0 0; 0 -(Total_damping * scale_x)/det_val -b*d/det_val 0; 0 0 0 1; 0 (Total_damping * scale_th)/det_val a*d/det_val 0];
    B_volt = [0; 2 * scale_x/det_val * force_per_volt; 0; -2 * scale_th/det_val * force_per_volt]; 
    V_batt = 11.0; PWM_Max = 10000; B_pwm = B_volt * (V_batt / PWM_Max);
    
    dt = 0.001; T_sim = 3.0; steps = round(T_sim / dt); % 延长测试时间到 3 秒
    X_state = [0; 0; 0; 0]; 
    
    spd_i_term = 0; ang_i_term = 0; target_pitch = 0; target_gyro = 0;
    actual_pwm = 0; cost = 0; last_pwm = 0;
    
    pitch_hist = zeros(steps, 1); speed_hist = zeros(steps, 1); 
    time_hist = zeros(steps, 1); pwm_hist = zeros(steps, 1);
    valid_steps = steps;
    
    for k = 1:steps
        time = k * dt;
        
        % 【地狱路况生成器】
        % 1. 连续的碎石路面 (高频白噪声，持续影响车身角速度)
        gravel_noise = 0.05 * randn(); 
        X_state(4) = X_state(4) + gravel_noise * dt;
        
        % 2. 随机的深坑/砖块冲击 (每隔 0.6 秒左右，突然给底盘一个极大的物理冲量)
        if mod(time, 0.6) < dt && time > 0.1
            X_state(3) = X_state(3) + 0.12; % 瞬间被顶起 约 7 度
            X_state(4) = X_state(4) + 1.5;  % 伴随极大的角加速度
        end
        
        meas_speed = X_state(2);
        meas_pitch = X_state(3); 
        meas_gyro  = X_state(4);
        
        if mod(k, 10) == 1 || k == 1
            spd_err = 0 - meas_speed; 
            spd_i_term = max(min(spd_i_term + spd_err * spd_ki, 2.0), -2.0); 
            target_pitch = max(min(spd_kp * spd_err + spd_i_term, 0.25), -0.25); 
        end
        
        if mod(k, 5) == 1 || k == 1
            ang_err = target_pitch - meas_pitch;
            ang_i_term = max(min(ang_i_term + ang_err * ang_ki, 5.0), -5.0); 
            target_gyro = max(min(ang_kp * ang_err + ang_i_term - ang_kd * meas_gyro, 15.0), -15.0); 
        end
        
        gyro_err = target_gyro - meas_gyro;
        pwm_cmd = -(gyro_kp * gyro_err);
        pwm_cmd = max(min(pwm_cmd, PWM_Max), -PWM_Max);
        
        tau_motor = 0.008;
        actual_pwm = actual_pwm + (pwm_cmd - actual_pwm) * (dt / tau_motor);
        
        dX = A * X_state + B_pwm * actual_pwm;
        X_state = X_state + dX * dt;
        
        % 代价打分：在颠簸路面，允许车身有一定晃动，但严厉惩罚“发散”和“电机剧烈抖动”
        delta_pwm = abs(actual_pwm - last_pwm);
        cost = cost + abs(X_state(3))*50 + abs(X_state(2))*20 + delta_pwm*1.0;
        last_pwm = actual_pwm;
        
        pitch_hist(k) = X_state(3); speed_hist(k) = X_state(2); 
        time_hist(k) = time; pwm_hist(k) = actual_pwm;
        
        if abs(X_state(3)) > 0.8 
            cost = cost + 1e7; 
            valid_steps = k;
            break;
        end
    end
end