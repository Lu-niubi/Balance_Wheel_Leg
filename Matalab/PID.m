% =========================================================================
% 轮腿小车 串级PID 粒子群优化 (PSO) 独立脚本
% =========================================================================
clear; clc; close all;

%% 1. 算法参数与搜索空间设置
% 优化的变量顺序: [角速度环Kp, 角度环Kp, 角度环Kd, 速度环Kp, 速度环Ki]
% 注：根据你的C代码，内环Kd和速度环Kd通常为0，这里简化搜索空间以提高收敛率
dim = 5;  
pop_size = 30;      % 粒子数量 (种群大小)
max_iter = 50;      % 最大迭代次数

% 搜索边界 (根据你C代码的经验值设定，防止由于系统发散导致求出无效解)
%         [Gyro_Kp, Ang_Kp, Ang_Kd, Spd_Kp, Spd_Ki]
ub =      [ 600.0,   3.0,    0.5,    0.3,    0.01 ]; % 上限
lb =      [ 200.0,   0.5,    0.0,    0.01,   0.0  ]; % 下限

%% 2. 粒子群初始化
Vmax = (ub - lb) * 0.15; 
pos = repmat(lb, pop_size, 1) + rand(pop_size, dim) .* repmat((ub - lb), pop_size, 1);
vel = zeros(pop_size, dim);

pBest_pos = pos;
pBest_score = inf(pop_size, 1);
gBest_pos = zeros(1, dim);
gBest_score = inf;

Convergence_curve = zeros(1, max_iter);
fprintf('开始 PSO 优化串级 PID 参数...\n');

%% 3. PSO 主循环
for iter = 1:max_iter
    for i = 1:pop_size
        % 边界约束
        pos(i,:) = max(min(pos(i,:), ub), lb);
        
        % 计算适应度
        fitness = Objective_WheeledLeg(pos(i,:));
        
        % 更新个体最优
        if fitness < pBest_score(i)
            pBest_score(i) = fitness;
            pBest_pos(i,:) = pos(i,:);
        end
        
        % 更新全局最优
        if fitness < gBest_score
            gBest_score = fitness;
            gBest_pos = pos(i,:);
        end
    end
    
    % 动态惯性权重
    w = 0.9 - iter * (0.4 / max_iter); 
    c1 = 1.5; c2 = 1.5;
    
    % 更新速度与位置
    for i = 1:pop_size
        vel(i,:) = w * vel(i,:) ...
                 + c1 * rand(1,dim) .* (pBest_pos(i,:) - pos(i,:)) ...
                 + c2 * rand(1,dim) .* (gBest_pos - pos(i,:));
        vel(i,:) = max(min(vel(i,:), Vmax), -Vmax);
        pos(i,:) = pos(i,:) + vel(i,:);
    end
    
    Convergence_curve(iter) = gBest_score;
    fprintf('迭代次数: %d/%d, 当前最优代价: %.4f\n', iter, max_iter, gBest_score);
end

%% 4. 结果展示
fprintf('\n// ===== 优化完成，请将以下参数填入 C 代码 =====\n');
fprintf('#define GYRO_KP   %.2ff\n', gBest_pos(1));
fprintf('#define GYRO_KI   0.0f\n');
fprintf('#define GYRO_KD   0.0f\n\n');

fprintf('#define ANG_KP    %.2ff\n', gBest_pos(2));
fprintf('#define ANG_KI    0.0f  // 建议保持为0\n');
fprintf('#define ANG_KD    %.2ff\n\n', gBest_pos(3));

fprintf('#define SPD_KP    %.2ff\n', gBest_pos(4));
fprintf('#define SPD_KI    %.4ff\n', gBest_pos(5));
fprintf('#define SPD_KD    0.0f\n');
fprintf('// ===============================================\n');

% 绘制收敛曲线与最优响应
figure('Name', 'PSO 优化结果');
subplot(2,1,1);
plot(Convergence_curve, 'LineWidth', 2);
title('PSO 适应度收敛曲线'); xlabel('迭代次数'); ylabel('Cost (ITAE)'); grid on;

% 运行最优参数获取轨迹
[~, y_pitch, y_speed, t] = Objective_WheeledLeg(gBest_pos);
subplot(2,1,2);
plot(t, y_pitch, 'r', 'LineWidth', 1.5); hold on;
plot(t, y_speed, 'b', 'LineWidth', 1.5);
title('最优 PID 参数下的系统阶跃响应 (初始倾角扰动)');
legend('Pitch 角度 (rad)', '小车线速度 (m/s)');
xlabel('Time (s)'); grid on;


% =========================================================================
% 目标函数：模拟小车动力学与三闭环离散 PID
% =========================================================================
function [cost, pitch_hist, speed_hist, time_hist] = Objective_WheeledLeg(x)
    % 提取 PID 参数
    gyro_kp = x(1);
    ang_kp  = x(2); ang_kd = x(3);
    spd_kp  = x(4); spd_ki = x(5);
    
    % --- 物理模型建立 (来自你的 LQR 脚本) ---
    m_total = 0.4113; m_wheel = 0.085; M_body = m_total - 2 * m_wheel;
    r = 0.034; I_wheel = 0.5 * m_wheel * r^2;
    l = 0.04123; % 选取中间腿长 5cm 作为 PID 整定的基准点
    g = 9.81; J_body = 0.000671;
    Kt = 0.0191; Rm = 1.0; force_per_volt = Kt / (Rm * r);
    
    a = M_body + 2*m_wheel + 2*I_wheel/(r^2);
    b = M_body * l; c = J_body + M_body * l^2; d = M_body * g * l;
    det_val = a*c - b*b;
    
    A = [0 1  0   0;
         0 0 -b*d/det_val 0;
         0 0  0   1;
         0 0 a*d/det_val  0];
         
    scale_x = c + b*r; scale_th = b + a*r;
    B_volt = [0; 2 * scale_x/det_val * force_per_volt; 0; 2 * scale_th/det_val * force_per_volt];
    
    % 将 B 矩阵输入从电压转换为 PWM (-10000 ~ 10000)
    V_batt = 11.0; PWM_Max = 10000;
    B_pwm = B_volt * (V_batt / PWM_Max);
    
    % --- 离散时间仿真设置 ---
    dt = 0.001;          % 仿真步长 1ms
    T_sim = 2.0;         % 仿真时间 2 秒
    steps = round(T_sim / dt);
    
    % 初始状态 [x; v; theta; omega]
    % 给一个 0.2 rad (约 11.4 度) 的初始倾角作为扰动，测试恢复能力
    X_state = [0; 0; 0.2; 0]; 
    
    % PID 历史变量
    spd_i_term = 0; target_pitch = 0; target_gyro = 0;
    
    % 记录数组
    cost = 0;
    pitch_hist = zeros(steps, 1);
    speed_hist = zeros(steps, 1);
    time_hist = zeros(steps, 1);
    
    for k = 1:steps
        actual_speed = X_state(2);
        actual_pitch = X_state(3);
        actual_gyro  = X_state(4);
        
        % 1. 速度环 (10ms 执行一次)
        if mod(k, 10) == 1 || k == 1
            spd_err = 0 - actual_speed; % 目标速度为 0 (原地平衡)
            spd_i_term = spd_i_term + spd_err * spd_ki;
            % 积分限幅
            spd_i_term = max(min(spd_i_term, 2.0), -2.0);
            
            target_pitch = spd_kp * spd_err + spd_i_term;
            % C 代码里的输出限幅 SPD_MAX_PITCH 约 12 度 = 0.21 rad
            target_pitch = max(min(target_pitch, 0.21), -0.21);
        end
        
        % 2. 角度环 (5ms 执行一次)
        if mod(k, 5) == 1 || k == 1
            ang_err = target_pitch - actual_pitch;
            % PD 控制，微分项直接使用测量的角速度实际值，减小高频噪声
            target_gyro = ang_kp * ang_err - ang_kd * actual_gyro; 
            target_gyro = max(min(target_gyro, 15.0), -15.0);
        end
        
        % 3. 角速度环 (1ms 执行一次)
        gyro_err = target_gyro - actual_gyro;
        pwm_out = gyro_kp * gyro_err;
        pwm_out = max(min(pwm_out, PWM_Max), -PWM_Max);
        
        % 4. 状态更新 (欧拉法积分求解 dx = Ax + Bu)
        dX = A * X_state + B_pwm * pwm_out;
        X_state = X_state + dX * dt;
        
        % 5. 适应度函数 (ITAE 标准：时间乘绝对误差)
        % 惩罚项权重：角度偏离(最高优先级) > 速度偏离 > 控制器努力程度
        time = k * dt;
        cost = cost + (abs(actual_pitch) * 100 + abs(actual_speed) * 10) * time + abs(pwm_out) * 0.001;
        
        % 如果倾角过大(超过45度)，直接判定为倒地，赋予极大的惩罚值并终止
        if abs(actual_pitch) > 0.8 
            cost = cost + 1e6;
            break;
        end
        
        % 记录数据
        pitch_hist(k) = actual_pitch;
        speed_hist(k) = actual_speed;
        time_hist(k) = time;
    end
end