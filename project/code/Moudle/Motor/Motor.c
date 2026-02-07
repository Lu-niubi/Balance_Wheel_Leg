#include "Motor.h"
#include <string.h> 
#include "controller.h" 
#include <math.h>

//-------------------------------------------------------------------------------------------------------------------
//  LQR 拟合参数 (BDS3620, 85g轮组, R=34mm)
//-------------------------------------------------------------------------------------------------------------------
// [修改点]：使用 MATLAB 生成的最新参数
// 注意：MATLAB 输出显示 k_x 为负值，k_theta 为正值，这由 LQR 求解器决定，保持原样即可。
float k_x_poly[4] = {0.000002, -0.000000, 0.000000, -4545.454545};
float k_v_poly[4] = {11223146.756673, -2518565.657087, 204284.812516, -9309.403979};
float k_theta_poly[4] = {2085657.222577, -688928.563716, 115088.793536, 14379.960975};
float k_omega_poly[4] = {-537135.911097, 133183.763695, -7332.475099, 1555.495022};

// -------------------------------------------------------------------------
// PID参数调优区 (根据实际车重和电机响应调整)
// -------------------------------------------------------------------------
// 1. 直立环参数 (输入: 弧度, 输出: PWM)
// Kp: 角度产生的回复力, Kd: 角速度产生的阻尼
#define BAL_KP  2800.0f  
#define BAL_KI  0.0f     // 直立环通常不需要积分，除非重心严重不对称
#define BAL_KD  20.0f    // 这里的D对应角速度

// 2. 速度环参数 (输入: 编码器速度, 输出: 期望角度弧度)
// 这里的输出是给直立环的“角度”，所以限幅要很小 (比如最大倾斜 0.1弧度 = 5.7度)
#define SPD_KP  0.003f
#define SPD_KI  0.00001f // 速度环需要积分来消除静差
#define SPD_KD  0.0f
#define SPD_MAX_PITCH  0.15f // 最大俯仰角限制 (弧度)

// 3. 转向环参数 (输入: 摇杆差值, 输出: PWM差分)
#define TURN_KP 15.0f
#define TURN_KI 0.0f
#define TURN_KD 0.5f

// 4. 手柄映射配置
#define JOYSTICK_DEADZONE 2000   // 摇杆死区 (防止漂移)
#define MAX_TARGET_SPEED  800.0f // 满油门时的最大编码器速度
#define MAX_TURN_SPEED    3000.0f // 满舵时的最大转向PWM分量
//-------------------------------------------------------------------------------------------------------------------
//  内部变量
//-------------------------------------------------------------------------------------------------------------------

//LQR内部变量声明
typedef struct {
    float x;        // 位移 (m)
    float v;        // 速度 (m/s)
    float theta;    // 角度 (rad)
    float omega;    // 角速度 (rad/s)
} Robot_State_t;

//PID内部变量声明
static Robot_State_t robot_state = {0}; 
static PIDInstance pid_balance; // 直立环
static PIDInstance pid_speed;   // 速度环
static PIDInstance pid_turn;    // 转向环

static float target_speed_filter = 0.0f; // 速度设定值滤波
static float actual_speed_filter = 0.0f; // 实际速度滤波
//-------------------------------------------------------------------------------------------------------------------
//  内部静态函数（辅助函数）
//-------------------------------------------------------------------------------------------------------------------
static int16_t Remove_Joystick_Deadzone(int16_t raw_val)
{
    if (abs(raw_val) < JOYSTICK_DEADZONE) return 0;
    // 简单的线性映射，去掉死区部分
    if (raw_val > 0) return raw_val - JOYSTICK_DEADZONE;
    else return raw_val + JOYSTICK_DEADZONE;
}
static int16_t clip_value(int16_t val, int16_t min, int16_t max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// y = ax^3 + bx^2 + cx + d
static float poly_eval(float* poly, float l) {
    return poly[0]*l*l*l + poly[1]*l*l + poly[2]*l + poly[3];
}

static int16_t deadzone_compensate(int16_t pwm_in) {
    // [调试提示]：BDS3620 负载变大(85g轮子)后，死区可能会变化，请实测。
    // 如果车在小角度时只有电流声不转，请加大这个值。
    int16_t deadzone = 300; 
    if (pwm_in > 0) return pwm_in + deadzone;
    if (pwm_in < 0) return pwm_in - deadzone;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  接口函数实现
//-------------------------------------------------------------------------------------------------------------------

void Motor_Init(void)
{
    small_driver_uart_init();
    Motor_Reset_State(); 
}

void Motor_Set_Duty(int16_t left_duty, int16_t right_duty)
{
    int16_t safe_left  = clip_value(left_duty, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    int16_t safe_right = clip_value(right_duty, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    small_driver_set_duty(safe_left, safe_right);
}

int16_t Motor_Get_Left_Speed(void)
{
    return motor_value.receive_left_speed_data;
}

int16_t Motor_Get_Right_Speed(void)
{
    return motor_value.receive_right_speed_data;
}
void Motor_Reset_State(void)
{
    //LQR重置
    robot_state.x = 0.0f;
    robot_state.v = 0.0f;
    robot_state.theta = 0.0f;
    robot_state.omega = 0.0f;
    //PID重置
    // 清空积分项，防止重启瞬间飞车
    pid_balance.ITerm = 0;
    pid_balance.Iout = 0;
    pid_speed.ITerm = 0;
    pid_speed.Iout = 0;
    pid_turn.ITerm = 0;
    pid_turn.Iout = 0;
    
    target_speed_filter = 0;
    actual_speed_filter = 0;
    Motor_Set_Duty(0, 0);
}
#ifdef USE_LQR_CONTROL
// LQR 平衡控制核心
void Motor_LQR_Balance_Control(float leg_length, float imu_pitch, float imu_gyro)
{
    // --- Step 1: 状态解算 ---
    int16_t speed_L_raw = Motor_Get_Left_Speed();
    int16_t speed_R_raw = Motor_Get_Right_Speed();
    
    // 转换为线速度 (m/s) = (RPM / 60) * 2 * PI * r
    float speed_L_ms = (speed_L_raw / 60.0f) * 2 * 3.14159f * WHEEL_RADIUS;
    float speed_R_ms = (speed_R_raw / 60.0f) * 2 * 3.14159f * WHEEL_RADIUS;
    float v_avg = (speed_L_ms + speed_R_ms) / 2.0f;
    
    robot_state.v = v_avg;
    robot_state.x += robot_state.v * CONTROL_DT; 
    robot_state.theta = imu_pitch; 
    robot_state.omega = imu_gyro;

    // --- Step 2: 倒地保护 ---
    // 角度过大 (约70度1.22/45度0.8) 停机
    if (fabs(imu_pitch) > 1.22f) {
        Motor_Set_Duty(0, 0);
        robot_state.x = 0; 
        small_driver_get_speed(); 
        return;
    }

    // --- Step 3: 增益调度 (计算 K) ---
    // [修改点]：限制范围必须匹配 MATLAB 的计算范围 (0.03 ~ 0.08)
    // 如果超出这个范围，多项式拟合会发散，算出极其离谱的 K 值导致飞车
    float l_safe = leg_length;
    if(l_safe < 0.03f) l_safe = 0.03f; // 最小值 3cm
    if(l_safe > 0.08f) l_safe = 0.08f; // 最大值 8cm

    float k_x     = poly_eval(k_x_poly, l_safe);
    float k_v     = poly_eval(k_v_poly, l_safe);
    float k_theta = poly_eval(k_theta_poly, l_safe);
    float k_omega = poly_eval(k_omega_poly, l_safe);

    // --- Step 4: LQR 输出计算 ---
    // u = -K * x
    // [极性检查重要提示]：
    // 如果车子往倒下的方向加速（比如前倾时轮子往后转，或者前倾时轮子猛冲导致飞出去），
    // 请去掉下面的负号，改为 float pwm_out_float = (...);
    float pwm_out_float = -(k_x * robot_state.x + 
                            k_v * robot_state.v + 
                            k_theta * robot_state.theta + 
                            k_omega * robot_state.omega);

    // --- Step 5: 死区与输出 ---
    int16_t pwm_final = (int16_t)pwm_out_float;
    pwm_final = deadzone_compensate(pwm_final);
    
    Motor_Set_Duty(pwm_final, pwm_final);

    // --- Step 6: 保持数据链路 ---
    small_driver_get_speed(); 
}
#endif

#ifdef USE_PID_CONTORL 
// -------------------------------------------------------------------------
// PID 初始化
// -------------------------------------------------------------------------
void Motor_PID_Init(void)
{
    Motor_Reset_State();

    PID_Init_Config_s config;
    
    // --- 1. 直立环初始化 ---
    config.Kp = BAL_KP;
    config.Ki = BAL_KI;
    config.Kd = BAL_KD;
    config.dt = CONTROL_DT; // 1ms
    config.MaxOut = MOTOR_MAX_DUTY; 
    config.DeadBand = 0;
    config.Improve = PID_Derivative_On_Measurement; // 使用微分先行(对反馈做微分)，效果等同于 PD 控制中的 D*Gyro
    config.Derivative_LPF_RC = 0.005f; // 微分滤波
    config.Output_LPF_RC = 0.0f;
    PIDInit(&pid_balance, &config);

    // --- 2. 速度环初始化 ---
    config.Kp = SPD_KP;
    config.Ki = SPD_KI;
    config.Kd = SPD_KD;
    config.dt = CONTROL_DT * 10; // 假设每 10ms 跑一次速度环
    config.MaxOut = SPD_MAX_PITCH; // 速度环输出的是角度，一定要限幅！
    config.IntegralLimit = 0.05f;  // 积分限幅
    config.Improve = PID_Integral_Limit | PID_OutputFilter;
    config.Output_LPF_RC = 0.1f;   // 让角度变化平滑一点
    PIDInit(&pid_speed, &config);

    // --- 3. 转向环初始化 ---
    config.Kp = TURN_KP;
    config.Ki = TURN_KI;
    config.Kd = TURN_KD;
    config.dt = CONTROL_DT;
    config.MaxOut = MOTOR_MAX_DUTY / 2; // 转向占一半动力
    config.Improve = PID_Derivative_On_Measurement;
    PIDInit(&pid_turn, &config);
}

// -------------------------------------------------------------------------
// PID 平衡控制核心 (需在 pit0_ch0_isr 中调用)
// 参数: 
//   imu_pitch_rad: 当前俯仰角(弧度)
//   imu_gyro_rad:  当前俯仰角速度(rad/s) -> 对应 Gyro X
//   imu_yaw_gyro_rad: 当前航向角速度(rad/s) -> 对应 Gyro Z
// -------------------------------------------------------------------------
void Motor_PID_Balance_Control(float imu_pitch_rad, float imu_gyro_rad, float imu_yaw_gyro_rad)
{
    static uint8_t speed_loop_cnt = 0;
    float balance_out = 0.0f;
    float velocity_out_pitch = 0.0f;
    float turn_out = 0.0f;

    // ============================================================
    // 0. 安全保护 (倒地检测)
    // ============================================================
    // 如果倾角超过 45度 (约0.78弧度) 或按下手柄 A 键，强制停机
    if (fabsf(imu_pitch_rad) > 0.78f || IPCS->M1_Pub.xbox_btn_a == 1)
    {
        Motor_Reset_State();
        small_driver_get_speed(); // 保持心跳
        return;
    }

    // ============================================================
    // 1. 速度环 (每 10ms 执行一次)
    // ============================================================
    speed_loop_cnt++;
    if (speed_loop_cnt >= 10) 
    {
        speed_loop_cnt = 0;

        // 1.1 获取手柄输入 (左摇杆 Y轴控制前后)
        // Xbox摇杆范围 int16 (-32768 ~ 32767)
        int16_t joy_y = IPCS->M1_Pub.xbox_joy_l_vert;
        joy_y = Remove_Joystick_Deadzone(joy_y);
        
        // 目标速度映射 (假设摇杆推到底对应 800 编码器速度)
        // 注意方向：假设摇杆向上是正数，如果反了在这里加负号
        float target_speed_raw = ((float)joy_y / 32768.0f) * MAX_TARGET_SPEED;
        
        // 简单平滑处理
        target_speed_filter = 0.9f * target_speed_filter + 0.1f * target_speed_raw;

        // 1.2 获取反馈速度 (左右轮平均)
        float current_speed = (Motor_Get_Left_Speed() + Motor_Get_Right_Speed()) / 2.0f;
        // 简单低通滤波 (编码器噪声大)
        actual_speed_filter = 0.7f * actual_speed_filter + 0.3f * current_speed;

        // 1.3 速度 PID 计算
        // 通常：加速 -> 车轮要跑得比车身快 -> 车身看起来像后仰 -> 产生前向力矩
        // 这部分极性非常关键！如果车子越跑越快停不下来，尝试把这里的符号反一下
        velocity_out_pitch = PIDCalculate(&pid_speed, actual_speed_filter, target_speed_filter);
    }
    else
    {
        // 非采样周期，维持上一次的计算结果
        velocity_out_pitch = pid_speed.Output; 
    }

    // ============================================================
    // 2. 转向环 (每 1ms 执行)
    // ============================================================
    // 2.1 获取手柄输入 (右摇杆 X轴控制左右)
    int16_t joy_x = IPCS->M1_Pub.xbox_joy_r_hori;
    joy_x = Remove_Joystick_Deadzone(joy_x);

    // 2.2 目标转向速度
    float target_turn_rate = ((float)joy_x / 32768.0f) * MAX_TURN_SPEED;

    // 2.3 转向 PID 计算
    // 目标是转向速度，反馈是 Gyro Z (imu_yaw_gyro_rad 是弧度，可能需要缩放匹配 Joy)
    // 简单起见，这里直接做开环控制叠加 Gyro 阻尼
    // 也就是：PWM = 摇杆分量 + Gyro阻尼
    // 或者使用 PID 库：
    // Measure = imu_yaw_gyro_rad * 系数 (转成对应的量级)
    // 这里简化逻辑：直接用 PID 算，Kp 控制灵敏度，Kd 抑制摆动
    turn_out = PIDCalculate(&pid_turn, imu_yaw_gyro_rad * 500.0f, target_turn_rate);

    // ============================================================
    // 3. 直立环 (每 1ms 执行) - 核心
    // ============================================================
    
    // 期望角度 = 机械中值(0) - 速度环输出
    // 减号是因为：通常PID计算正速度误差输出正值，而让车前行通常需要给负的期望角度(前倾)
    // 或者是 PWM 输出给正值车轮前转。
    // *调试技巧*: 先把 velocity_out_pitch 设为 0，调平直立环。然后加手柄，如果推油门车子反而后退，改这里符号。
    float target_pitch = 0.0f - velocity_out_pitch;

    // 计算直立 PWM
    balance_out = PIDCalculate(&pid_balance, imu_pitch_rad, target_pitch);

    // ============================================================
    // 4. 电机输出混合
    // ============================================================
    
    // 叠加直立和转向
    float motor_l = balance_out + turn_out;
    float motor_r = balance_out - turn_out;

    // 死区补偿 (复制你 LQR 里的逻辑)
    int16_t final_l = deadzone_compensate((int16_t)motor_l);
    int16_t final_r = deadzone_compensate((int16_t)motor_r);

    Motor_Set_Duty(final_l, final_r);

    // 保持链路
    small_driver_get_speed();
}
#endif