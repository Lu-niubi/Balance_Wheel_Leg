#include "Motor.h"
#include <string.h> 
#include "controller.h" 
#include <math.h>
#include "ShareData.h"
#include "KSCal.h"
//-------------------------------------------------------------------------------------------------------------------
//  LQR 拟合参数 (BDS3620, 85g轮组, R=34mm)
//-------------------------------------------------------------------------------------------------------------------
// [修改点]：使用 MATLAB 生成的最新参数
// 注意：MATLAB 输出显示 k_x 为负值，k_theta 为正值，这由 LQR 求解器决定，保持原样即可。
float k_x_poly[4] = {0.000003, -0.000000, 0.000000, -5248.638811};
float k_v_poly[4] = {5114683.336813, -1116363.941160, 85004.353155, -11376.839187};
float k_theta_poly[4] = {-466039.789735, -594529.919467, 293048.266096, 8187.207806};
float k_omega_poly[4] = {-480106.781393, 144801.055782, -409.647554, 1531.209091};
float k_out[4] = {0,0,0,0};
// -------------------------------------------------------------------------
// PID参数调优区 (根据实际车重和电机响应调整)
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// LQR 速度环 & 转向环 PID 参数 (与 PID 模式对齐)
// -------------------------------------------------------------------------
#define LQR_SPD_KP         0.10f
#define LQR_SPD_KI         0.001f
#define LQR_SPD_KD         0.0f
#define LQR_SPD_MAX_PITCH  12.0f   // 速度环最大允许输出的角度偏移 (度)

#define LQR_TURN_KP        1.0f
#define LQR_TURN_KI        0.0f
#define LQR_TURN_KD        0.05f
#define LQR_MAX_TURN_SPEED 2000.0f // 满舵时最大转向 PWM 差分

// -------------------------------------------------------------------------
// 串级 PID 参数定义 (需重新调参)
// -------------------------------------------------------------------------

// 1. 最内环：角速度环 (输入: rad/s, 输出: PWM)
// 只有这一环直接控制电机，Kp决定了电机对旋转速度的响应快慢
// #define GYRO_KP   200.0f   // 典型值：根据 PWM 量程调整         480 580
// #define GYRO_KI   0.0f     // 也就是图片中的 "角速度环 PD"
// #define GYRO_KD   0.0f     // 抑制高频噪声

// // 2. 中间环：角度环 (输入: 角度/弧度, 输出: 目标角速度 rad/s)
// // 这一环决定了车回正的"欲望"强弱，输出的是给内环的指令
// #define ANG_KP    1.0f     // 典型值：输出是 rad/s，所以不会很大    1.0 1.4 1.5/1.2
// #define ANG_KI    0.04f     //0.55  0.04
// #define ANG_KD    0.09f     // 通常设为0，因为微分作用由内环(角速度)承担了 // 0.01//1.88 0.09

// 脚本调参
  #define GYRO_KP    191.433409f
  #define ANG_KP     1.321422f
  #define ANG_KI     0.005641f
  #define ANG_KD     0.0508300f
  #define GYRO_KI    0.0f     
  #define GYRO_KD    0.0f

// 3. 最外环：速度环 (输入: 速度, 输出: 目标角度)
#define SPD_KP    0.08f   //非常小
#define SPD_KI    0.005f  // 积分也要很小
#define SPD_KD    0.0f     // 速度环通常不需要 D
#define SPD_MAX_PITCH  20.0f // 速度环最大允许输出多少度倾角安全限制

// 4. 转向环参数 (输入: 摇杆差值, 输出: PWM差分)
#define TURN_KP 1.0f
#define TURN_KI 0.0f
#define TURN_KD 0.05f

// 5. 手柄映射配置
#define JOYSTICK_DEADZONE 2000   // 摇杆死区 (防止漂移)
#define MAX_TARGET_SPEED  5000.0f // 满油门时的最大编码器速度
#define MAX_TURN_SPEED    2000.0f // 满舵时的最大转向PWM分量

//全局变量
int16_t Left_motor_duty = 0;
int16_t Right_motor_duty = 0;
float outtest = 0;
float y,x = 0;
float Leg = 0;
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
static Robot_State_t robot_state = {0};

// PID内部变量声明
static PIDInstance pid_velo;   // 速度环 (最外层)
static PIDInstance pid_angle;  // 角度环 (中间层)
static PIDInstance pid_gyro;   // 角速度环 (最内层)
static PIDInstance pid_turn;    // 转向环

// LQR 模式专用 PID 实例
static PIDInstance pid_lqr_speed; // LQR 速度环 (输出: 目标角度偏移 rad)
static PIDInstance pid_lqr_turn;  // LQR 转向环 (输出: PWM 差分)

static float target_speed_filter = 0.0f; // 速度设定值滤波
static float actual_speed_filter = 0.0f; // 实际速度滤波

// LQR 模式内部状态
static float lqr_target_pitch_offset = 0.0f; // 速度环输出的目标角度偏移 (rad)
static float lqr_turn_out = 0.0f;             // 转向环输出的 PWM 差分
//-------------------------------------------------------------------------------------------------------------------
//  内部静态函数（辅助函数）
//-------------------------------------------------------------------------------------------------------------------

// ==================================================================
// 处理奇怪的摇杆数值映射
// ==================================================================
// 输入：原始摇杆值 (int16)
// 输出：标准化后的浮点数 (-1.0f ~ 1.0f)，对应 (-Max ~ +Max)
static float Map_Strange_Joystick(int16_t raw_val, int is_vertical)
{
    float mapped_val = 0.0f;

    // 逻辑解析：
    // 正数区域 [0, 32767]：数值越小，油门越大 (32767是停, 0是满)
    // 负数区域 [-32768, -1]：数值越大，油门越大 (-32768是停, -1是满)
    
    if (raw_val >= 0) {
        // 处理正数区域 (前进/左转)
        // 32767 - raw_val 将反转逻辑：32767->0, 0->32767
        mapped_val = (float)(32767 - raw_val);
    } else {
        // 处理负数区域 (后退/右转)
        // raw_val + 32768 将范围从 [-32768, -1] 映射到 [0, 32767]
        // 也就是 -32768 -> 0 (停), -1 -> 32767 (满)
        // 因为是反向，所以最后取负号
        mapped_val = -((float)(raw_val + 32768));
    }

    // 归一化到 -1.0 ~ 1.0
    // 32767.0f 是最大量程
    float normalized = mapped_val / 32767.0f;

    // 死区处理 (软件死区，数值在 -0.05 ~ 0.05 之间强制为0)
    if (fabsf(normalized) < 0.05f) return 0.0f;

    return normalized;
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

//-------------------------------------------------------------------------------------------------------------------
//  接口函数实现
//-------------------------------------------------------------------------------------------------------------------

void Motor_Init(void)
{
    small_driver_uart_init();
    Motor_Reset_State();
    #ifdef USE_LQR_CONTROL
    Motor_LQR_Init();
    #endif
    #ifdef USE_PID_CONTROL
    Motor_PID_Init();
    #endif
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
    return -motor_value.receive_right_speed_data;
}
void Motor_Reset_State(void)
{
    // --- 1. LQR 状态重置 (保留以备切换) ---
    robot_state.x = 0.0f;
    robot_state.v = 0.0f;
    robot_state.theta = 0.0f;
    robot_state.omega = 0.0f;

    // --- 2. 串级 PID 状态重置 ---
    // 必须清空所有环的积分项、历史误差和输出，防止重启瞬间"飞车"

    // 2.1 最外环：速度环 (pid_velo)
    pid_velo.ITerm = 0;
    pid_velo.Iout = 0;
    pid_velo.Output = 0;      // 关键：清空输出，防止开机就有目标角度
    pid_velo.Last_Err = 0;
    pid_velo.Last_Output = 0;

    // 2.2 中间环：角度环 (pid_angle)
    pid_angle.ITerm = 0;
    pid_angle.Iout = 0;
    pid_angle.Output = 0;     // 关键：清空输出，防止开机就有目标角速度
    pid_angle.Last_Err = 0;
    pid_angle.Last_Output = 0;

    // 2.3 最内环：角速度环 (pid_gyro)
    pid_gyro.ITerm = 0;
    pid_gyro.Iout = 0;
    pid_gyro.Output = 0;      // 关键：清空输出，防止开机电机就有PWM
    pid_gyro.Last_Err = 0;
    pid_gyro.Last_Output = 0;

    // 2.4 转向环 (pid_turn)
    pid_turn.ITerm = 0;
    pid_turn.Iout = 0;
    pid_turn.Output = 0;
    pid_turn.Last_Err = 0;

    // 2.5 LQR 速度环 (pid_lqr_speed)
    pid_lqr_speed.ITerm = 0;
    pid_lqr_speed.Iout = 0;
    pid_lqr_speed.Output = 0;
    pid_lqr_speed.Last_Err = 0;
    pid_lqr_speed.Last_Output = 0;

    // 2.6 LQR 转向环 (pid_lqr_turn)
    pid_lqr_turn.ITerm = 0;
    pid_lqr_turn.Iout = 0;
    pid_lqr_turn.Output = 0;
    pid_lqr_turn.Last_Err = 0;
    pid_lqr_turn.Last_Output = 0;

    // --- 3. 滤波器与执行器重置 ---
    target_speed_filter = 0.0f;
    actual_speed_filter = 0.0f;
    lqr_target_pitch_offset = 0.0f;
    lqr_turn_out = 0.0f;
    
    // 强制关闭电机
    Motor_Set_Duty(0, 0);
}
#ifdef USE_LQR_CONTROL
// -------------------------------------------------------------------------
// LQR 模式初始化：用 PIDInit 配置速度环和转向环
// -------------------------------------------------------------------------
void Motor_LQR_Init(void)
{
    Motor_Reset_State();

    PID_Init_Config_s config;

    // --- 速度环 (10ms，输出: 腿部角度偏移 度) ---
    config.Kp = LQR_SPD_KP;
    config.Ki = LQR_SPD_KI;
    config.Kd = LQR_SPD_KD;
    config.dt = CONTROL_DT * 10; // 10ms
    config.MaxOut = LQR_SPD_MAX_PITCH;
    config.IntegralLimit = 2.0f;
    config.DeadBand = 0.0f;
    config.Improve = PID_Integral_Limit | PID_OutputFilter;
    config.Output_LPF_RC = 0.5f;
    config.Derivative_LPF_RC = 0.0f;
    config.CoefA = 0.0f;
    config.CoefB = 0.0f;
    PIDInit(&pid_lqr_speed, &config);

    // --- 转向环 (1ms，输出: PWM 差分) ---
    config.Kp = LQR_TURN_KP;
    config.Ki = LQR_TURN_KI;
    config.Kd = LQR_TURN_KD;
    config.dt = CONTROL_DT; // 1ms
    config.MaxOut = LQR_MAX_TURN_SPEED;
    config.IntegralLimit = 500.0f;
    config.DeadBand = 0.0f;
    config.Improve = PID_Integral_Limit;
    config.Output_LPF_RC = 0.0f;
    config.Derivative_LPF_RC = 0.0f;
    config.CoefA = 0.0f;
    config.CoefB = 0.0f;
    PIDInit(&pid_lqr_turn, &config);
}

// -------------------------------------------------------------------------
// LQR 平衡控制核心 (含速度环 & 转向环)
// 参数:
//   leg_length:    腿长 (m)
//   imu_pitch:     俯仰角 (rad)
//   imu_gyro:      俯仰角速度 (rad/s)
//   imu_yaw_gyro:  航向角速度 (rad/s)，用于转向环
// -------------------------------------------------------------------------
void Motor_LQR_Balance_Control(float leg_length, float imu_pitch, float imu_gyro, float imu_yaw_gyro)
{
    static uint8_t speed_loop_cnt = 0;
    // 与 PID 模式对齐的信号流变量
    static float target_pitch_angle = 0.0f; // 速度环决定的目标 Pitch (轮腿解耦下锁死为 0)

    Leg = leg_length;

    // --- Step 1: 状态解算 ---
    int16_t speed_L_raw = Motor_Get_Left_Speed();
    int16_t speed_R_raw = Motor_Get_Right_Speed();
    Left_motor_duty = speed_L_raw;
    Right_motor_duty = speed_R_raw;

    // 转换为线速度 (m/s)
    float speed_L_ms = (speed_L_raw / 60.0f) * 2.0f * 3.14159f * WHEEL_RADIUS;
    float speed_R_ms = (speed_R_raw / 60.0f) * 2.0f * 3.14159f * WHEEL_RADIUS;
    float v_avg = (speed_L_ms + speed_R_ms) / 2.0f;

    robot_state.v = v_avg;
    robot_state.x += robot_state.v * CONTROL_DT;
    robot_state.theta = imu_pitch;
    robot_state.omega = imu_gyro;

    // --- Step 2: 倒地保护 ---
    if (fabsf(imu_pitch) > 0.8f || IPCS->M1_Pub.xbox_btn_a == 1) {
        Motor_Set_Duty(0, 0);
        Motor_Reset_State();
        return;
    }

    // ============================================================
    // Step 3: 速度环 (10ms 执行一次) —— 与 PID 模式完全一致
    // 功能：摇杆 -> 目标速度 -> PID -> 腿部逆解改变重心
    // ============================================================
    speed_loop_cnt++;
    if (speed_loop_cnt >= 10)
    {
        speed_loop_cnt = 0;

        // --- Step 3.1: 摇杆数据清洗 ---
        int16_t joy_v_raw = IPCS->M1_Pub.xbox_joy_l_vert;
        float speed_ratio = Map_Strange_Joystick(joy_v_raw, 1);
        y = speed_ratio;
        float target_speed = speed_ratio * MAX_TARGET_SPEED * 0.25f;

        // 目标速度滤波
        target_speed_filter = 0.9f * target_speed_filter + 0.1f * target_speed;

        // 实际速度滤波 (左右轮平均，与 PID 模式一致)
        float current_speed = (Motor_Get_Left_Speed() + Motor_Get_Right_Speed()) / 2.0f;
        actual_speed_filter = 0.7f * actual_speed_filter + 0.3f * current_speed;

        // --- Step 3.2: 速度环 PID 计算 ---
        // 输出作为腿部角度偏移量 (度)，与 PID 模式 pid_velo 逻辑完全一致
        float speed_pid_out = PIDCalculate(&pid_lqr_speed, actual_speed_filter, 0);
        outtest = speed_pid_out;

        // --- Step 3.3: 腿部逆解控制 (重心偏移) —— 与 PID 模式完全一致 ---
        // 逻辑：想前进(PID>0) -> 腿后摆(>90) -> 重心前移
        float target_leg_angle = 90.0f - speed_pid_out;

        // 物理限幅 (防止机构卡死)
        if (target_leg_angle > 150.0f) target_leg_angle = 150.0f;
        if (target_leg_angle < 30.0f)  target_leg_angle = 30.0f;

        // 执行逆解 (腿长固定 41.23mm)，驱动舵机改变重心
        float dummy_deg1, dummy_deg4;
        float wish_leg = target_leg_angle;
        Left_FiveBar_IK_Degree_Interface(41.23f, wish_leg, &dummy_deg1, &dummy_deg4);
        Right_FiveBar_IK_Degree_Interface(41.23f, wish_leg, &dummy_deg1, &dummy_deg4);

        // 轮腿解耦：目标 Pitch 锁死为 0，重心偏移已由逆解完成
        target_pitch_angle = 0.0f;
    }

    // ============================================================
    // Step 4: 增益调度 (计算 K)
    // ============================================================
    float l_safe = leg_length;
    if (l_safe < 0.03f) l_safe = 0.03f;
    if (l_safe > 0.08f) l_safe = 0.08f;

    float k_x     = poly_eval(k_x_poly, l_safe);
    float k_v     = poly_eval(k_v_poly, l_safe);
    float k_theta = poly_eval(k_theta_poly, l_safe);
    float k_omega = poly_eval(k_omega_poly, l_safe);

    // ============================================================
    // Step 5: LQR 姿态控制输出 (融合速度环重心偏移)
    // target_pitch_angle 由速度环决定 (轮腿解耦下为 0)
    // angle_error = theta - target_pitch_angle(rad)
    // ============================================================
    float u_x = k_x * robot_state.x;
    float u_v = k_v * robot_state.v;

    // 限幅
    if (u_v >  4000.0f) u_v =  4000.0f;
    else if (u_v < -4000.0f) u_v = -4000.0f;
    if (u_x >  4000.0f) u_x =  4000.0f;
    else if (u_x < -4000.0f) u_x = -4000.0f;

    // 目标角度转 rad，作为 LQR 的角度参考
    float target_pitch_rad = target_pitch_angle * 3.14159f / 180.0f;
    float angle_error = robot_state.theta - target_pitch_rad;

    float u_theta = k_theta * angle_error;
    float u_omega = k_omega * robot_state.omega;

    // 调试数组
    k_out[0] = u_x;
    k_out[1] = u_v;
    k_out[2] = u_theta;
    k_out[3] = u_omega;

    float pwm_out_float = -(u_v + u_x + u_theta + u_omega);

    // ============================================================
    // Step 6: 转向环 (1ms 执行) —— 与 PID 模式转向环结构一致
    // 摇杆水平轴 -> 目标转向速度 -> PID(Yaw角速度) -> PWM 差分
    // ============================================================
    float turn_ratio = Map_Strange_Joystick(IPCS->M1_Pub.xbox_joy_l_hori, 0);
    float target_turn_pwm = turn_ratio * LQR_MAX_TURN_SPEED;
    lqr_turn_out = PIDCalculate(&pid_lqr_turn, imu_yaw_gyro * 50.0f, target_turn_pwm);

    // ============================================================
    // Step 7: 输出混合
    // ============================================================
    int16_t pwm_base = (int16_t)pwm_out_float;
    pwm_base = clip_value(pwm_base, -9999, 9999);

    int16_t turn = (int16_t)lqr_turn_out;

    int16_t final_l = clip_value(-(pwm_base + turn), -9999, 9999);
    int16_t final_r = clip_value( (pwm_base - turn), -9999, 9999);

    Left_motor_duty  = final_l;
    Right_motor_duty = final_r;
    Motor_Set_Duty(final_l, final_r);
}
#endif // USE_LQR_CONTROL
#ifdef USE_PID_CONTROL
void Motor_PID_Init(void)
{
    Motor_Reset_State();

    PID_Init_Config_s config;

    // --- 1. 最内环：角速度环 (Gyro Loop) ---
    config.Kp = GYRO_KP;
    config.Ki = GYRO_KI;
    config.Kd = GYRO_KD;
    config.dt = CONTROL_DT; // 1ms
    config.MaxOut = MOTOR_MAX_DUTY; // PWM 满幅限制
    config.IntegralLimit = 1000.0f;
    config.DeadBand = 0;
    // 内环可以使用微分滤波来减少噪声
    config.Improve = PID_Derivative_On_Measurement | PID_Integral_Limit;
    config.Derivative_LPF_RC = 0.00;
    config.Output_LPF_RC = 0.0f;
    PIDInit(&pid_gyro, &config);

    // --- 2. 中间环：角度环 (Angle Loop) ---
    config.Kp = ANG_KP;
    config.Ki = ANG_KI;
    config.Kd = ANG_KD;
    config.dt = CONTROL_DT * 5; // 5ms
    // 重要：角度环的输出是"角速度"，物理极限通常在 10 rad/s 以内
    config.MaxOut = 15.0f;
    config.IntegralLimit = 5.0f;
    config.Improve = PID_OutputFilter|PID_Integral_Limit; // 这里的滤波可以平滑给内环的指令
    config.Output_LPF_RC = 0.0f; 
    PIDInit(&pid_angle, &config);

    // --- 3. 最外环：速度环 (Speed Loop) ---
    config.Kp = SPD_KP;
    config.Ki = SPD_KI;
    config.Kd = SPD_KD;
    config.dt = CONTROL_DT * 10; // 10ms
    config.MaxOut = SPD_MAX_PITCH; // 输出限制为最大倾角
    config.IntegralLimit = 2.0f;
    config.Improve = PID_Integral_Limit | PID_OutputFilter;
    config.Output_LPF_RC = 0.5f;
    PIDInit(&pid_velo, &config);
}
// -------------------------------------------------------------------------
// PID 平衡控制核心 (需在 pit0_ch0_isr 中调用)
// 参数: 
//   imu_pitch: 当前俯仰角(角度度)
//   imu_gyro_rad:  当前俯仰角速度(rad/s) -> 对应 Gyro X
//   imu_yaw_gyro_rad: 当前航向角速度(rad/s) -> 对应 Gyro Z
// -------------------------------------------------------------------------
void Motor_PID_Balance_Control(float imu_pitch, float imu_gyro_rad, float imu_yaw_gyro_rad)
{
    static uint8_t speed_loop_cnt = 0;
    static uint8_t angle_loop_cnt = 0;
    // 串级信号流变量
    static float target_pitch_angle = 0.0f; // 来自速度环
    static float target_gyro_rate = 0.0f;   // 来自角度环
    float balance_pwm_out = 0.0f;    // 来自角速度环

    // 0. 倒地保护
    // if (fabsf(imu_pitch) > 60.0f || IPCS->M1_Pub.xbox_btn_a == 1)
    // {
    //     Motor_Reset_State();
    //     return;
    // }

    // ============================================================
    // 1. 最外环：速度环 (10ms 执行一次)
    // 功能：根据摇杆控制腿部角度，改变重心
    // ============================================================
    speed_loop_cnt++;
    if (speed_loop_cnt >= 10) 
    {
        speed_loop_cnt = 0;

        // --- Step 1.1: 摇杆数据清洗 ---
        // 获取竖直摇杆 (控制前后)
        int16_t joy_v_raw = IPCS->M1_Pub.xbox_joy_l_vert;
        // 使用专用函数映射到 -1.0 ~ 1.0
        float speed_ratio = Map_Strange_Joystick(joy_v_raw, 1); 
        y = speed_ratio;
        // 计算目标速度 (单位: 编码器数值/PWM量级)
        float target_speed = speed_ratio * MAX_TARGET_SPEED * 0.25;
        
        // 目标速度滤波 (使加减速平滑)
        target_speed_filter = 0.9f * target_speed_filter + 0.1f * target_speed;

        // 获取实际速度 (左右轮平均)
        float current_speed = (Motor_Get_Left_Speed() + Motor_Get_Right_Speed()) / 2.0f;
        actual_speed_filter = 0.7f * actual_speed_filter + 0.3f * current_speed;

        // --- Step 1.2: 速度环 PID 计算 ---
        // 这里的 PID 输出将作为“腿部角度的偏移量”
        float speed_pid_out = PIDCalculate(&pid_velo, actual_speed_filter, 0);
        outtest = speed_pid_out;
        // --- Step 1.3: 腿部逆解控制 (重心偏移) ---
        // 逻辑：想前进(PID>0) -> 腿后摆(Pi0<90) -> 摆杆前倾
        // 基础角度 90度 - PID输出
        float target_leg_angle = 90.0f-speed_pid_out;

        // 物理限幅 (防止机构卡死)
        if(target_leg_angle > 150.0f) target_leg_angle = 150.0f;
        if(target_leg_angle < 30.0f)  target_leg_angle = 30.0f;

        // 执行逆解 (假设腿长固定 41.23)
        float dummy_deg1, dummy_deg4;
        float wish_leg = target_leg_angle;
        Left_FiveBar_IK_Degree_Interface(41.23f, wish_leg, &dummy_deg1, &dummy_deg4);
        Right_FiveBar_IK_Degree_Interface(41.23f, wish_leg, &dummy_deg1, &dummy_deg4);
        
        // 注意：在轮腿解耦模式下，目标 Pitch 角度始终锁死为 0 (或者机械中值偏移)
        target_pitch_angle = 0.0f; 
    }
    // // ============================================================
    // 2. 中间环：角度环 (5ms 执行)
    // 目标：目标角度 -> 输出：目标角速度 (Target Gyro Rate)
    // ============================================================
    angle_loop_cnt++;
    if (angle_loop_cnt >= 5) 
    {
        angle_loop_cnt = 0;
    // 这里使用标准的 PIDCalculate
    // 输入：实际角度，期望：目标角度 (机械中值（中断中） + 速度环输出)
    float final_target_pitch = target_pitch_angle;
    
    // 角度环计算
    // 物理含义：如果我还没到目标角度，我希望以多快的速度转过去？
    target_gyro_rate = PIDCalculate(&pid_angle, imu_pitch, final_target_pitch);
    }
    // ============================================================
    // 3. 最内环：角速度环 (1ms 执行)
    // 目标：目标角速度 -> 输出：电机 PWM
    // ============================================================
    
    // 输入：实际角速度 (imu_gyro_rad)，期望：目标角速度
    balance_pwm_out = PIDCalculate(&pid_gyro, imu_gyro_rad, target_gyro_rate);

    // ============================================================
    // 4. 转向环 & 输出混合
    // ============================================================
    
    // --- Step 4.1: 摇杆数据清洗 ---
    // 获取水平摇杆 (控制左右)
    // int16_t joy_h_raw = IPCS->M1_Pub.xbox_joy_l_hori;
    // // 使用专用函数映射到 -1.0 ~ 1.0 (正为左，负为右)
    // float turn_ratio = Map_Strange_Joystick(joy_h_raw, 0);

    // // 计算目标转向速度 (PWM 差分量级)
    // // 注意：通常左推摇杆希望向左转，左轮减速右轮加速
    // float target_turn_pwm = turn_ratio * MAX_TURN_SPEED;

    // --- Step 4.2: 转向 PID (简单 P 或 PD) ---
    // 输入：Yaw轴角速度 (rad/s) * 系数， 期望：摇杆设定值
    // 目的：让车身的旋转速度跟随摇杆的深浅
    // float turn_out = PIDCalculate(&pid_turn, imu_yaw_gyro_rad * 50.0f, target_turn_pwm);

    float motor_l = balance_pwm_out; //+ turn_out;
    float motor_r = balance_pwm_out; //- turn_out;
    // outtest = balance_pwm_out;
    int16_t final_l = -(int16_t)motor_l; // 极性根据你原来的代码保留
    int16_t final_r = (int16_t)motor_r;
    Left_motor_duty = final_l;
    Right_motor_duty = final_r;
    Motor_Set_Duty(final_l, final_r);
}
#endif
