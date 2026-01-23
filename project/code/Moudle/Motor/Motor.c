#include "Motor.h"
#include <string.h> 
#include <math.h> // 需要 fabs, fmax, fmin

//-------------------------------------------------------------------------------------------------------------------
//  LQR 拟合参数 (BDS3620, 85g轮组, R=34mm)
//-------------------------------------------------------------------------------------------------------------------
// [修改点]：使用 MATLAB 生成的最新参数
// 注意：MATLAB 输出显示 k_x 为负值，k_theta 为正值，这由 LQR 求解器决定，保持原样即可。
float k_x_poly[4] = {0.000002, -0.000000, 0.000000, -4545.454545};
float k_v_poly[4] = {11223146.756673, -2518565.657087, 204284.812516, -9309.403979};
float k_theta_poly[4] = {2085657.222577, -688928.563716, 115088.793536, 14379.960975};
float k_omega_poly[4] = {-537135.911097, 133183.763695, -7332.475099, 1555.495022};

//-------------------------------------------------------------------------------------------------------------------
//  内部变量
//-------------------------------------------------------------------------------------------------------------------
typedef struct {
    float x;        // 位移 (m)
    float v;        // 速度 (m/s)
    float theta;    // 角度 (rad)
    float omega;    // 角速度 (rad/s)
} Robot_State_t;

static Robot_State_t robot_state = {0}; 

//-------------------------------------------------------------------------------------------------------------------
//  内部静态函数
//-------------------------------------------------------------------------------------------------------------------

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
    robot_state.x = 0.0f;
    robot_state.v = 0.0f;
    robot_state.theta = 0.0f;
    robot_state.omega = 0.0f;
    Motor_Set_Duty(0, 0);
}

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