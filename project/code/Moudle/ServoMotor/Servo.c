#include "Servo.h"

// -------------------------------------------------------------------------
// 内部宏定义：占空比计算公式
// -------------------------------------------------------------------------
// 原理说明：
// 50Hz 周期 = 20ms
// 0度   = 0.5ms 高电平
// 180度 = 2.5ms 高电平
// 公式推导：
// 高电平时间(ms) = 0.5 + (angle / 90.0)
// 占空比 = (高电平时间 / 周期时间) * PWM_DUTY_MAX
// -------------------------------------------------------------------------
#define CALC_SERVO_DUTY(angle)  ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + ((float)(angle) / 90.0)))

/**
 * @brief 初始化舵机模块
 */
void Servo_Init(void)
{
    // 检查频率安全性
#if (SERVO_FREQ < 50 || SERVO_FREQ > 300)
    #error "SERVO_FREQ Error! Must be between 50-300Hz"
#endif

    // 初始化各个舵机通道，初始占空比设为0 (或者设为中值)
    // 300 是 PWM 分辨率参数，根据逐飞库的底层实现保留
    pwm_init(SERVO_CH1_PIN, SERVO_FREQ, 0);
    pwm_init(SERVO_CH2_PIN, SERVO_FREQ, 0);
    pwm_init(SERVO_CH3_PIN, SERVO_FREQ, 0);
    pwm_init(SERVO_CH4_PIN, SERVO_FREQ, 0);
}

/**
 * @brief 设置舵机角度
 * @param pwm_ch 舵机PWM通道
 * @param angle  角度 0-180
 */
void Servo_SetAngle(pwm_channel_enum pwm_ch, float angle)
{
    // 1. 软件限位保护 (防止机械损坏)
    if(angle < SERVO_ANGLE_MIN)
    {
        angle = SERVO_ANGLE_MIN;
    }
    else if(angle > SERVO_ANGLE_MAX)
    {
        angle = SERVO_ANGLE_MAX;
    }

    // 2. 计算目标占空比
    uint32 duty = (uint32)CALC_SERVO_DUTY(angle);

    // 3. 设置PWM
    pwm_set_duty(pwm_ch, duty);
}