#ifndef _SERVO_H_
#define _SERVO_H_

#include "zf_common_headfile.h"

// -------------------------------------------------------------------------
// 硬件引脚配置区 (根据实际接线修改此处)
// -------------------------------------------------------------------------
#define SERVO_CH1_PIN       (TCPWM_CH13_P00_3)  
#define SERVO_CH2_PIN       (TCPWM_CH12_P01_0)  
#define SERVO_CH3_PIN       (TCPWM_CH11_P01_1)  
#define SERVO_CH4_PIN       (TCPWM_CH17_P00_1)  
// -------------------------------------------------------------------------
// 舵机参数配置
// -------------------------------------------------------------------------
#define SERVO_FREQ          (50)    // 舵机频率 50Hz 

// 舵机机械限位 (度)
#define SERVO_ANGLE_MIN     (0)
#define SERVO_ANGLE_MAX     (180)

// -------------------------------------------------------------------------
// 函数声明
// -------------------------------------------------------------------------

/**
 * @brief 初始化舵机模块
 * 初始化配置文件中定义的所有舵机引脚
 */
void Servo_Init(void);

/**
 * @brief 设置舵机角度
 * * @param pwm_ch  舵机对应的PWM引脚枚举 (例如 SERVO_CH1_PIN)
 * @param angle   目标角度 (浮点数，建议范围 0.0 - 180.0)
 */
void Servo_SetAngle(pwm_channel_enum pwm_ch, float angle);

#endif // _SERVO_H_
