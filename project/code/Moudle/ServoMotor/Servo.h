#ifndef _SERVO_H_
#define _SERVO_H_

#include "zf_common_headfile.h"

// -------------------------------------------------------------------------
// 硬件引脚配置区 (根据实际接线修改此处)
// -------------------------------------------------------------------------
#define SERVO_CH1_PIN       (TCPWM_CH13_P00_3)//左下舵机  对应单脚左角度
#define SERVO_CH4_PIN       (TCPWM_CH04_P04_0)//左上舵机  对应单脚右角度

//电机驱动黑色线部分
#define SERVO_CH2_PIN       (TCPWM_CH12_P01_0)//右上舵机  对应单脚左角度
#define SERVO_CH3_PIN       (TCPWM_CH11_P01_1)//右下舵机  对应单脚右角度
// -------------------------------------------------------------------------
// 舵机参数配置
// -------------------------------------------------------------------------
#define SERVO_FREQ          (300)    // 舵机频率 300HZ,50Hz 

// 舵机机械限位 (度)
#define SERVO_ANGLE_MIN     (0)
#define SERVO_ANGLE_MAX     (180)

// 舵机步进速度配置
// 每次调用 Servo_Task 时移动的角度
#define SERVO_DEFAULT_STEP  (0.8f) 

// -------------------------------------------------------------------------
// 类型定义
// -------------------------------------------------------------------------
// 舵机通道索引（用于数组管理，方便循环）
typedef enum {
    SERVO_ID_1_LD = 0, // 左下 (Left Down)
    SERVO_ID_2_RU,     // 右上 (Right Up)
    SERVO_ID_3_RD,     // 右下 (Right Down)
    SERVO_ID_4_LU,     // 左上 (Left Up)
    SERVO_NUM_MAX      // 舵机总数
} Servo_ID_Enum;
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
void Servo_SetAngle_Direct(pwm_channel_enum pwm_ch, float angle);

// id: 舵机索引 (SERVO_ID_1_LD 等)
// angle: 目标角度
void Servo_SetTarget(Servo_ID_Enum id, float angle);
#endif // _SERVO_H_
