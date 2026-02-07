#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stdint.h"
#include "small_driver_uart_control.h" 

//-------------------------------------------------------------------------------------------------------------------
// 控制类型，注意只能选择一个，不选的那个要注释掉
//-------------------------------------------------------------------------------------------------------------------
#define USE_LQR_CONTROL    // 使用 LQR 平衡控制
// #define USE_PID_CONTROL // 使用 PID 平衡控制

//-------------------------------------------------------------------------------------------------------------------
//  宏定义 & 物理参数
//-------------------------------------------------------------------------------------------------------------------
#define MOTOR_MAX_DUTY      10000   // 电机最大占空比限制
#define MOTOR_MIN_DUTY      -10000  // 电机反转最大占空比限制

#define CONTROL_DT          0.005f  // 控制周期 (秒)，需与定时器中断频率一致
// [修改点]：根据最新测量，半径修正为 34mm
#define WHEEL_RADIUS        0.034f  // 车轮半径 (米)

//-------------------------------------------------------------------------------------------------------------------
// 模式定义冲突检查 (编译时检测错误)
//-------------------------------------------------------------------------------------------------------------------

// 1. 检查是否同时定义了多种模式 (互斥检查)
#if defined(USE_LQR_CONTROL) && defined(USE_PID_CONTROL)
    #error Conflict control mode! You can only define ONE control mode (LQR or PID).
#endif

// 2. 检查是否未定义任何模式 (缺失检查)
#if !defined(USE_LQR_CONTROL) && !defined(USE_PID_CONTROL)
    #error Missing control mode! You must define at least one control mode.
#endif
//-------------------------------------------------------------------------------------------------------------------
//  函数声明 (保持不变)
//-------------------------------------------------------------------------------------------------------------------

void Motor_Init(void);
void Motor_Set_Duty(int16_t left_duty, int16_t right_duty);
int16_t Motor_Get_Left_Speed(void);
int16_t Motor_Get_Right_Speed(void);
void Motor_Reset_State(void);

#ifdef USE_LQR_CONTROL
// LQR 平衡控制核心函数
void Motor_LQR_Balance_Control(float leg_length, float imu_pitch, float imu_gyro);
#endif

#ifdef USE_PID_CONTROL
void Motor_PID_Init(void); // PID 参数初始化
// PID 平衡控制核心函数
void Motor_PID_Balance_Control(float imu_pitch_rad, float imu_gyro_rad, float imu_yaw_gyro_rad);
#endif

#endif // _MOTOR_H_
