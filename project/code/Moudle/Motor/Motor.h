#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stdint.h"
#include "small_driver_uart_control.h" 

//-------------------------------------------------------------------------------------------------------------------
//  宏定义 & 物理参数
//-------------------------------------------------------------------------------------------------------------------
#define MOTOR_MAX_DUTY      10000   // 电机最大占空比限制
#define MOTOR_MIN_DUTY      -10000  // 电机反转最大占空比限制

#define CONTROL_DT          0.005f  // 控制周期 (秒)，需与定时器中断频率一致
// [修改点]：根据最新测量，半径修正为 34mm
#define WHEEL_RADIUS        0.034f  // 车轮半径 (米)

//-------------------------------------------------------------------------------------------------------------------
//  函数声明 (保持不变)
//-------------------------------------------------------------------------------------------------------------------

void Motor_Init(void);
void Motor_Set_Duty(int16_t left_duty, int16_t right_duty);
int16_t Motor_Get_Left_Speed(void);
int16_t Motor_Get_Right_Speed(void);

// LQR 平衡控制核心函数
void Motor_LQR_Balance_Control(float leg_length, float imu_pitch, float imu_gyro);
void Motor_Reset_State(void);

#endif // _MOTOR_H_