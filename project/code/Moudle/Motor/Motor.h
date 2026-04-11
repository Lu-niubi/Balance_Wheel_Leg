#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stdint.h"
#include "small_driver_uart_control.h"
#include "controller.h"
#include "zf_common_typedef.h"


//中等高度下

//-------------------------------------------------------------------------------------------------------------------
// 控制类型，注意只能选择一个，不选的那个要注释掉
//-------------------------------------------------------------------------------------------------------------------
//#define USE_LQR_CONTROL    // 使用 LQR 平衡控制
#define USE_PID_CONTROL // 使用 PID 平衡控制

//-------------------------------------------------------------------------------------------------------------------
//  宏定义 & 物理参数
//-------------------------------------------------------------------------------------------------------------------
#define MOTOR_MAX_DUTY      9999   // 电机最大占空比限制10000
#define MOTOR_MIN_DUTY      -9999  // 电机反转最大占空比限制-10000

#define CONTROL_DT          0.001f  // 控制周期 (秒)，需与定时器中断频率一致
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

extern int16_t Left_motor_duty;
extern int16_t Right_motor_duty;
extern float outtest;
extern float y,x;
extern float Leg;
extern float k_out[4];

// 动态调参全局变量（可通过串口修改）
extern float GYRO_KP;
extern float ANG_KP;
extern float ANG_KI;
extern float ANG_KD;
extern float GYRO_KI;
extern float GYRO_KD;
void Motor_Init(void);
void Motor_Set_Duty(int16_t left_duty, int16_t right_duty);
//获取到的最后速度rpm已经归一化前进为正值
int16_t Motor_Get_Left_Speed(void);
//获取到的最后速度rpm已经归一化前进为正值
int16_t Motor_Get_Right_Speed(void);
void Motor_Reset_State(void);

// 供 TuneCmd 动态调参使用
PIDInstance* Motor_Get_Angle_PID(void);
PIDInstance* Motor_Get_Gyro_PID(void);

// 外部目标控制接口 (惯导/科目任务使用)
void Motor_Set_Ext_Control(uint8 enable);   // 1=启用外部目标, 0=恢复原有逻辑
void Motor_Set_Ext_Speed(float speed_rpm);    // 设置目标速度 (RPM)
void Motor_Set_Ext_Yaw(float yaw_deg);        // 设置目标航向 (度，连续累积值)
float Motor_Get_Yaw_Continuous(void);         // 获取连续累积航向角 (无跳变)
float Motor_Get_Ext_Yaw(void);               // 获取当前外部目标航向 (连续度)

// 旋转模式接口 (科目2定点旋转使用)
void Motor_Set_Spin(int16_t spin_pwm);       // 非0=旋转模式(差速), 0=恢复正常PID转向

// ─── 单边桥模式 (科目3颠簸路段) ───
// Roll PID 参数
#define ROLL_MECH_ZERO      0.0f    // 机械水平中值 (需实际标定)
#define ROLL_KP_INIT        2.0f
#define ROLL_KI_INIT        0.0f
#define ROLL_KD_INIT        0.05f
#define ROLL_MAX_LEG_DELTA  30.0f    // 腿长最大补偿量 (mm)
// 腿长范围
#define SB_BASE_L0          41.23f  // 单边桥基础腿长 (mm)
#define SB_LEG_MIN          25.0f
#define SB_LEG_MAX          65.0f
// 速度限制
#define SB_SPEED_LIMIT      100.0f  // 单边桥速度限制 (RPM)

extern uint8 g_single_bridge_mode;         // 单边桥模式标志 (视觉模块设置)
void Motor_Set_Single_Bridge_Mode(uint8 enable); // 启用/关闭单边桥模式
void Motor_Roll_Loop(float imu_roll);              // Roll 补偿环 (10ms 调用)

#ifdef USE_LQR_CONTROL
void Motor_LQR_Init(void); // LQR 参数初始化
// LQR 平衡控制核心函数 (含速度环 & 转向环)
void Motor_LQR_Balance_Control(float leg_length, float imu_pitch, float imu_gyro, float imu_yaw_gyro);
#endif

#ifdef USE_PID_CONTROL
void Motor_PID_Init(void); // PID 参数初始化
#endif

#ifdef USE_PID_CONTROL
void Motor_PID_Balance_Control(float imu_pitch_rad, float imu_gyro_rad, float imu_yaw_gyro_rad);// PID 平衡控制核心函数
#endif

#endif // _MOTOR_H_
