/**
 * @file     controller.h
 * @author   Wang Hongxi (Ported for CYT4BB Fixed DT)
 * @version  V1.1.4
 * @date     2024/1/4
 * @brief    PID控制器头文件 (适配固定周期调用)
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "zf_common_headfile.h" // 包含逐飞库头文件以获取基础类型定义
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

// PID 优化环节使能标志位
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000 基础PID模式
    PID_Integral_Limit = 0b00000001,              // 0000 0001 积分限幅
    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010 微分先行
    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100 梯形积分
    PID_Proportional_On_Measurement = 0b00001000, // 0000 1000 比例先行 (未实现)
    PID_OutputFilter = 0b00010000,                // 0001 0000 输出滤波
    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000 变速积分
    PID_DerivativeFilter = 0b01000000,            // 0100 0000 微分滤波
    PID_ErrorHandle = 0b10000000,                 // 1000 0000 错误处理(堵转检测)
} PID_Improvement_e;

/* PID 报错类型枚举 */
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID结构体 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分参数A
    float CoefB;             // 变速积分参数B
    float Output_LPF_RC;     // 输出滤波器系数
    float Derivative_LPF_RC; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    float dt; // 固定的控制周期 (单位:秒)

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于PID初始化的结构体 */
typedef struct
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区
    float dt;       // 固定的控制周期 (单位:秒)，例如 1ms 填 0.001f

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分参数
    float CoefB;             // 变速积分参数
    float Output_LPF_RC;     // 输出滤波系数
    float Derivative_LPF_RC; // 微分滤波系数
} PID_Init_Config_s;

/**
 * @brief 初始化PID实例
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief 计算PID输出 (固定周期调用)
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);
float PIDCalculate_Balance(PIDInstance *pid, float measure, float ref, float gyro);

#endif // _CONTROLLER_H