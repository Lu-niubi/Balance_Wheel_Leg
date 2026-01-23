#ifndef _IMU_DEAL_H_
#define _IMU_DEAL_H_

#include "zf_common_headfile.h"

//==============================================================================
// 核心参数调优 (根据 BDS3620 和你的车重优化)
//==============================================================================
#define FUSION_DT           0.005f          // 运行周期 5ms

// --- Mahony 算法参数 ---
#define KP_NORMAL           0.8f            // 正常运行时的比例增益 (动态修正速度)
#define KI_NORMAL           0.002f          // 正常运行时的积分增益 (消除温漂)
#define KP_INIT             20.0f           // [关键] 初始化时的超大增益，用于秒收敛

// --- 抗干扰阈值 ---
#define ACC_MIN_G           0.85f           // 最小合力 (小于此值认为失重/下坡)
#define ACC_MAX_G           1.15f           // 最大合力 (大于此值认为震动/撞击)

// --- Smart Yaw (Z轴防漂移) ---
#define YAW_DEADZONE        0.005f          // rad/s，陀螺仪Z轴死区
#define YAW_LOCK_COUNT      50              // 连续多少次静止才锁死

typedef struct
{
    // --- 输出数据 (给 LQR 用) ---
    float pitch;        // 度 (Deg)
    float roll;         // 度 (Deg)
    float yaw;          // 度 (Deg)
    
    // --- 调试数据 ---
    float gyro_x_filt;  // rad/s (去零偏、滤波后的角速度)
    float gyro_y_filt;
    float gyro_z_filt;

    // --- 内部状态 ---
    float q0, q1, q2, q3;       // 四元数
    float exInt, eyInt, ezInt;  // 积分误差累加
    uint16_t yaw_stable_cnt;    // Yaw轴稳定计数器
    uint8_t is_ready;           // 初始化完成标志

} imu_fusion_t;

extern imu_fusion_t imu_sys;

// 接口函数
void IMU_Fusion_Init(void);
void IMU_Fusion_Update(void);

#endif