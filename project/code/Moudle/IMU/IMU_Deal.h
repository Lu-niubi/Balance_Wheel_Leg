#ifndef _IMU_DEAL_H_
#define _IMU_DEAL_H_

#include "zf_common_headfile.h"
#include "KSCal.h"
//==============================================================================
// 核心参数
//==============================================================================
#define FUSION_DT           0.001f          // 运行周期 1ms (1000Hz)
#define GYRO_LSB_2000DPS    16.4f           // 2000dps 灵敏度
#define ACC_LSB_8G          4096.0f         // 8g 灵敏度

// --- Mahony 算法参数 ---
#define KP_NORMAL           0.8f            // 正常比例增益0.8
#define KI_NORMAL           0.002f          // 正常积分增益0.2
#define KP_INIT             20.0f           // 初始化快速收敛增益

// --- 抗干扰阈值 ---
#define ACC_MIN_G           0.85f           
#define ACC_MAX_G           1.15f           

// --- Smart Yaw (Z轴防漂移) ---
#define YAW_DEADZONE        0.05f           // rad/s，略微调大死区

typedef struct
{
    // 输出数据
    float pitch;
    float roll;
    float yaw;
    
    // 滤波/去偏后的角速度 (rad/s)
    float gx, gy, gz;

    // 零偏数据
    float offset_gx;
    float offset_gy;
    float offset_gz;

    // 内部状态
    float q0, q1, q2, q3;
    float exInt, eyInt, ezInt;
    float yaw_offset;
    uint8_t is_ready;

} imu_fusion_t;

extern imu_fusion_t imu_sys;

// 接口函数
void IMU_Fusion_Init(void);
void IMU_Fusion_Update(void);
void IMU_Gyro_Calibration(void);

#endif