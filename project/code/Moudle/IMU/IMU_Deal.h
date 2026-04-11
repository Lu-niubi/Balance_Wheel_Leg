#ifndef _IMU_DEAL_H_
#define _IMU_DEAL_H_

#include "zf_common_headfile.h"
#include "KSCal.h"
#include "zf_common_typedef.h"

//==============================================================================
// 核心参数
//==============================================================================
#define FUSION_DT           0.001f          // 运行周期 1ms (1000Hz)
#define GYRO_LSB_2000DPS    16.384f           // 2000dps 灵敏度
#define ACC_LSB_8G          4096.0f         // 8g 灵敏度

// --- Madgwick 算法参数 ---
// beta 越大收敛越快但动态噪声越大，越小静态越稳但收敛慢
#define BETA_NORMAL         0.1f            // 正常运行 beta：0.1偏大但保证动后能回原点
#define BETA_INIT           5.0f            // 初始化快速收敛 beta

// --- Smart Yaw (Z轴防漂移) ---
#define YAW_DEADZONE        0.010f          // rad/s，只过滤真正静止噪声

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
    uint8 is_ready;

} imu_fusion_t;

extern imu_fusion_t imu_sys;

// 接口函数
void IMU_Fusion_Init(void);
void IMU_Fusion_Update(void);
void IMU_Gyro_Calibration(void);

#endif