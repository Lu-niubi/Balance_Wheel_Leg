#ifndef _IMU_DEAL_H_
#define _IMU_DEAN_H_

#include "zf_common_headfile.h" // 包含你的BSP库

//==============================================================================
// 1. 参数配置 (根据你的车模调整)
//==============================================================================
#ifndef PI
    #define PI 3.1415926535f
#endif

#define FUSION_DT           0.005f          // 运行周期 5ms (200Hz)

// --- 防漂移与抗干扰 ---
#define GYRO_OFFSET_COUNT   100             // 启动时校准采样的次数
#define ACC_LPF_ALPHA       0.3f            // 加速度计低通滤波 (越小越平滑)
#define YAW_DEADZONE        0.003f          // [关键] Yaw轴死区 (rad/s)，小于此值强行置0
#define YAW_LOCK_COUNT      20              // 连续多少次在死区内才锁定

// --- EKF 矩阵参数 (Q:过程噪声, R:观测噪声) ---
// Q越小，相信预测(陀螺仪)越多；R越小，相信观测(加速度计)越多
#define Q_PROCESS           0.002f          // 过程噪声 (陀螺仪信赖度)
#define R_MEASURE           1.0f            // 观测噪声 (加速度计信赖度)
#define MAX_ACC_ERR         0.2f            // 残差阈值 (超过此值认为加速度计不可信/震动过大)

//==============================================================================
// 2. 数据结构
//==============================================================================
typedef struct
{
    // 原始数据 (处理后)
    float acc_x;        // g
    float acc_y;
    float acc_z;
    float gyro_x;       // rad/s
    float gyro_y;
    float gyro_z;

    // 零偏
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;

    // 欧拉角输出 (度)
    float pitch;
    float roll;
    float yaw;

    // 四元数状态 (q0, q1, q2, q3)
    float q[4];

    // 内部状态
    uint8_t yaw_stable_count; // 稳定性计数
    uint8_t is_ready;         // 初始化完成标志

} imu_fusion_t;

extern imu_fusion_t imu_sys;

//==============================================================================
// 3. 函数接口
//==============================================================================
void IMU_Fusion_Init(void);     // 初始化+校准
void IMU_Fusion_Update(void);   // 主循环调用 (建议5ms一次)

#endif