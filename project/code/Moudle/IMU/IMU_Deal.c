#include "IMU_Deal.h"
#include <math.h>

imu_fusion_t imu_sys;

// 内部使用的动态参数
static float safe_Kp = KP_NORMAL;
static float safe_Ki = KI_NORMAL;

// 快速平方根倒数 (经典算法，用于归一化)
static float InvSqrt(float x) {
    if (x <= 0) return 0;
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//==============================================================================
// 1. 初始化 (融合了你的快速收敛思路)
//==============================================================================
void IMU_Fusion_Init(void)
{
    // 1. 初始化四元数
    imu_sys.q0 = 1.0f; imu_sys.q1 = 0.0f; imu_sys.q2 = 0.0f; imu_sys.q3 = 0.0f;
    imu_sys.exInt = 0.0f; imu_sys.eyInt = 0.0f; imu_sys.ezInt = 0.0f;
    imu_sys.is_ready = 0;

    // 2. [关键步骤] 快速收敛
    // 在开机的前 200 次循环中，使用极大的 Kp，让姿态瞬间对齐重力方向
    // 这样你就不用傻等几秒钟让数据稳定了
    safe_Kp = KP_INIT; 
    safe_Ki = 0.0f; // 初始化时不积分，防止把初始误差积进去

    for(int i = 0; i < 200; i++)
    {
        // 这里假设底层驱动有阻塞延时，或者跑得足够快
        // 必须在这里读数据，否则 Update 算的是旧数据
        imu660ra_get_acc(); 
        imu660ra_get_gyro(); 
        IMU_Fusion_Update();
        // system_delay_ms(1); // 如果需要的话
    }

    // 3. 恢复正常参数
    safe_Kp = KP_NORMAL;
    safe_Ki = KI_NORMAL;
    imu_sys.is_ready = 1;
}

//==============================================================================
// 2. 核心更新函数 (Mahony + 抗震 + Smart Yaw) - 修正版
//==============================================================================
void IMU_Fusion_Update(void)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float halfT = FUSION_DT * 0.5f;

    // --- 1. 获取硬件数据 & 预处理 ---
    // [修正]: 陀螺仪单位转换 Raw -> rad/s
    // 驱动设置的是 2000dps，对应灵敏度 16.4 LSB/(deg/s)
    float gx = ((float)imu660ra_gyro_x) / 16.4f * (3.14159f / 180.0f);
    float gy = ((float)imu660ra_gyro_y) / 16.4f * (3.14159f / 180.0f);
    float gz = ((float)imu660ra_gyro_z) / 16.4f * (3.14159f / 180.0f);

    // 获取原始加速度 (LSB)
    float ax = (float)imu660ra_acc_x;
    float ay = (float)imu660ra_acc_y;
    float az = (float)imu660ra_acc_z;

    // --- 2. Smart Yaw (Z轴静态锁死) ---
    if (fabsf(gz) < YAW_DEADZONE) {
        imu_sys.yaw_stable_cnt++;
        if (imu_sys.yaw_stable_cnt >= YAW_LOCK_COUNT) {
            gz = 0.0f; 
            if(imu_sys.yaw_stable_cnt > 1000) imu_sys.yaw_stable_cnt = 1000;
        }
    } else {
        imu_sys.yaw_stable_cnt = 0;
    }
    
    // 输出滤波后的陀螺仪数据给外部 (LQR用)
    imu_sys.gyro_x_filt = gx; 
    imu_sys.gyro_y_filt = gy; 
    imu_sys.gyro_z_filt = gz;

    // --- 3. Mahony 互补滤波核心 ---
    
    // [致命错误修正点]: 将原始加速度转换为 g 单位
    // 驱动设置的是 ±8g 模式，灵敏度为 4096 LSB/g
    float acc_norm_raw = sqrtf(ax*ax + ay*ay + az*az); // 算出的是 LSB 模长 (静止时约4096)
    float acc_norm_g = acc_norm_raw / 4096.0f;         // 转换为 g 单位 (静止时约1.0)

    // [抗干扰核心]: 只有受力在 0.85g ~ 1.15g 之间，才认为加速度计可信
    if(acc_norm_g > ACC_MIN_G && acc_norm_g < ACC_MAX_G)
    {
        // 归一化加速度向量 (用原始数据归一化即可，结果是一样的向量方向)
        norm = InvSqrt(ax*ax + ay*ay + az*az);
        if(norm == 0.0f) return; // 防止除以0
        ax *= norm;
        ay *= norm;
        az *= norm;

        // 估计重力方向 (四元数转旋转矩阵第三列)
        vx = 2.0f * (imu_sys.q1 * imu_sys.q3 - imu_sys.q0 * imu_sys.q2);
        vy = 2.0f * (imu_sys.q0 * imu_sys.q1 + imu_sys.q2 * imu_sys.q3);
        vz = imu_sys.q0 * imu_sys.q0 - imu_sys.q1 * imu_sys.q1 - imu_sys.q2 * imu_sys.q2 + imu_sys.q3 * imu_sys.q3;

        // 向量叉积计算误差 (测量值 x 估计值)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // 积分误差 (消除温漂)
        imu_sys.exInt += ex * safe_Ki;
        imu_sys.eyInt += ey * safe_Ki;
        imu_sys.ezInt += ez * safe_Ki;

        // PI 修正陀螺仪
        gx += safe_Kp * ex + imu_sys.exInt;
        gy += safe_Kp * ey + imu_sys.eyInt;
        gz += safe_Kp * ez + imu_sys.ezInt;
    }
    else
    {
        // 震动状态！放弃加速度计修正，只信陀螺仪，防止姿态乱跳
        // 此时 gx, gy, gz 保持原值 (纯积分)
        // 调试时可以亮个灯提示震动过大
    }

    // --- 4. 四元数积分 (一阶龙格库塔) ---
    float q0 = imu_sys.q0;
    float q1 = imu_sys.q1;
    float q2 = imu_sys.q2;
    float q3 = imu_sys.q3;

    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    // --- 5. 四元数归一化 ---
    norm = InvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    imu_sys.q0 = q0 * norm;
    imu_sys.q1 = q1 * norm;
    imu_sys.q2 = q2 * norm;
    imu_sys.q3 = q3 * norm;

    // --- 6. 欧拉角转换 (Z-Y-X 顺序) ---
    // Pitch (X轴)
    imu_sys.pitch = asinf(-2.0f * (imu_sys.q1 * imu_sys.q3 - imu_sys.q0 * imu_sys.q2)) * 57.29578f;
    // Roll (Y轴)
    imu_sys.roll  = atan2f(2.0f * (imu_sys.q0 * imu_sys.q1 + imu_sys.q2 * imu_sys.q3), 1.0f - 2.0f * (imu_sys.q1 * imu_sys.q1 + imu_sys.q2 * imu_sys.q2)) * 57.29578f;
    // Yaw (Z轴)
    imu_sys.yaw   = atan2f(2.0f * (imu_sys.q1 * imu_sys.q2 + imu_sys.q0 * imu_sys.q3), 1.0f - 2.0f * (imu_sys.q2 * imu_sys.q2 + imu_sys.q3 * imu_sys.q3)) * 57.29578f;
}