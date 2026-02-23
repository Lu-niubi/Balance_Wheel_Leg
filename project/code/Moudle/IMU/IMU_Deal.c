#include "IMU_Deal.h"
#include <math.h>

imu_fusion_t imu_sys;

static float safe_Kp = KP_NORMAL;
static float safe_Ki = KI_NORMAL;

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
// 1. 陀螺仪自动校准 (静止校准)
//==============================================================================
void IMU_Gyro_Calibration(void)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const uint16_t samples = 500;

    for(uint16_t i = 0; i < samples; i++)
    {
        imu660ra_get_gyro();
        sum_gx += (float)imu660ra_gyro_x;
        sum_gy += (float)imu660ra_gyro_y;
        sum_gz += (float)imu660ra_gyro_z;
        system_delay_ms(2); // 采样间隔需要与实际运行接近
    }
    
    // 存储原始 LSB 级别的偏置，或者直接存储 rad/s 级别的偏置
    imu_sys.offset_gx = (sum_gx / samples) / GYRO_LSB_2000DPS * (M_PI / 180.0f);
    imu_sys.offset_gy = (sum_gy / samples) / GYRO_LSB_2000DPS * (M_PI / 180.0f);
    imu_sys.offset_gz = (sum_gz / samples) / GYRO_LSB_2000DPS * (M_PI / 180.0f);
}

//==============================================================================
// 2. 初始化
//==============================================================================
void IMU_Fusion_Init(void)
{
    imu_sys.q0 = 1.0f; imu_sys.q1 = 0.0f; imu_sys.q2 = 0.0f; imu_sys.q3 = 0.0f;
    imu_sys.exInt = 0.0f; imu_sys.eyInt = 0.0f; imu_sys.ezInt = 0.0f;
    imu_sys.is_ready = 0;

    // 第一步：校准零偏（车必须静止！）
    IMU_Gyro_Calibration();

    // 第二步：快速收敛对齐重力
    safe_Kp = KP_INIT; 
    safe_Ki = 0.0f; 

    for(int i = 0; i < 200; i++)
    {
        imu660ra_get_acc(); 
        imu660ra_get_gyro(); 
        IMU_Fusion_Update();
    }
    safe_Kp = KP_NORMAL;
    safe_Ki = KI_NORMAL;
    imu_sys.is_ready = 1;
}

//==============================================================================
// 3. 核心更新函数
//==============================================================================
void IMU_Fusion_Update(void)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float halfT = FUSION_DT * 0.5f;

    // --- 1. 数据转换 & 扣除零偏 ---
    float gx = ((float)imu660ra_gyro_x / GYRO_LSB_2000DPS * (M_PI / 180.0f)) - imu_sys.offset_gx;
    float gy = ((float)imu660ra_gyro_y / GYRO_LSB_2000DPS * (M_PI / 180.0f)) - imu_sys.offset_gy;
    float gz = ((float)imu660ra_gyro_z / GYRO_LSB_2000DPS * (M_PI / 180.0f)) - imu_sys.offset_gz;

    float ax = -(float)imu660ra_acc_x;
    float ay = -(float)imu660ra_acc_y;
    float az = -(float)imu660ra_acc_z;

    // --- 2. 加速度计处理 & Mahony 补偿 ---
    float acc_norm_raw = sqrtf(ax*ax + ay*ay + az*az);
    float acc_norm_g = acc_norm_raw / ACC_LSB_8G;

    if(acc_norm_g > ACC_MIN_G && acc_norm_g < ACC_MAX_G)
    {
        norm = InvSqrt(ax*ax + ay*ay + az*az);
        ax *= norm; ay *= norm; az *= norm;

        // 估计重力方向
        vx = 2.0f * (imu_sys.q1 * imu_sys.q3 - imu_sys.q0 * imu_sys.q2);
        vy = 2.0f * (imu_sys.q0 * imu_sys.q1 + imu_sys.q2 * imu_sys.q3);
        vz = imu_sys.q0 * imu_sys.q0 - imu_sys.q1 * imu_sys.q1 - imu_sys.q2 * imu_sys.q2 + imu_sys.q3 * imu_sys.q3;

        // 误差计算
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        imu_sys.exInt += ex * safe_Ki;
        imu_sys.eyInt += ey * safe_Ki;
        // imu_sys.ezInt += ez * safe_Ki;

        gx += safe_Kp * ex + imu_sys.exInt;
        gy += safe_Kp * ey + imu_sys.eyInt;
        // gz += safe_Kp * ez + imu_sys.ezInt;
    }

    // --- 3. Smart Yaw 静态锁死 (在积分前最后执行) ---
    if (fabsf(gz) < YAW_DEADZONE) {
        gz = 0.0f;
    }

    imu_sys.gx = -gx; imu_sys.gy = gy; imu_sys.gz = gz;

    // --- 4. 四元数更新 ---
    float q0 = imu_sys.q0, q1 = imu_sys.q1, q2 = imu_sys.q2, q3 = imu_sys.q3;
    imu_sys.q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    imu_sys.q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    imu_sys.q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    imu_sys.q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = InvSqrt(imu_sys.q0*imu_sys.q0 + imu_sys.q1*imu_sys.q1 + imu_sys.q2*imu_sys.q2 + imu_sys.q3*imu_sys.q3);
    imu_sys.q0 *= norm; imu_sys.q1 *= norm; imu_sys.q2 *= norm; imu_sys.q3 *= norm;

    // --- 5. 欧拉角转换 ---
   // 修改后的逻辑：直接将算法算出的“数学Roll”给“物理Pitch”
   float math_pitch = asinf(-2.0f * (imu_sys.q1 * imu_sys.q3 - imu_sys.q0 * imu_sys.q2)) * 57.29578f;
   float math_roll  = atan2f(2.0f * (imu_sys.q0 * imu_sys.q1 + imu_sys.q2 * imu_sys.q3), 1.0f - 2.0f * (imu_sys.q1 * imu_sys.q1 + imu_sys.q2 * imu_sys.q2)) * 57.29578f;

   imu_sys.pitch = -math_roll;  // 关键：把算出来的 Roll 存进 Pitch
   imu_sys.roll  = math_pitch; // 把算出来的 Pitch 存进 Roll
   imu_sys.yaw   = atan2f(2.0f * (imu_sys.q1 * imu_sys.q2 + imu_sys.q0 * imu_sys.q3), 1.0f - 2.0f * (imu_sys.q2 * imu_sys.q2 + imu_sys.q3 * imu_sys.q3)) * 57.29578f;
}