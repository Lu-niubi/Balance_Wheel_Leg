#include "IMU_Deal.h"
#include <math.h>

imu_fusion_t imu_sys;

static float s_beta = BETA_NORMAL;  // 当前 Madgwick beta

static float InvSqrt(float x)
{
    if (x <= 0) return 0;
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//==============================================================================
// 1. 陀螺仪自动校准 (静止校准)
//==============================================================================
void IMU_Gyro_Calibration(void)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const uint16_t samples = 3000;

    for (uint16_t i = 0; i < samples; i++)
    {
        imu660ra_get_gyro();
        sum_gx += (float)imu660ra_gyro_x;
        sum_gy += (float)imu660ra_gyro_y;
        sum_gz += (float)imu660ra_gyro_z;
        system_delay_ms(1);
    }

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

    // 第二步：大 beta 快速收敛对齐重力
    s_beta = BETA_INIT;
    for (int i = 0; i < 200; i++)
    {
        imu660ra_get_acc();
        imu660ra_get_gyro();
        IMU_Fusion_Update();
    }
    s_beta = BETA_NORMAL;
    imu_sys.is_ready = 1;
}

//==============================================================================
// 3. Madgwick 核心更新函数
//==============================================================================
void IMU_Fusion_Update(void)
{
    float halfT = FUSION_DT * 0.5f;

    // --- 1. 数据转换 & 扣除零偏 ---
    float gx = ((float)imu660ra_gyro_x / GYRO_LSB_2000DPS * (M_PI / 180.0f)) - imu_sys.offset_gx;
    float gy = ((float)imu660ra_gyro_y / GYRO_LSB_2000DPS * (M_PI / 180.0f)) - imu_sys.offset_gy;
    float gz = ((float)imu660ra_gyro_z / GYRO_LSB_2000DPS * (M_PI / 180.0f)) - imu_sys.offset_gz;

    float ax = -(float)imu660ra_acc_x;
    float ay = -(float)imu660ra_acc_y;
    float az = -(float)imu660ra_acc_z;

    float q0 = imu_sys.q0, q1 = imu_sys.q1, q2 = imu_sys.q2, q3 = imu_sys.q3;

    // --- 2. Madgwick 梯度下降修正 ---
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    float acc_norm_g = acc_norm / ACC_LSB_8G;

    // 动态 beta：仅在极端情况（完全失重/超过2倍重力）才压制修正
    // 普通运动中保持全 beta，确保动后能收敛回正确姿态
    float dyn_beta = s_beta;
    if (acc_norm_g < 0.3f || acc_norm_g > 3.0f) dyn_beta = 0.0f;  // 完全不信任
    else if (acc_norm_g < 0.5f || acc_norm_g > 2.5f) dyn_beta = s_beta * 0.2f;

    if (acc_norm > 0.0f)
    {
        // 归一化加速度
        float inv_acc = InvSqrt(ax*ax + ay*ay + az*az);
        ax *= inv_acc; ay *= inv_acc; az *= inv_acc;

        // 预计算常用乘积
        float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
        float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
        float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

        // 目标函数关于四元数的梯度（仅重力参考）
        float s0 = _4q0*q2q2 + 2.0f*q2*ax  + _4q0*q1q1 - 2.0f*q1*ay;
        float s1 = _4q1*q3q3 - 2.0f*q3*ax  + 4.0f*q0q0*q1 - 2.0f*q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        float s2 = 4.0f*q0q0*q2 + 2.0f*q0*ax + _4q2*q3q3 - 2.0f*q3*ay  - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        float s3 = 4.0f*q1q1*q3 - 2.0f*q1*ax + 4.0f*q2q2*q3 - 2.0f*q2*ay;

        // 归一化梯度
        float inv_s = InvSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= inv_s; s1 *= inv_s; s2 *= inv_s; s3 *= inv_s;

        // 梯度下降修正陀螺仪（yaw 方向 s0/s3 不修正，无磁力计参考）
        gx -= dyn_beta * s1;
        gy -= dyn_beta * s2;
        gz -= dyn_beta * s3;
    }

    // --- 3. Smart Yaw 静态锁死 ---
    if (fabsf(gz) < YAW_DEADZONE)
    {
        gz = 0.0f;
    }

    imu_sys.gx = -gx; imu_sys.gy = gy; imu_sys.gz = gz;

    // --- 4. 四元数积分更新 ---
    imu_sys.q0 += (-q1*gx - q2*gy - q3*gz) * halfT;
    imu_sys.q1 += ( q0*gx + q2*gz - q3*gy) * halfT;
    imu_sys.q2 += ( q0*gy - q1*gz + q3*gx) * halfT;
    imu_sys.q3 += ( q0*gz + q1*gy - q2*gx) * halfT;

    // 归一化四元数
    float norm = InvSqrt(imu_sys.q0*imu_sys.q0 + imu_sys.q1*imu_sys.q1 +
                         imu_sys.q2*imu_sys.q2 + imu_sys.q3*imu_sys.q3);
    imu_sys.q0 *= norm; imu_sys.q1 *= norm;
    imu_sys.q2 *= norm; imu_sys.q3 *= norm;

    // --- 5. 欧拉角转换 (保持与原来完全相同的轴映射) ---
    float math_pitch = asinf(-2.0f * (imu_sys.q1*imu_sys.q3 - imu_sys.q0*imu_sys.q2)) * 57.29578f;
    float math_roll  = atan2f(2.0f  * (imu_sys.q0*imu_sys.q1 + imu_sys.q2*imu_sys.q3),
                              1.0f  - 2.0f*(imu_sys.q1*imu_sys.q1 + imu_sys.q2*imu_sys.q2)) * 57.29578f;

    imu_sys.pitch = -math_roll;   // 算法Roll → 物理Pitch（保持原有映射）
    imu_sys.roll  =  math_pitch;  // 算法Pitch → 物理Roll
    imu_sys.yaw   = atan2f(2.0f * (imu_sys.q1*imu_sys.q2 + imu_sys.q0*imu_sys.q3),
                           1.0f - 2.0f*(imu_sys.q2*imu_sys.q2 + imu_sys.q3*imu_sys.q3)) * 57.29578f;
}
