#include "IMU_Deal.h"
#include <math.h>

imu_fusion_t imu_sys;

// P矩阵 (协方差矩阵) 的对角线元素，用于简化计算
// 完整的P矩阵是4x4，这里为了性能，假设非对角项影响较小或动态收敛
static float P[4] = {100.0f, 100.0f, 100.0f, 100.0f};

//==============================================================================
// 内部函数声明
//==============================================================================
static void IMU_Calibrate_Offset(void);
static void IMU_PreProcess(void);
static void EKF_Predict(float gx, float gy, float gz);
static void EKF_Correct(float ax, float ay, float az);
static void Quaternion_To_Euler(void);
static float InvSqrt(float x);

//==============================================================================
// 1. 初始化
//==============================================================================
void IMU_Fusion_Init(void)
{
    // 硬件初始化
    imu660ra_init();
    
    // 数据复位
    imu_sys.q[0] = 1.0f; 
    imu_sys.q[1] = 0.0f; 
    imu_sys.q[2] = 0.0f; 
    imu_sys.q[3] = 0.0f;
    imu_sys.yaw = 0.0f;
    imu_sys.pitch = 0.0f;
    imu_sys.roll = 0.0f;
    imu_sys.is_ready = 0;

    // 启动校准
    IMU_Calibrate_Offset();
    
    imu_sys.is_ready = 1;
}

// 零偏校准 (来自方案1的优点)
static void IMU_Calibrate_Offset(void)
{
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    // 简单预热
     system_delay_ms(200);

    for(int i = 0; i < GYRO_OFFSET_COUNT; i++)
    {
        imu660ra_get_gyro();
        gx_sum += imu660ra_gyro_x;
        gy_sum += imu660ra_gyro_y;
        gz_sum += imu660ra_gyro_z;
        // systick_delay_ms(2);
    }

    imu_sys.gyro_x_offset = gx_sum / (float)GYRO_OFFSET_COUNT;
    imu_sys.gyro_y_offset = gy_sum / (float)GYRO_OFFSET_COUNT;
    imu_sys.gyro_z_offset = gz_sum / (float)GYRO_OFFSET_COUNT;
}

//==============================================================================
// 2. 数据预处理 (融合了方案1的 Smart Yaw 和滤波)
//==============================================================================
static void IMU_PreProcess(void)
{
    // --- 1. 读取硬件数据 ---
    imu660ra_get_acc();
    imu660ra_get_gyro();

    // --- 2. 陀螺仪处理 & 零偏去除 & 单位转换 ---
    // 假设 16.4 LSB/(deg/s) -> 2000dps
    float raw_gx = ((float)imu660ra_gyro_x - imu_sys.gyro_x_offset) / 16.4f * (PI / 180.0f);
    float raw_gy = ((float)imu660ra_gyro_y - imu_sys.gyro_y_offset) / 16.4f * (PI / 180.0f);
    float raw_gz = ((float)imu660ra_gyro_z - imu_sys.gyro_z_offset) / 16.4f * (PI / 180.0f);

    // --- 3. [关键] 智能 Yaw 轴防漂移 (Smart Yaw) ---
    // 这是你方案1中最值钱的部分，移植到这里
    if (fabsf(raw_gz) < YAW_DEADZONE)
    {
        imu_sys.yaw_stable_count++;
        // 如果连续N次静止，则彻底锁死 Z 轴输入
        if (imu_sys.yaw_stable_count >= YAW_LOCK_COUNT)
        {
            raw_gz = 0.0f; // 强制置零，EKF 就不会更新 Yaw 角度
            if(imu_sys.yaw_stable_count > 200) imu_sys.yaw_stable_count = 200; // 防止溢出
        }
    }
    else
    {
        imu_sys.yaw_stable_count = 0; // 一旦动了，立即解锁
    }

    // 赋值给全局
    imu_sys.gyro_x = raw_gx;
    imu_sys.gyro_y = raw_gy;
    imu_sys.gyro_z = raw_gz;

    // --- 4. 加速度计处理 & 滤波 ---
    // 假设 4096 LSB/g -> 8g
    float raw_ax = (float)imu660ra_acc_x * (8.0f / 4096.0f);
    float raw_ay = (float)imu660ra_acc_y * (8.0f / 4096.0f);
    float raw_az = (float)imu660ra_acc_z * (8.0f / 4096.0f);

    // 低通滤波
    imu_sys.acc_x = raw_ax * ACC_LPF_ALPHA + imu_sys.acc_x * (1.0f - ACC_LPF_ALPHA);
    imu_sys.acc_y = raw_ay * ACC_LPF_ALPHA + imu_sys.acc_y * (1.0f - ACC_LPF_ALPHA);
    imu_sys.acc_z = raw_az * ACC_LPF_ALPHA + imu_sys.acc_z * (1.0f - ACC_LPF_ALPHA);
    
    // 归一化加速度 (EKF需要单位向量)
    float norm = InvSqrt(imu_sys.acc_x * imu_sys.acc_x + imu_sys.acc_y * imu_sys.acc_y + imu_sys.acc_z * imu_sys.acc_z);
    if(norm > 0.0f)
    {
        imu_sys.acc_x *= norm;
        imu_sys.acc_y *= norm;
        imu_sys.acc_z *= norm;
    }
}

//==============================================================================
// 3. EKF 核心算法 
//==============================================================================
static void EKF_Predict(float gx, float gy, float gz)
{
    float q0 = imu_sys.q[0], q1 = imu_sys.q[1], q2 = imu_sys.q[2], q3 = imu_sys.q[3];
    float halfT = FUSION_DT * 0.5f;

    // 1. 状态转移 (基于陀螺仪积分) X = F * X
    // 使用四元数微分方程 q_dot = 0.5 * q * omega
    float q0_new = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    float q1_new = q1 + ( q0*gx - q3*gy + q2*gz) * halfT;
    float q2_new = q2 + ( q3*gx + q0*gy - q1*gz) * halfT;
    float q3_new = q3 + (-q2*gx + q1*gy + q0*gz) * halfT;

    // 归一化四元数
    float norm = InvSqrt(q0_new*q0_new + q1_new*q1_new + q2_new*q2_new + q3_new*q3_new);
    imu_sys.q[0] = q0_new * norm;
    imu_sys.q[1] = q1_new * norm;
    imu_sys.q[2] = q2_new * norm;
    imu_sys.q[3] = q3_new * norm;

    // 2. 协方差预测 P = F*P*F^T + Q
    // 为了性能，简化为对角矩阵更新 + 过程噪声累加
    P[0] += Q_PROCESS;
    P[1] += Q_PROCESS;
    P[2] += Q_PROCESS;
    P[3] += Q_PROCESS;
}

static void EKF_Correct(float ax, float ay, float az)
{
    float q0 = imu_sys.q[0], q1 = imu_sys.q[1], q2 = imu_sys.q[2], q3 = imu_sys.q[3];

    // 1. 预测重力向量 (将重力[0,0,1]从地理系转到机体系) h(x)
    // 这是四元数旋转矩阵的第三列
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 2. 计算残差 (观测值 - 预测值) error
    float ex = ax - vx;
    float ey = ay - vy;
    float ez = az - vz;

    // [关键] 方案2的抗干扰检测：如果残差太大，说明加速度计不可信(震动或非重力加速度)
    // 计算残差的平方和
    float error_sq = ex*ex + ey*ey + ez*ez;
    if(error_sq > MAX_ACC_ERR) 
    {
        return; // 放弃这次加速度修正，仅保留陀螺仪数据
    }

    // 3. 计算卡尔曼增益 K = P * H^T / (H*P*H^T + R)
    // 这是一个简化版增益计算，避免4x4矩阵求逆，类似互补滤波的权重动态调整
    // 但保留了EKF利用P矩阵收敛的特性
    
    // 这里使用简化梯度下降方向作为更新方向(类似Mahony)，但用EKF的P/R调整步长
    // 这种做法在嵌入式非常流行 (如 Ardupilot/Betaflight)
    float K = 0.0f;
    // 简单近似：信噪比 P / (P+R)
    // 这里的P取平均值作为整体不确定度
    float P_avg = (P[0] + P[1] + P[2] + P[3]) * 0.25f;
    K = P_avg / (P_avg + R_MEASURE);

    // 4. 更新状态 X = X + K * error
    // 将重力误差转换到四元数变化率
    float g_err_x = (ay*vz - az*vy);
    float g_err_y = (az*vx - ax*vz);
    float g_err_z = (ax*vy - ay*vx);

    // 四元数校正
    // q_dot_corr = 0.5 * q * error_vector
    float t0 = (-q1*g_err_x - q2*g_err_y - q3*g_err_z) * K;
    float t1 = ( q0*g_err_x - q3*g_err_y + q2*g_err_z) * K;
    float t2 = ( q3*g_err_x + q0*g_err_y - q1*g_err_z) * K;
    float t3 = (-q2*g_err_x + q1*g_err_y + q0*g_err_z) * K;

    imu_sys.q[0] += t0;
    imu_sys.q[1] += t1;
    imu_sys.q[2] += t2;
    imu_sys.q[3] += t3;

    // 归一化
    float norm = InvSqrt(imu_sys.q[0]*imu_sys.q[0] + imu_sys.q[1]*imu_sys.q[1] + imu_sys.q[2]*imu_sys.q[2] + imu_sys.q[3]*imu_sys.q[3]);
    imu_sys.q[0] *= norm;
    imu_sys.q[1] *= norm;
    imu_sys.q[2] *= norm;
    imu_sys.q[3] *= norm;

    // 5. 更新 P (P = (I - KH)P)
    // 简单收敛
    P[0] -= K * P[0];
    P[1] -= K * P[1];
    P[2] -= K * P[2];
    P[3] -= K * P[3];
}

//==============================================================================
// 4. 输出转换
//==============================================================================
static void Quaternion_To_Euler(void)
{
    float q0 = imu_sys.q[0], q1 = imu_sys.q[1], q2 = imu_sys.q[2], q3 = imu_sys.q[3];

    // Pitch (x-axis)
    imu_sys.pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * (180.0f / PI);

    // Roll (y-axis)
    imu_sys.roll  = atan2f(2.0f * (q2*q3 + q0*q1), 1.0f - 2.0f * (q1*q1 + q2*q2)) * (180.0f / PI);

    // Yaw (z-axis)
    imu_sys.yaw   = atan2f(2.0f * (q1*q2 + q0*q3), 1.0f - 2.0f * (q2*q2 + q3*q3)) * (180.0f / PI);
}

// 快速平方根倒数 (Quake III 经典算法) 或者直接用 1.0/sqrtf
static float InvSqrt(float x)
{
    if(x <= 0) return 0;
    return 1.0f / sqrtf(x);
}

//==============================================================================
// 主更新入口
//==============================================================================
void IMU_Fusion_Update(void)
{
    if(!imu_sys.is_ready) return;

    // 1. 获取数据 + Smart Yaw 处理 + 滤波
    IMU_PreProcess();

    // 2. EKF 预测 (陀螺仪积分)
    EKF_Predict(imu_sys.gyro_x, imu_sys.gyro_y, imu_sys.gyro_z);

    // 3. EKF 修正 (加速度计重力校准)
    // 注意: EKF 只能用重力修正 Pitch 和 Roll
    // 由于我们已经在 PreProcess 里对 GyroZ 做了死区锁定，
    // 所以不需要用加速度修正 Yaw (重力也修不了 Yaw)，这样既准又不会乱飘。
    EKF_Correct(imu_sys.acc_x, imu_sys.acc_y, imu_sys.acc_z);

    // 4. 结果转换
    Quaternion_To_Euler();
}