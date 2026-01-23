#include "chassic.h"
#include <math.h>
#include <stdio.h>

// === 1. 机械参数定义 (单位: mm) ===
// 使用 float 宏定义，防止嵌入式设备进行双精度运算降低效率
#define LEG_L1  61.0f   // 左主动臂
#define LEG_L2  90.0f   // 左从动臂
#define LEG_L3  90.0f   // 右从动臂 (对称)
#define LEG_L4  61.0f   // 右主动臂
#define LEG_L5  38.0f   // 机身间距


/**
 * @brief 五连杆位置正解 (Forward Kinematics)
 * @param[in]  phi1      左电机角度 (rad)
 * @param[in]  phi4      右电机角度 (rad)
 * @param[out] out_L0    输出: 虚拟腿长度 (mm)
 * @param[out] out_phi0  输出: 虚拟腿角度 (rad)
 * @return int           0: 成功, -1: 无解(物理不可达)
 */
int FiveBar_FwdKinematics(float phi1, float phi4, float* out_L0, float* out_phi0)
{
    // === 2. 计算主动臂末端坐标 (B点 和 D点) ===
    // 坐标系原点 A(0,0) 在左电机轴心
    float xB = LEG_L1 * cosf(phi1);
    float yB = LEG_L1 * sinf(phi1);

    float xD = LEG_L5 + LEG_L4 * cosf(phi4);
    float yD = LEG_L4 * sinf(phi4);

    // === 3. 构建闭环方程求解从动臂角度 (求C点) ===
    
    // BD 之间的距离平方 (避免过早开根号损失精度)
    float diff_x = xD - xB;
    float diff_y = yD - yB;
    float lBD_sq = diff_x * diff_x + diff_y * diff_y;
    
    // 构造辅助变量 (对应 A0, B0, C0)
    float A0 = 2.0f * LEG_L2 * diff_x;
    float B0 = 2.0f * LEG_L2 * diff_y;
    float C0 = LEG_L2 * LEG_L2 + lBD_sq - LEG_L3 * LEG_L3;

    // 判别式检测 (物理可行性保护)
    float delta = A0 * A0 + B0 * B0 - C0 * C0;

    if (delta < 0.0f) {
        // 目标位置物理不可达 (两腿构不成三角形)
        *out_L0 = 0.0f;
        *out_phi0 = 0.0f;
        return -1; // 返回错误代码
    }

    // 计算 phi2 (左从动臂角度)
    // MATLAB: phi2 = 2 * atan2((B0 + sqrt(delta)), (A0 + C0));
    // 注意：这里 sqrtf 前面的符号 (+) 决定了膝盖朝向
    float phi2 = 2.0f * atan2f(B0 + sqrtf(delta), A0 + C0);

    // 计算 C 点 (末端) 坐标 (相对于左电机A)
    float xC = xB + LEG_L2 * cosf(phi2);
    float yC = yB + LEG_L2 * sinf(phi2);

    // === 4. 转换为虚拟腿坐标 (L0, phi0) ===
    // 虚拟腿原点在机身中心 (L5/2, 0)
    float x_center = xC - (LEG_L5 / 2.0f);
    float y_center = yC;

    // 输出结果
    *out_L0 = sqrtf(x_center * x_center + y_center * y_center);
    *out_phi0 = atan2f(y_center, x_center);
    
    //转换为C点坐标
    return 0; // 成功
}

/**
 * @brief 五连杆位置逆解 (Inverse Kinematics)
 * @param[in]  L0        虚拟腿长度 (mm)
 * @param[in]  phi0      虚拟腿角度 (rad)
 * @param[out] out_phi1  输出: 左电机角度 (rad)
 * @param[out] out_phi4  输出: 右电机角度 (rad)
 * @return int           0: 成功, -1: 无解(超出工作空间)
 */
int FiveBar_InverseKinematics(float L0, float phi0, float* out_phi1, float* out_phi4)
{
    // === 1. 将虚拟腿坐标转换为末端 C 点的全局坐标 ===
    // 假设虚拟腿原点在机身中心 (L5/2, 0)
    // MATLAB: x_center = L0 * cos(phi0);
    float x_center = L0 * cosf(phi0);
    float y_center = L0 * sinf(phi0);

    // 转换到以左电机 A(0,0) 为原点的全局坐标系
    float xC = x_center + (LEG_L5 / 2.0f);
    float yC = y_center;

    // === 2. 计算左电机角度 phi1 (解三角形 O-A-C) ===
    // AC 的长度平方
    float Lac_sq = xC * xC + yC * yC;
    float Lac = sqrtf(Lac_sq);

    // 使用余弦定理求三角形内角
    // cos(angle_A) = (l1^2 + Lac^2 - l2^2) / (2 * l1 * Lac)
    float cos_angle_L = (LEG_L1 * LEG_L1 + Lac_sq - LEG_L2 * LEG_L2) / (2.0f * LEG_L1 * Lac);

    // === 3. 计算右电机角度 phi4 (解三角形 O-E-C) ===
    // 右电机 E 的坐标是 (L5, 0)
    float x_EC = xC - LEG_L5;
    float y_EC = yC;
    
    // EC 的长度平方
    float Lec_sq = x_EC * x_EC + y_EC * y_EC;
    float Lec = sqrtf(Lec_sq);

    // cos(angle_E) = (l4^2 + Lec^2 - l3^2) / (2 * l4 * Lec)
    float cos_angle_R = (LEG_L4 * LEG_L4 + Lec_sq - LEG_L3 * LEG_L3) / (2.0f * LEG_L4 * Lec);

    // === 4. 合法性检查 ===
    // 检查余弦值是否在 [-1, 1] 之间 (预留微小误差空间)
    if (fabsf(cos_angle_L) > 1.0001f || fabsf(cos_angle_R) > 1.0001f) {
        return -1; // 无解
    }

    // 安全限幅，防止 acos 出错
    cos_angle_L = LIMIT(cos_angle_L, -1.0f, 1.0f);
    cos_angle_R = LIMIT(cos_angle_R, -1.0f, 1.0f);

    // === 5. 计算最终角度 ===
    // 左边：基础角 + 内角 (膝盖向外/上)
    float base_angle_L = atan2f(yC, xC);
    *out_phi1 = base_angle_L + acosf(cos_angle_L);

    // 右边：基础角 - 内角 (膝盖向外/上)
    float base_angle_R = atan2f(y_EC, x_EC);
    *out_phi4 = base_angle_R - acosf(cos_angle_R);

    return 0; // 成功
}

/**
 * @brief 五连杆 XY 逆解 (XY Inverse Kinematics)
 * @param[in]  x         目标 X 坐标 (mm)
 * @param[in]  y         目标 Y 坐标 (mm)
 * @param[out] out_phi1  输出: 左电机角度 (rad)
 * @param[out] out_phi4  输出: 右电机角度 (rad)
 * @return int           0: 成功, -1: 无解(超出工作空间)
 */
int FiveBar_XY_InverseKinematics(float x, float y, float* out_phi1, float* out_phi4)
{
    float w = LEG_L5 / 2.0f; // 半个机身宽

    // === 1. 计算左电机角度 phi1 ===
    // 左电机 A 坐标: (-w, 0)
    // 向量 AC = (x - (-w), y - 0) = (x+w, y)
    float x_AC = x + w;
    float y_AC = y;
    
    // AC 长度平方及长度
    float Lac_sq = x_AC * x_AC + y_AC * y_AC;
    float Lac = sqrtf(Lac_sq);

    // 基础角度 (向量 AC 相对于 X 轴的角度)
    float theta_A = atan2f(y_AC, x_AC);

    // 三角形内角 (余弦定理)
    // cos(alpha) = (l1^2 + Lac^2 - l2^2) / (2 * l1 * Lac)
    float cos_angle_L = (LEG_L1 * LEG_L1 + Lac_sq - LEG_L2 * LEG_L2) / (2.0f * LEG_L1 * Lac);

    // === 2. 计算右电机角度 phi4 ===
    // 右电机 E 坐标: (w, 0)
    // 向量 EC = (x - w, y - 0) = (x-w, y)
    float x_EC = x - w;
    float y_EC = y;

    // EC 长度平方及长度
    float Lec_sq = x_EC * x_EC + y_EC * y_EC;
    float Lec = sqrtf(Lec_sq);

    // 基础角度 (向量 EC 相对于 X 轴的角度)
    float theta_E = atan2f(y_EC, x_EC);

    // 三角形内角 (余弦定理)
    // cos(beta) = (l4^2 + Lec^2 - l3^2) / (2 * l4 * Lec)
    float cos_angle_R = (LEG_L4 * LEG_L4 + Lec_sq - LEG_L3 * LEG_L3) / (2.0f * LEG_L4 * Lec);

    // === 3. 合法性检查 ===
    // 检查余弦值是否在 [-1, 1] 之间 (预留微小误差空间)
    if (fabsf(cos_angle_L) > 1.0001f || fabsf(cos_angle_R) > 1.0001f) {
        return -1; // 无解
    }

    // 安全限幅，防止 acos 出错
    cos_angle_L = LIMIT(cos_angle_L, -1.0f, 1.0f);
    cos_angle_R = LIMIT(cos_angle_R, -1.0f, 1.0f);

    // === 4. 角度合成 (构型选择) ===
    // 对应 MATLAB 代码:
    // phi1 = theta_A + acos(cos_angle_L);
    // phi4 = theta_E - acos(cos_angle_R);
    
    *out_phi1 = theta_A + acosf(cos_angle_L);
    *out_phi4 = theta_E - acosf(cos_angle_R);

    return 0; // 成功
}