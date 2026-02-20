#include "KSCal.h"
#include <math.h>
#include <stdio.h>

// === 1. 机械参数定义 (单位: mm) ===
#define LEG_L1  61.0f   // 左主动臂
#define LEG_L2  90.0f   // 左从动臂
#define LEG_L3  90.0f   // 右从动臂 (对称)
#define LEG_L4  61.0f   // 右主动臂
#define LEG_L5  38.0f   // 机身间距


/**
 * @brief 将输入角度转换为目标角度，并限制在 -180 到 180 范围内
 * * 逻辑: output = input - 90
 * 随后进行归一化处理
 * * @param input_angle 输入角度
 * @return float 转换后的角度 (-180, 180]
 */
   static float Left_transform_angle(float input_angle)
    {
    // 1. 执行基本线性变换
    float output = input_angle - 90.0f;

    // 2. 归一化处理：确保角度在 -180 到 180 之间
    
    // 当角度超过 180 时，减去 360 (例如 190 变为 -170)
    while (output > 180.0f) {
        output -= 360.0f;
    }

    // 当角度小于等于 -180 时，加上 360 (例如 -210 变为 150)
    while (output <= -180.0f) {
        output += 360.0f;
    }

    return output;
}

// 角度限幅工具
float fclamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// /**
//  * 电机驱动黑色线部分正解
//  * @brief 五连杆位置正解 (Forward Kinematics)
//  * @param[in]  phi1      左电机角度 (rad)
//  * @param[in]  phi4      右电机角度 (rad)
//  * @param[out] out_L0    输出: 虚拟腿长度 (mm)
//  * @param[out] out_phi0  输出: 虚拟腿角度 (rad)
//  * @return int           0: 成功, -1: 无解(物理不可达)
//  */
// int FiveBar_FwdKinematics(float phi1, float phi4, float* out_L0, float* out_phi0)
// {
//     //算法角度对应到舵机对应角度
//     float Left_Servo_Angle = Left_transform_angle(phi1);
//     float Right_Servo_Angle = phi4+90.0f;

//     // 角度转弧度去算杆子位置和角度
//     phi1 = phi1 * 3.1415926f / 180.0f;
//     phi4 = phi4 * 3.1415926f / 180.0f;

//     // === 2. 计算主动臂末端坐标 (B点 和 D点) ===
//     // 坐标系原点 A(0,0) 在左电机轴心
//     float xB = LEG_L1 * cosf(phi1);
//     float yB = LEG_L1 * sinf(phi1);

//     float xD = LEG_L5 + LEG_L4 * cosf(phi4);
//     float yD = LEG_L4 * sinf(phi4);

//     // === 3. 构建闭环方程求解从动臂角度 (求C点) ===
    
//     // BD 之间的距离平方 (避免过早开根号损失精度)
//     float diff_x = xD - xB;
//     float diff_y = yD - yB;
//     float lBD_sq = diff_x * diff_x + diff_y * diff_y;
    
//     // 构造辅助变量 (对应 A0, B0, C0)
//     float A0 = 2.0f * LEG_L2 * diff_x;
//     float B0 = 2.0f * LEG_L2 * diff_y;
//     float C0 = LEG_L2 * LEG_L2 + lBD_sq - LEG_L3 * LEG_L3;

//     // 判别式检测 (物理可行性保护)
//     float delta = A0 * A0 + B0 * B0 - C0 * C0;

//     if (delta < 0.0f) {
//         // 目标位置物理不可达 (两腿构不成三角形)
//         *out_L0 = 0.0f;
//         *out_phi0 = 0.0f;
//         return -1; // 返回错误代码
//     }
//     Servo_SetAngle(SERVO_CH2_PIN,Left_Servo_Angle);
//     Servo_SetAngle(SERVO_CH3_PIN,Right_Servo_Angle);
//     Servo_SetAngle(SERVO_CH1_PIN,Left_Servo_Angle);
//     Servo_SetAngle(SERVO_CH4_PIN,Right_Servo_Angle);
//     // 计算 phi2 (左从动臂角度)
//     // MATLAB: phi2 = 2 * atan2((B0 + sqrt(delta)), (A0 + C0));
//     // 注意：这里 sqrtf 前面的符号 (+) 决定了膝盖朝向
//     float phi2 = 2.0f * atan2f(B0 + sqrtf(delta), A0 + C0);

//     // 计算 C 点 (末端) 坐标 (相对于左电机A)
//     float xC = xB + LEG_L2 * cosf(phi2);
//     float yC = yB + LEG_L2 * sinf(phi2);

//     // === 4. 转换为虚拟腿坐标 (L0, phi0) ===
//     // 虚拟腿原点在机身中心 (L5/2, 0)
//     float x_center = xC - (LEG_L5 / 2.0f);
//     float y_center = yC;
    
//     // 输出结果
//     *out_L0 = sqrtf(x_center * x_center + y_center * y_center);
//     *out_phi0 = atan2f(y_center, x_center);
//     //转换为C点坐标
//     return 0; // 成功
// }

/**
 * @brief 五连杆位置正解,电机驱动黑色线部分 (Forward Kinematics) - 以机身中心为原点
 * @param[in]  phi1      左电机角度 (rad)
 * @param[in]  phi4      右电机角度 (rad)
 * @param[out] out_L0    输出: 虚拟腿长度 (mm)
 * @param[out] out_phi0  输出: 虚拟腿角度 (rad)
 * @return int           0: 成功, -1: 无解
 */
int FiveBar_FwdKinematics(float phi1, float phi4, float* out_L0, float* out_phi0)
{
    // 1. 角度预处理与弧度转换 (输入为角度制)
    float Left_Servo_Angle = Left_transform_angle(phi1);
    float Right_Servo_Angle = phi4 + 90.0f;

    float rad1 = phi1 * 3.1415926f / 180.0f;
    float rad4 = phi4 * 3.1415926f / 180.0f;

    // 2. 计算主动臂末端坐标 (相对于机身中心原点)
    // 左电机 A 位于 (-L5/2, 0), 右电机 E 位于 (L5/2, 0)
    float offset = LEG_L5 / 2.0f;

    float xB = -offset + LEG_L1 * cosf(rad1);
    float yB = LEG_L1 * sinf(rad1);

    float xD = offset + LEG_L4 * cosf(rad4);
    float yD = LEG_L4 * sinf(rad4);

    // 3. 求解从动臂交点 C (末端)
    float diff_x = xD - xB;
    float diff_y = yD - yB;
    float lBD_sq = diff_x * diff_x + diff_y * diff_y;
    
    // 几何判别式计算
    float A0 = 2.0f * LEG_L2 * diff_x;
    float B0 = 2.0f * LEG_L2 * diff_y;
    float C0 = LEG_L2 * LEG_L2 + lBD_sq - LEG_L3 * LEG_L3;

    float delta = A0 * A0 + B0 * B0 - C0 * C0;

    if (delta < 0.0f) {
        *out_L0 = 0.0f; *out_phi0 = 0.0f;
        return -1; 
    }

    // 执行舵机控制
    Servo_SetAngle(SERVO_CH2_PIN,Left_Servo_Angle);
    Servo_SetAngle(SERVO_CH3_PIN,Right_Servo_Angle);
    Servo_SetAngle(SERVO_CH1_PIN,Left_Servo_Angle);
    Servo_SetAngle(SERVO_CH4_PIN,Right_Servo_Angle);

    // 计算 phi2 并得出 C 点坐标 (此时 C 已经是相对于中心原点的坐标)
    float phi2 = 2.0f * atan2f(B0 + sqrtf(delta), A0 + C0);
    float xC = xB + LEG_L2 * cosf(phi2);
    float yC = yB + LEG_L2 * sinf(phi2);

    // 4. 计算虚拟腿极坐标 (极点在机身中心)
    *out_L0 = sqrtf(xC * xC + yC * yC);
    *out_phi0 = atan2f(yC, xC);

    return 0;
}
/**
 * @brief 五连杆位置逆解 (Inverse Kinematics) 
 * @param[in]  L0       虚拟腿长度 (mm)
 * @param[in]  phi0     虚拟腿角度 (rad)
 * @param[out] out_phi1 输出: 左电机角度 (rad, 范围-pi~pi)
 * @param[out] out_phi4 输出: 右电机角度 (rad, 范围-pi~pi)
 * @return int          0: 成功, -1: 无解(超出工作空间)
 */
int FiveBar_InverseKinematics(float L0, float phi0, float* out_phi1, float* out_phi4)
{
    // === 1. 将虚拟腿坐标转换为末端 C 点的全局坐标 ===
    // 假设虚拟腿原点在机身中心 (L5/2, 0)
    float x_center = L0 * cosf(phi0);
    float y_center = L0 * sinf(phi0);

    // 转换到以左电机 A(0,0) 为原点的全局坐标系
    float xC = x_center + (LEG_L5 / 2.0f);
    float yC = y_center;

    // === 2. 计算左电机 A 相关的几何参数 ===
    float Lac_sq = xC * xC + yC * yC;
    float Lac = sqrtf(Lac_sq);

    // 避免除零风险
    if (Lac < 0.0001f) return -1; 

    // cos(angle_A) = (l1^2 + Lac^2 - l2^2) / (2 * l1 * Lac)
    float cos_angle_L = (LEG_L1 * LEG_L1 + Lac_sq - LEG_L2 * LEG_L2) / (2.0f * LEG_L1 * Lac);

    // === 3. 计算右电机 E 相关的几何参数 ===
    // 右电机 E 的坐标是 (L5, 0)
    float x_EC = xC - LEG_L5;
    float y_EC = yC;
    
    float Lec_sq = x_EC * x_EC + y_EC * y_EC;
    float Lec = sqrtf(Lec_sq);

    if (Lec < 0.0001f) return -1;

    // cos(angle_E) = (l4^2 + Lec^2 - l3^2) / (2 * l4 * Lec)
    float cos_angle_R = (LEG_L4 * LEG_L4 + Lec_sq - LEG_L3 * LEG_L3) / (2.0f * LEG_L4 * Lec);

    // === 4. 合法性检查与限幅 ===
    if (fabsf(cos_angle_L) > 1.0001f || fabsf(cos_angle_R) > 1.0001f) {
        return -1; // 目标点物理不可达
    }

    // 安全限幅
    if (cos_angle_L > 1.0f) cos_angle_L = 1.0f; else if (cos_angle_L < -1.0f) cos_angle_L = -1.0f;
    if (cos_angle_R > 1.0f) cos_angle_R = 1.0f; else if (cos_angle_R < -1.0f) cos_angle_R = -1.0f;

    // === 5. 计算并归一化角度 (关键修改点) ===
    
    // 计算左电机原始角度
    float phi1_raw = atan2f(yC, xC) + acosf(cos_angle_L);
    // 使用 atan2f(sin, cos) 将角度强制收拢到 (-pi, pi]
    *out_phi1 = atan2f(sinf(phi1_raw), cosf(phi1_raw));

    // 计算右电机原始角度
    float phi4_raw = atan2f(y_EC, x_EC) - acosf(cos_angle_R);
    // 使用 atan2f(sin, cos) 将角度强制收拢到 (-pi, pi]
    *out_phi4 = atan2f(sinf(phi4_raw), cosf(phi4_raw));

    return 0; 
}
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 左边腿角度版五连杆逆解接口
 * @param[in]  L0_mm      虚拟腿长度 (mm)
 * @param[in]  phi0_deg   虚拟腿角度 (度, 90度垂直向下)
 * @param[out] out_deg1   左电机目标角度 (度)
 * @param[out] out_deg4   右电机目标角度 (度)
 * @return int            0: 成功, -1: 无解
 */
int Left_FiveBar_IK_Degree_Interface(float L0_mm, float phi0_deg, float* out_deg1, float* out_deg4)
{
    float phi0_rad, phi1_rad, phi4_rad;
    int status;
    phi0_deg = 180 - phi0_deg;
    // 1. 输入转换：角度 -> 弧度
    phi0_rad = phi0_deg * (M_PI / 180.0f);

    // 2. 调用原有的核心计算函数 (不改动原有逻辑)
    status = FiveBar_InverseKinematics(L0_mm, phi0_rad, &phi1_rad, &phi4_rad);

    // 3. 结果处理
    if (status == 0) {
        // 4. 输出转换：弧度 -> 角度
        *out_deg1 = phi1_rad * (180.0f / M_PI);
        *out_deg4 = phi4_rad * (180.0f / M_PI);
        float Left_Angle = phi1_rad * (180.0f / M_PI);
        float Right_Angle = phi4_rad * (180.0f / M_PI);
        float Left_Servo_Angle = Left_transform_angle(Left_Angle);
        float Right_Servo_Angle = Right_Angle + 90.0f;


        
        Servo_SetTarget(SERVO_ID_1_LD,Left_Servo_Angle);
        Servo_SetAngle_Direct(SERVO_CH4_PIN,Right_Servo_Angle);
        // Servo_SetAngle(SERVO_CH1_PIN,Left_Servo_Angle);
        // Servo_SetAngle(SERVO_CH4_PIN,Right_Servo_Angle);
        return 0;
    } else {
        return -1; // 保持失败状态返回
    }
}
/**
 * @brief 右边边腿角度版五连杆逆解接口
 * @param[in]  L0_mm      虚拟腿长度 (mm)
 * @param[in]  phi0_deg   虚拟腿角度 (度, 90度垂直向下)
 * @param[out] out_deg1   左电机目标角度 (度)
 * @param[out] out_deg4   右电机目标角度 (度)
 * @return int            0: 成功, -1: 无解
 */
int Right_FiveBar_IK_Degree_Interface(float L0_mm, float phi0_deg, float* out_deg1, float* out_deg4)
{
    float phi0_rad, phi1_rad, phi4_rad;
    int status;
    // 1. 输入转换：角度 -> 弧度
    phi0_rad = phi0_deg * (M_PI / 180.0f);

    // 2. 调用原有的核心计算函数 (不改动原有逻辑)
    status = FiveBar_InverseKinematics(L0_mm, phi0_rad, &phi1_rad, &phi4_rad);

    // 3. 结果处理
    if (status == 0) {
        // 4. 输出转换：弧度 -> 角度
        *out_deg1 = phi1_rad * (180.0f / M_PI);
        *out_deg4 = phi4_rad * (180.0f / M_PI);
        float Left_Angle = phi1_rad * (180.0f / M_PI);
        float Right_Angle = phi4_rad * (180.0f / M_PI);
        float Left_Servo_Angle = Left_transform_angle(Left_Angle);
        float Right_Servo_Angle = Right_Angle + 90.0f;
        

        // Servo_SetAngle(SERVO_CH2_PIN,Left_Servo_Angle);
        // Servo_SetAngle(SERVO_CH3_PIN,Right_Servo_Angle);
        Servo_SetTarget(SERVO_ID_2_RU,Left_Servo_Angle);
        Servo_SetTarget(SERVO_ID_3_RD,Right_Servo_Angle);
        return 0;
    } else {
        return -1; // 保持失败状态返回
    }
}
int FiveBar_IK_Degree_Interface(float L0_mm, float phi0_deg, float* out_deg1, float* out_deg4)
{
    float phi0_rad, phi1_rad, phi4_rad;
    int status;
    // 1. 输入转换：角度 -> 弧度
    phi0_rad = phi0_deg * (M_PI / 180.0f);

    // 2. 调用原有的核心计算函数 (不改动原有逻辑)
    status = FiveBar_InverseKinematics(L0_mm, phi0_rad, &phi1_rad, &phi4_rad);

    // 3. 结果处理
    if (status == 0) {
        // 4. 输出转换：弧度 -> 角度
        *out_deg1 = phi1_rad * (180.0f / M_PI);
        *out_deg4 = phi4_rad * (180.0f / M_PI);
        float Left_Angle = phi1_rad * (180.0f / M_PI);
        float Right_Angle = phi4_rad * (180.0f / M_PI);
        float Left_Servo_Angle = Left_transform_angle(Left_Angle);
        float Right_Servo_Angle = Right_Angle + 90.0f;
        Servo_SetAngle(SERVO_CH2_PIN,Left_Servo_Angle);
        Servo_SetAngle(SERVO_CH3_PIN,Right_Servo_Angle);
        Servo_SetAngle(SERVO_CH1_PIN,Left_Servo_Angle);
        Servo_SetAngle(SERVO_CH4_PIN,Right_Servo_Angle);
        return 0;
    } else {
        return -1; // 保持失败状态返回
    }
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
    
    *out_phi1 = theta_A + acosf(cos_angle_L);
    *out_phi4 = theta_E - acosf(cos_angle_R);

    return 0; // 成功
}