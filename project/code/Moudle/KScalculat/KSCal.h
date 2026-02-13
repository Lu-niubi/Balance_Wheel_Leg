#ifndef __KS_H
#define __KS_H

#include "zf_common_headfile.h"
#include "IMU_Deal.h"
#include "controller.h"
#include "math.h"
#include "Servo.h"

// 定义 PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
// 辅助宏：限幅函数，防止 acos 输入因浮点误差略微超过 1.0 导致 NaN
#define LIMIT(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

int FiveBar_FwdKinematics(float phi1, float phi4, float* out_L0, float* out_phi0);
int FiveBar_InverseKinematics(float L0, float phi0, float* out_phi1, float* out_phi4);
int FiveBar_IK_Degree_Interface(float L0_mm, float phi0_deg, float* out_deg1, float* out_deg4);
int Left_FiveBar_IK_Degree_Interface(float L0_mm, float phi0_deg, float* out_deg1, float* out_deg4);
int Right_FiveBar_IK_Degree_Interface(float L0_mm, float phi0_deg, float* out_deg1, float* out_deg4);

#endif
