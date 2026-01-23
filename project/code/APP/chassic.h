#ifndef _CHASSIC_H_
#define _CHASSIC_H_ 

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

#endif