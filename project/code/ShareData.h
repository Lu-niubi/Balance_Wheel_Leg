#ifndef __SHAREDDATA_H__   // 如果没定义过

#define __SHAREDDATA_H__   // 就定义它

#include <stdint.h>

typedef struct
{
    // ==========================================
    // 区域 A：M0 的地盘,控制部分 (M0 Write ONLY, M1 Read ONLY)
    // ==========================================
    struct
    {
        volatile float target_speed;    // M0 告诉 M1 要跑多快
        volatile int   run_mode;        // M0 告诉 M1现在的模式        
    } M0_Pub; // Pub = Publish (发布)

    // ==========================================
    // 区域 B：M1 的地盘,摄像头 (M1 Write ONLY, M0 Read ONLY)
    // ==========================================
    struct
    {
        volatile float error_angle;     // M1 告诉 M0 赛道偏差
        volatile uint8_t motor_ready;   // M1 告诉 M0 电机准备好了
    } M1_Pub;

} Shared_Data_t;

// 映射到绝对地址
#define SHARED_BASE 0x28001000
#define IPCS ((Shared_Data_t *)SHARED_BASE)

#endif
