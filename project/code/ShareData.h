#ifndef __SHAREDDATA_H__
#define __SHAREDDATA_H__

#include <stdint.h>

typedef struct
{
    // ==========================================
    // 区域 A：M0 的地盘 (M0 Write ONLY, M1 Read ONLY)
    // ==========================================
    struct
    {
        volatile float target_speed;    
        volatile int   run_mode;            
    } M0_Pub;

    // ==========================================
    // 区域 B：M1 的地盘 (M1 Write ONLY, M0 Read ONLY)
    // ==========================================
    struct
    {
        volatile float error_angle;      // 赛道偏差
        volatile uint8_t motor_ready;    // 电机准备标志

        // -------- 新增：Xbox 手柄数据 --------
        volatile uint8_t  xbox_btn_y;
        volatile uint8_t  xbox_btn_x;
        volatile uint8_t  xbox_btn_b;
        volatile uint8_t  xbox_btn_a;
        
        // 摇杆数据 (使用 int16_t 方便后续计算)
        volatile int16_t xbox_joy_l_hori;
        volatile int16_t xbox_joy_l_vert;
        volatile int16_t xbox_joy_r_hori;
        volatile int16_t xbox_joy_r_vert;
        
        volatile uint16_t xbox_trig_rt; // 扳机
        volatile uint8_t  xbox_updated; // 数据更新标志位(可选，用于判断连接状态)
        // ------------------------------------

    } M1_Pub;

} Shared_Data_t;

// 映射到绝对地址
#define SHARED_BASE 0x28001000
#define IPCS ((Shared_Data_t *)SHARED_BASE)

#endif