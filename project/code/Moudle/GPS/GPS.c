#include "GPS.h"

// GPS 模块封装
// 底层由 zf_device_gnss 驱动实现，包括 UART2 串口接收中断（cm7_0_isr.c 中 uart2_isr 已调用 gnss_uart_callback）
// 本模块只做初始化封装和数据解析触发

// GPS_Init: 初始化 TAU1201 GNSS 模块（UART2, 115200bps, 10Hz）
void GPS_Init(void)
{
    gnss_init(TAU1201);
}

// GPS_Update: 检查 gnss_flag，若有新数据则解析并返回 1，否则返回 0
// 调用方式：在主循环中轮询
uint8 GPS_Update(void)
{
    if(gnss_flag)
    {
        gnss_flag = 0;
        gnss_data_parse();
        return 1;
    }
    return 0;
}
