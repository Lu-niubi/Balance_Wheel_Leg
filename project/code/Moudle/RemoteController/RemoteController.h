#ifndef __REMOTECONTROLLER_H__
#define __REMOTECONTROLLER_H__

#include "zf_common_headfile.h" // 包含逐飞库公共头文件
#include "ShareData.h"         // 包含共享内存定义

// 定义使用的串口
#define RC_UART_INDEX       UART_2
#define RC_UART_BAUDRATE    115200
#define RC_UART_RX_PIN      UART2_RX_P10_0
#define RC_UART_TX_PIN      UART2_TX_P10_1

// 函数声明
void RemoteController_Init(void);     // 初始化
void RemoteController_ReceiveByte(uint8_t temp_byte);

#endif
