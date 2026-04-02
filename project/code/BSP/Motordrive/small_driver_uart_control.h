#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_4        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX                         (UART4_TX_P14_1)

#define SMALL_DRIVER_TX                         (UART4_RX_P14_0)

typedef struct
{
    uint8 send_data_buffer[7];                  // 字节包发送缓冲区

    uint8 receive_data_buffer[7];               // 字节包接收缓冲区

    uint8 receive_data_count;                   // 字节包接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_left_speed_data;              // 接收到的左电机速度数据

    int16 receive_right_speed_data;             // 接收到的右电机速度数据

    // 字符串协议接收
    char  str_rx_buf[32];                       // 字符串接收缓冲区
    uint8 str_rx_idx;                           // 字符串接收索引

    // 字符串协议解析结果
    int32 encoder_left;                         // 左编码器原始值
    int32 encoder_right;                        // 右编码器原始值
    float angle_left;                           // 左电机角度（度）
    float angle_right;                          // 右电机角度（度）

}small_device_value_struct;

extern small_device_value_struct motor_value;


// 字节包协议
void uart_control_callback(void);                                   // 小驱动 串口接收回调函数（中断中调用）
void small_driver_set_duty(int16 left_duty, int16 right_duty);      // 小驱动 设置占空比
void small_driver_get_speed(void);                                  // 小驱动 请求速度信息（字节包）

// 字符串指令发送
void small_driver_cmd_get_encoder(void);                            // 发送 GET-ENCODER 指令（持续回复编码器原始值）
void small_driver_cmd_get_angle(void);                              // 发送 GET-ANGLE 指令（持续回复角度）
void small_driver_cmd_get_speed_str(void);                          // 发送 GET-SPEED 指令（字符串模式）
void small_driver_cmd_stop_send(void);                              // 发送 STOP-SEND 指令（停止持续回复）
void small_driver_cmd_set_zero(void);                               // 发送 SET-ZERO 指令（校零）

// 初始化
void small_driver_uart_init(void);                                  // 小驱动 通讯初始化

#endif
