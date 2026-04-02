#include "small_driver_uart_control.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
small_device_value_struct motor_value;      // 通讯数据结构体

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback(1000, -1000);
// 备注信息     用于解析接收到的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;

    while(uart_query_byte(SMALL_DRIVER_UART, &receive_data))
    {
        if(receive_data == 0xA5)                                                            // 字节包帧头，切换到字节包接收模式
        {
            motor_value.receive_data_count = 0;
            memset(motor_value.receive_data_buffer, 0, 7);
            motor_value.str_rx_idx = 0;                                                     // 丢弃当前字符串缓冲区（两种帧不会同时来）
        }

        // ---- 字节包接收 ----
        if(receive_data == 0xA5 || motor_value.receive_data_buffer[0] == 0xA5)
        {
            motor_value.receive_data_buffer[motor_value.receive_data_count++] = receive_data;

            if(motor_value.receive_data_count >= 7)
            {
                motor_value.sum_check_data = 0;
                for(int i = 0; i < 6; i++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[6])
                {
                    if(motor_value.receive_data_buffer[1] == 0x02)                          // 速度功能字
                    {
                        motor_value.receive_left_speed_data  = (int16)(((int)motor_value.receive_data_buffer[2] << 8) | (int)motor_value.receive_data_buffer[3]);
                        motor_value.receive_right_speed_data = (int16)(((int)motor_value.receive_data_buffer[4] << 8) | (int)motor_value.receive_data_buffer[5]);
                    }
                }

                motor_value.receive_data_count = 0;
                memset(motor_value.receive_data_buffer, 0, 7);
            }
        }
        // ---- 字符串接收（编码器/角度回复） ----
        else
        {
            if(motor_value.str_rx_idx >= (uint8)(sizeof(motor_value.str_rx_buf) - 1))
            {
                motor_value.str_rx_idx = 0;                                                 // 防溢出复位
            }

            motor_value.str_rx_buf[motor_value.str_rx_idx++] = (char)receive_data;

            if(receive_data == '\n')                                                        // 收到换行，一帧完成
            {
                motor_value.str_rx_buf[motor_value.str_rx_idx] = '\0';

                char *comma = strchr(motor_value.str_rx_buf, ',');
                if(comma != NULL)
                {
                    if(strchr(motor_value.str_rx_buf, '.') != NULL)                         // 含小数点：GET-ANGLE 回复
                    {
                        motor_value.angle_left  = (float)atof(motor_value.str_rx_buf);
                        motor_value.angle_right = (float)atof(comma + 1);
                    }
                    else                                                                    // 纯整数：GET-ENCODER 回复
                    {
                        motor_value.encoder_left  = (int32)atoi(motor_value.str_rx_buf);
                        motor_value.encoder_right = (int32)atoi(comma + 1);
                    }
                }

                motor_value.str_rx_idx = 0;
                memset(motor_value.str_rx_buf, 0, sizeof(motor_value.str_rx_buf));
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置电机占空比
// 参数说明     left_duty       左侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 参数说明     right_duty      右侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(int16 left_duty, int16 right_duty)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X01;                                         // 配置功能字

    motor_value.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // 拆分 左侧占空比 的高八位

    motor_value.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // 拆分 左侧占空比 的低八位

    motor_value.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // 拆分 右侧占空比 的高八位

    motor_value.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // 拆分 右侧占空比 的低八位

    motor_value.send_data_buffer[6] = 0;                                            // 和校验清除

    for(int i = 0; i < 6; i ++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[i];         // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);                     // 发送设置占空比的 字节包 数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     仅需发送一次 驱动将周期发出速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 配置功能字

    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[3] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[4] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[5] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[6] = 0xA7;                                         // 配置校验位

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 7);                             // 清除缓冲区数据

    memset(motor_value.receive_data_buffer, 0, 7);                          // 清除缓冲区数据

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;

    motor_value.receive_right_speed_data    = 0;

    motor_value.receive_left_speed_data     = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体参数初始化

    small_driver_set_duty(0, 0);                                                                // 设置0占空比

    small_driver_get_speed(); 
    
    small_driver_cmd_get_encoder();
}
//-------------------------------------------------------------------------------------------------------------------
// 函数名称     小驱动 发送 GET-ENCODER 指令
// 备注信息     驱动收到后每控制周期持续回复编码器原始值，格式：左值,右值\r\n
//              调用 small_driver_cmd_stop_send() 可停止
//-------------------------------------------------------------------------------------------------------------------
void small_driver_cmd_get_encoder(void)
{
    uart_write_string(SMALL_DRIVER_UART, "GET-ENCODER\r\n");
}


//-------------------------------------------------------------------------------------------------------------------
// 函数名称     小驱动 发送 GET-ANGLE 指令
// 备注信息     驱动收到后每控制周期持续回复角度，格式：180.00,90.00\r\n（单位：度）
//              调用 small_driver_cmd_stop_send() 可停止
//-------------------------------------------------------------------------------------------------------------------
void small_driver_cmd_get_angle(void)
{
    uart_write_string(SMALL_DRIVER_UART, "GET-ANGLE\r\n");
}


//-------------------------------------------------------------------------------------------------------------------
// 函数名称     小驱动 发送 GET-SPEED 指令（字符串模式）
// 备注信息     驱动收到后每控制周期持续回复转速，格式：左速,右速\r\n
//              调用 small_driver_cmd_stop_send() 可停止
//-------------------------------------------------------------------------------------------------------------------
void small_driver_cmd_get_speed_str(void)
{
    uart_write_string(SMALL_DRIVER_UART, "GET-SPEED\r\n");
}


//-------------------------------------------------------------------------------------------------------------------
// 函数名称     小驱动 发送 STOP-SEND 指令
// 备注信息     停止驱动的持续字符串回复
//-------------------------------------------------------------------------------------------------------------------
void small_driver_cmd_stop_send(void)
{
    uart_write_string(SMALL_DRIVER_UART, "STOP-SEND\r\n");
}


//-------------------------------------------------------------------------------------------------------------------
// 函数名称     小驱动 发送 SET-ZERO 指令
// 备注信息     驱动执行编码器校零
//-------------------------------------------------------------------------------------------------------------------
void small_driver_cmd_set_zero(void)
{
    uart_write_string(SMALL_DRIVER_UART, "SET-ZERO\r\n");
}















