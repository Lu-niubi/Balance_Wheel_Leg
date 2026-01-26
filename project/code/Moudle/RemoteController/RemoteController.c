#include "RemoteController.h"

// 接收状态机枚举
typedef enum
{
    RC_STATE_WAIT_HEADER,
    RC_STATE_RECEIVE_DATA
} rc_state_e;

// 缓冲区和状态变量
static rc_state_e parse_state = RC_STATE_WAIT_HEADER;
static uint8_t rx_buffer[15]; // 存放一帧数据
static uint8_t rx_index = 0;

// 初始化函数
void RemoteController_Init(void)
{
    // 1. 初始化串口2，波特率115200，引脚 P10.0 RX, P10.1 TX
    uart_init(RC_UART_INDEX, RC_UART_BAUDRATE, RC_UART_TX_PIN, RC_UART_RX_PIN);
    
    // 2. 开启接收中断 (逐飞库 uart_init 默认开启了中断，但需要去 isr 文件里关联回调)
    // 确保在 cm7_1_isr.c 的 uart2_isr 中调用 RemoteController_Callback
}

// 串口接收回调函数 (需要在中断里调用)
void RemoteController_Callback(void)
{
    uint8_t temp_byte;

    // 从硬件 FIFO 读取一个字节
    // 注意：zf_driver_uart.c 中的 uart_query_byte 实际上是查询模式，
    // 但在 ISR 中我们确定 FIFO 有数据，所以直接读。
    // 为了兼容性，我们直接调用库的读取函数，或者直接操作寄存器(如果库函数不适用ISR)
    // 假设库提供了 uart_read_byte 或者我们在中断里直接用 query
    if(uart_query_byte(RC_UART_INDEX, &temp_byte))
    {
        switch (parse_state)
        {
        case RC_STATE_WAIT_HEADER:
            if (temp_byte == 0xAA) // 匹配帧头 0xAA
            {
                rx_index = 0;
                rx_buffer[rx_index++] = temp_byte; // 存入帧头
                parse_state = RC_STATE_RECEIVE_DATA;
            }
            break;

        case RC_STATE_RECEIVE_DATA:
            rx_buffer[rx_index++] = temp_byte;

            // ESP32 发送包长度为 15 字节
            if (rx_index >= 15)
            {
                // === 数据包接收完整，开始解析 ===
                
                // [1-4] 按钮
                IPCS->M1_Pub.xbox_btn_y = rx_buffer[1];
                IPCS->M1_Pub.xbox_btn_x = rx_buffer[2];
                IPCS->M1_Pub.xbox_btn_b = rx_buffer[3];
                IPCS->M1_Pub.xbox_btn_a = rx_buffer[4];

                // [5-12] 摇杆 (大端拼接: High << 8 | Low)
                IPCS->M1_Pub.xbox_joy_l_hori = (int16_t)((rx_buffer[5] << 8) | rx_buffer[6]);
                IPCS->M1_Pub.xbox_joy_l_vert = (int16_t)((rx_buffer[7] << 8) | rx_buffer[8]);
                IPCS->M1_Pub.xbox_joy_r_hori = (int16_t)((rx_buffer[9] << 8) | rx_buffer[10]);
                IPCS->M1_Pub.xbox_joy_r_vert = (int16_t)((rx_buffer[11] << 8) | rx_buffer[12]);

                // [13-14] RT 扳机
                IPCS->M1_Pub.xbox_trig_rt    = (uint16_t)((rx_buffer[13] << 8) | rx_buffer[14]);
                
                // 更新标志，表示活着
                IPCS->M1_Pub.xbox_updated = !IPCS->M1_Pub.xbox_updated;

                // === 解析完成，重置状态 ===
                parse_state = RC_STATE_WAIT_HEADER;
                rx_index = 0;
            }
            break;
            
        default:
            parse_state = RC_STATE_WAIT_HEADER;
            break;
        }
    }
}