#include "Motor.h"
#include <string.h> // 用于 memset 等，如果底层没包含的话

//-------------------------------------------------------------------------------------------------------------------
//  内部静态函数
//-------------------------------------------------------------------------------------------------------------------

/**
 * @brief   数值限幅函数
 * @param   val 输入值
 * @param   min 最小值
 * @param   max 最大值
 * @return  限幅后的值
 */
static int16_t clip_value(int16_t val, int16_t min, int16_t max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

//-------------------------------------------------------------------------------------------------------------------
//  接口函数实现
//-------------------------------------------------------------------------------------------------------------------

// 1. 电机初始化
void Motor_Init(void)
{
    // 调用底层的串口和参数初始化
    small_driver_uart_init();
    
    // 初始化后，底层驱动会自动发送 small_driver_get_speed() 
    // 从而让驱动板开始周期性回传数据，无需在此重复调用
}

// 2. 设置电机占空比 (主要增加了安全限幅)
void Motor_Set_Duty(int16_t left_duty, int16_t right_duty)
{
    int16_t safe_left;
    int16_t safe_right;

    // 进行限幅处理，防止PID计算输出超过驱动板接收范围 (-10000 ~ 10000)
    safe_left  = clip_value(left_duty, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    safe_right = clip_value(right_duty, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);

    // 调用底层发送协议
    small_driver_set_duty(safe_left, safe_right);
}

// 3. 获取左电机速度
int16_t Motor_Get_Left_Speed(void)
{
    // 直接从底层暴露的结构体中读取解包后的数据
    return motor_value.receive_left_speed_data;
}

// 4. 获取右电机速度
int16_t Motor_Get_Right_Speed(void)
{
    return motor_value.receive_right_speed_data;
}