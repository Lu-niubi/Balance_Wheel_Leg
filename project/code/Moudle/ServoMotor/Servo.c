#include "Servo.h"

// -------------------------------------------------------------------------
// 内部宏定义：占空比计算公式
// -------------------------------------------------------------------------
// 原理说明：
// 50Hz 周期 = 20ms
// 0度   = 0.5ms 高电平
// 180度 = 2.5ms 高电平
// 公式推导：
// 高电平时间(ms) = 0.5 + (angle / 90.0)
// 占空比 = (高电平时间 / 周期时间) * PWM_DUTY_MAX
// -------------------------------------------------------------------------
#define CALC_SERVO_DUTY(angle)  ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + ((float)(angle) / 90.0)))

// /**
//  * @brief 初始化舵机模块
//  */
// void Servo_Init(void)
// {
//     // 检查频率安全性
// #if (SERVO_FREQ < 50 || SERVO_FREQ > 300)
//     #error "SERVO_FREQ Error! Must be between 50-300Hz"
// #endif

//     // 初始化各个舵机通道，初始占空比设为0 (或者设为中值)
//     // 300 是 PWM 分辨率参数，根据逐飞库的底层实现保留
//     pwm_init(SERVO_CH1_PIN, SERVO_FREQ, 0);
//     pwm_init(SERVO_CH2_PIN, SERVO_FREQ, 0);
//     pwm_init(SERVO_CH3_PIN, SERVO_FREQ, 0);
//     pwm_init(SERVO_CH4_PIN, SERVO_FREQ, 0);
// }
#define CALC_SERVO_DUTY(angle)  ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + ((float)(angle) / 90.0)))

// -------------------------------------------------------------------------
// 内部结构体：管理每个舵机的状态
// -------------------------------------------------------------------------
typedef struct {
    pwm_channel_enum pin;   // 对应的硬件引脚
    float current_angle;    // 当前所在的逻辑角度
    float target_angle;     // 想要去的目标角度
    float step_size;        // 步进速度
} Servo_Ctrl_t;

// 定义4个舵机的管理数组
static Servo_Ctrl_t Servo_List[SERVO_NUM_MAX];

/**
 * @brief 初始化舵机模块,也就是腿平的时候舵机角度为90度
 */
void Servo_Init(void)
{
    // 检查频率安全性
#if (SERVO_FREQ < 50 || SERVO_FREQ > 300)
    #error "SERVO_FREQ Error! Must be between 50-300Hz"
#endif

    // 1. 配置硬件引脚映射和初始状态
    // 左下
    Servo_List[SERVO_ID_1_LD].pin = SERVO_CH1_PIN;
    Servo_List[SERVO_ID_1_LD].current_angle = 90.0f; // 假设上电在中值
    Servo_List[SERVO_ID_1_LD].target_angle  = 90.0f;
    Servo_List[SERVO_ID_1_LD].step_size     = SERVO_DEFAULT_STEP;

    // 右上
    Servo_List[SERVO_ID_2_RU].pin = SERVO_CH2_PIN;
    Servo_List[SERVO_ID_2_RU].current_angle = 90.0f;
    Servo_List[SERVO_ID_2_RU].target_angle  = 90.0f;
    Servo_List[SERVO_ID_2_RU].step_size     = SERVO_DEFAULT_STEP;

    // 右下
    Servo_List[SERVO_ID_3_RD].pin = SERVO_CH3_PIN;
    Servo_List[SERVO_ID_3_RD].current_angle = 90.0f;
    Servo_List[SERVO_ID_3_RD].target_angle  = 90.0f;
    Servo_List[SERVO_ID_3_RD].step_size     = SERVO_DEFAULT_STEP;

    // 左上
    Servo_List[SERVO_ID_4_LU].pin = SERVO_CH4_PIN;
    Servo_List[SERVO_ID_4_LU].current_angle = 90.0f;
    Servo_List[SERVO_ID_4_LU].target_angle  = 90.0f;
    Servo_List[SERVO_ID_4_LU].step_size     = SERVO_DEFAULT_STEP;

    // 2. 硬件初始化 (初始给0或中值)
    for(int i = 0; i < SERVO_NUM_MAX; i++)
    {
        pwm_init(Servo_List[i].pin, SERVO_FREQ, 0);
        // 上电立即归位到初始角度
        Servo_SetAngle_Direct(Servo_List[i].pin, Servo_List[i].current_angle);
    }
}

/**
 * @brief 底层设置角度（直接设置PWM）
 */
void Servo_SetAngle_Direct(pwm_channel_enum pwm_ch, float angle)
{
    if(angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    else if(angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    uint32 duty = (uint32)CALC_SERVO_DUTY(angle);
    pwm_set_duty(pwm_ch, duty);
}
/**
 * @brief 设置舵机角度
 * @param pwm_ch 舵机PWM通道
 * @param angle  角度 0-180
 */
void Servo_SetAngle(pwm_channel_enum pwm_ch, float angle)
{
    // 1. 软件限位保护 (防止机械损坏)
    if(angle < SERVO_ANGLE_MIN)
    {
        angle = SERVO_ANGLE_MIN;
    }
    else if(angle > SERVO_ANGLE_MAX)
    {
        angle = SERVO_ANGLE_MAX;
    }

    // 2. 计算目标占空比
    uint32 duty = (uint32)CALC_SERVO_DUTY(angle);

    // 3. 设置PWM
    pwm_set_duty(pwm_ch, duty);
}

/**
 * @brief 设置目标角度（非阻塞）
 * 用户逻辑调用此函数，只是修改目标值，不会阻塞CPU
 */
void Servo_SetTarget(Servo_ID_Enum id, float angle)
{
    if(id >= SERVO_NUM_MAX) return;

    // 限位保护
    if(angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if(angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    Servo_List[id].target_angle = angle;
}

/**
 * @brief 舵机步进任务
 * 请放入 10ms 或 20ms 的定时器中断中调用
 */
void Servo_Task(void)
{
    for(int i = 0; i < SERVO_NUM_MAX; i++)
    {
        // 获取当前指针方便操作
        Servo_Ctrl_t *servo = &Servo_List[i];

        // 只有当当前角度 != 目标角度时才计算
        if(servo->current_angle != servo->target_angle)
        {
            // 计算偏差
            float error = servo->target_angle - servo->current_angle;
            
            // 如果偏差小于一步的距离，直接到位
            if(fabs(error) <= servo->step_size)
            {
                servo->current_angle = servo->target_angle;
            }
            else
            {
                // 否则向目标方向走一步
                if(error > 0)
                    servo->current_angle += servo->step_size;
                else
                    servo->current_angle -= servo->step_size;
            }

            // 更新物理硬件
            Servo_SetAngle_Direct(servo->pin, servo->current_angle);
        }
    }
}