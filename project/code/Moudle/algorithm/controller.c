/**
 * @file controller.c
 * @author wanghongxi (Ported)
 * @brief  PID控制器定义 
 * @version V1.1.4
 * @date 2024-01-04
 */
#include "controller.h"

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
}

// 变速积分
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

// 积分限幅
static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    
    // 抗积分饱和 (Anti-windup): 如果总输出超限且积分仍在助推超限，则清零当前积分项
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    // 纯积分限幅
    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// ==========================================
// 微分滤波 (EMA 滤波)
// 参数意义：pid->Derivative_LPF_RC 代表旧数据的权重 (0.0 ~ 1.0)
// ==========================================
static void f_Derivative_Filter(PIDInstance *pid)
{
    // 提取旧数据权重，限制在 0~1 之间防止溢出暴走
    float alpha = pid->Derivative_LPF_RC;
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;

    // 新的Dout = (新计算的原始Dout * 新数据占比) + (上一次的Dout * 旧数据占比)
    pid->Dout = (pid->Dout * (1.0f - alpha)) + (pid->Last_Dout * alpha);
}

// ==========================================
// 输出滤波 (EMA 滤波)
// 参数意义：pid->Output_LPF_RC 代表旧数据的权重 (0.0 ~ 1.0)
// ==========================================
static void f_Output_Filter(PIDInstance *pid)
{
    // 提取旧数据权重，限制在 0~1 之间防止溢出暴走
    float alpha = pid->Output_LPF_RC;
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;

    // 新的Output = (新计算的原始Output * 新数据占比) + (上一次的Output * 旧数据占比)
    pid->Output = (pid->Output * (1.0f - alpha)) + (pid->Last_Output * alpha);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times (原文注释写1000但代码是500，保留原逻辑)
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // 清空PID结构体
    memset(pid, 0, sizeof(PIDInstance));
    
    // 赋值配置参数
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut;
    pid->DeadBand = config->DeadBand;
    pid->dt = config->dt; // 重要：从配置中获取固定的dt

    pid->Improve = config->Improve;
    pid->IntegralLimit = config->IntegralLimit;
    pid->CoefA = config->CoefA;
    pid->CoefB = config->CoefB;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
}

/**
 * @brief           PID计算
 * @param[in]       PID结构体
 * @param[in]       测量值
 * @param[in]       期望值
 * @retval          返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    // 移除 DWT 获取时间间隔，直接使用 pid->dt (固定周期)

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (abs(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                                 // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

/**
 * @brief           直立环专用PID计算 (使用外部陀螺仪角速度作为微分项)
 * @param[in]       pid      PID结构体
 * @param[in]       measure  测量值 (当前角度 Pitch)
 * @param[in]       ref      期望值 (期望角度 Target Pitch)
 * @param[in]       gyro     角速度 (Gyro Y, 单位: rad/s 或 LSB)
 * @retval          计算结果 (PWM/Current)
 */
float PIDCalculate_Balance(PIDInstance *pid, float measure, float ref, float gyro)
{
    // 堵转检测 (通常直立环不太需要，但保留以防万一)
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    // 1. 保存数据 & 计算误差
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 2. 如果在死区外, 则计算PID
    if (abs(pid->Err) > pid->DeadBand)
    {
        // --- P项: 角度误差产生的回复力 ---
        pid->Pout = pid->Kp * pid->Err;

        // --- I项: 角度积分 (保留原有优化逻辑) ---
        pid->ITerm = pid->Ki * pid->Err * pid->dt;

        // 梯形积分优化
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);

        // 变速积分优化
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        
        // --- D项: 核心修改点 ---
        // 使用外部传入的 gyro (角速度) 代替 (Err - Last_Err)/dt
        // 直立环中 D 项的作用是阻尼，力矩方向应与角速度方向相反
        // 假设 gyro 为正代表向前倒的速度，我们需要产生向后的力矩，所以是减号
        // 如果实际测试车子加速倒下，请将这里的减号改为加号
        pid->Dout = -pid->Kd * gyro; 
        
        // 微分滤波器 (对 Gyro 数据进行滤波)
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);

        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        // 3. 计算总输出
        pid->Iout += pid->ITerm;                  // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; 

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区
    {
        pid->Output = 0;
        pid->ITerm = 0;
        pid->Iout = 0;
    }

    // 4. 保存当前数据 (为滤波器等提供历史数据)
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}