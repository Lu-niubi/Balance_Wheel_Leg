/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Motor.h"
#include "IMU_Deal.h"
#include "ShareData.h"
#include "Servo.h"
#include "KSCal.h"


// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_init();
    printf("\r\nSystem Booting...\r\n");

    // -------------------------------------------------------------------------
    // 2. 硬件外设初始化
    // -------------------------------------------------------------------------
    float leg1 = 43.12f;
    float deg = 90;
    float rad = 0;
    int time = 0;
    float angle1 = 0;
    float angle4 = 0;
    // 2.2 舵机机驱动初始化
    Servo_Init();
    system_delay_ms(500);
    //中间高度初始化
    // 2.3 电机驱动初始化
    Motor_Init();
    Motor_Reset_State();
    printf("\r\nMotor Driver OK.");
    system_delay_ms(500);
    while(imu660ra_init())
    {
        printf("\r\nIMU660RA Init Error! Check Wiring.");
        system_delay_ms(500);
    }
    printf("\r\nIMU660RA Hardware OK.");
    system_delay_ms(500);
    // -------------------------------------------------------------------------
    // 3. 算法层初始化 (必须在硬件OK后)
    // -------------------------------------------------------------------------
    // 3.1 IMU 融合算法初始化 (会执行200次采样进行快速收敛，耗时约1秒)
    printf("\r\nCalibrating IMU... Keep Stationary.");
    IMU_Fusion_Init();
    printf("\r\nIMU Calibration Done.");
    // 3.2 重置 LQR和PID 控制器状态
    Motor_Reset_State();
    system_delay_ms(500);
    // -------------------------------------------------------------------------
    // 4. 开启控制中断 (最后一步)
    // -------------------------------------------------------------------------
    pit_ms_init(PIT_CH0,1);
    pit_ms_init(PIT_CH1,5);

    float angle = 90;
    float angle_step = 10.0f;
    FiveBar_IK_Degree_Interface(Leg,90,&angle1,&angle4);
    while(true)
    {
        SCB_InvalidateDCache_by_Addr((void *)&IPCS->M1_Pub, sizeof(IPCS->M1_Pub));
        //  system_delay_ms(100);
        // deg= deg+angle_step;
        // if (deg<=40 || deg>=140)
        // {
        //     angle_step  = -angle_step;
        // }
        
        // Left_FiveBar_IK_Degree_Interface(50,deg,&angle1,&angle4);
        // Right_FiveBar_IK_Degree_Interface(50,deg,&angle1,&angle4);
        // printf("%.2f,%.2f,%.4f\n",2.8,imu_sys.pitch,imu_sys.gx);
        printf("%f\n",outtest);
        // angle_step++;
        // if (angle_step>=1000)
        // {
        // angle_step = 0;
        //  printf("%f,%f,%f,%f\r\n",k_out[0],k_out[1],k_out[2],k_out[3]);
        // }
        
        // 为了串口调试看着舒服，用了多行打印
        // system_delay_ms(1000);
        // printf("----------- Xbox Status (%d) -----------\r\n", IPCS->M1_Pub.xbox_updated);
        
        // // 打印按键 (0 或 1)
        // printf("BTN: [Y:%d] [X:%d] [B:%d] [A:%d]\r\n", 
        //        IPCS->M1_Pub.xbox_btn_y,
        //        IPCS->M1_Pub.xbox_btn_x,
        //        IPCS->M1_Pub.xbox_btn_b,
        //        IPCS->M1_Pub.xbox_btn_a);

        // // 打印摇杆 (使用 %5d 固定宽度对齐)
        // // int16范围: -32768 到 32767
        // printf("JoyL: H:%5d V:%5d  |  JoyR: H:%5d V:%5d\r\n", 
        //        IPCS->M1_Pub.xbox_joy_l_hori,
        //        IPCS->M1_Pub.xbox_joy_l_vert,
        //        IPCS->M1_Pub.xbox_joy_r_hori,
        //        IPCS->M1_Pub.xbox_joy_r_vert);

        // // 打印扳机和电机标志
        // printf("TrigRT: %5d  |  MotorReady: %d\r\n", 
        //        IPCS->M1_Pub.xbox_trig_rt,
        //        IPCS->M1_Pub.motor_ready);
               
        // printf("\r\n"); // 空行分隔

        // printf("%f,%f,%f,%d\r\n",imu_sys.pitch,3.5,imu_sys.gx,motor_duty);
        // printf("%f,%f,\r\n",leg,rad);
        // printf("%f,%f,%f\r\n",angle,angle1,angle4);
    }
        
}

// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看逐飞助手上位机打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 英飞凌TriCore 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看逐飞助手上位机设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：无刷电机无反应
//      检查Rx信号引脚是否接对，信号线是否松动
// 问题4：无刷电机转动但转速显示无速度
//      检查Tx信号引脚是否接对，信号线是否松动