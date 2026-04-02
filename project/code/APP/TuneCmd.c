/**
 * @file   TuneCmd.c
 * @brief  通过 debug 串口（UART_0）动态调整 PID 参数
 *
 * 使用方式（串口助手，结尾加换行 \n）：
 *   p=1.52   -> ANG_KP
 *   i=0.02   -> ANG_KI
 *   d=0.18   -> ANG_KD
 *   g=240    -> GYRO_KP
 *   save     -> 打印当前参数
 */

#include "TuneCmd.h"
#include "Motor.h"
#include "controller.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define TUNE_BUF_LEN 64

static char    s_buf[TUNE_BUF_LEN];
static uint8_t s_len = 0;
static uint8_t s_cmd_ready = 0;

// ISR 里调用：喂一个字节
void TuneCmd_Feed(uint8_t byte)
{
    if (byte == '\r') return;
    if (byte == '\n')
    {
        if (s_len > 0) { s_buf[s_len] = '\0'; s_cmd_ready = 1; }
        return;
    }
    if (s_len < TUNE_BUF_LEN - 1) s_buf[s_len++] = (char)byte;
    else s_len = 0;
}

// 解析并执行单条命令 token（不含分号）
static void parse_one(char *cmd, PIDInstance *ap, PIDInstance *gp)
{
    float val = 0.0f;
    if (strcmp(cmd, "save") == 0)
    {
        printf("[Tune] ANG  Kp=%.4f Ki=%.4f Kd=%.4f\r\n", ANG_KP,  ANG_KI,  ANG_KD);
        printf("[Tune] GYRO Kp=%.2f Ki=%.4f Kd=%.4f\r\n", GYRO_KP, GYRO_KI, GYRO_KD);
    }
    else if (cmd[0]=='p' && cmd[1]=='=')
    {
        val = strtof(cmd+2, NULL);
        ANG_KP = val;  ap->Kp = val;
        printf("[Tune] ANG_KP=%.4f\r\n", val);
    }
    else if (cmd[0]=='i' && cmd[1]=='=')
    {
        val = strtof(cmd+2, NULL);
        ANG_KI = val;  ap->Ki = val;
        ap->ITerm = 0.0f;  ap->Iout = 0.0f;
        printf("[Tune] ANG_KI=%.4f (integral cleared)\r\n", val);
    }
    else if (cmd[0]=='d' && cmd[1]=='=')
    {
        val = strtof(cmd+2, NULL);
        ANG_KD = val;  ap->Kd = val;
        printf("[Tune] ANG_KD=%.4f\r\n", val);
    }
    else if (cmd[0]=='g' && cmd[1]=='=')
    {
        val = strtof(cmd+2, NULL);
        GYRO_KP = val;  gp->Kp = val;
        printf("[Tune] GYRO_KP=%.2f\r\n", val);
    }
    else if (cmd[0] != '\0')
    {
        printf("[Tune] ?\r\n");
    }
}

// 主循环里调用：解析并执行
void TuneCmd_Process(void)
{
    if (!s_cmd_ready) return;
    s_cmd_ready = 0;

    PIDInstance *ap = Motor_Get_Angle_PID();
    PIDInstance *gp = Motor_Get_Gyro_PID();

    // 按分号拆分，支持 "p=1.62;i=0.02;d=0.08;g=240"
    char *token = s_buf;
    char *p     = s_buf;
    while (1)
    {
        if (*p == ';' || *p == '\0')
        {
            char end = *p;
            *p = '\0';
            parse_one(token, ap, gp);
            if (end == '\0') break;
            token = p + 1;
        }
        p++;
    }

    s_len = 0;
}
