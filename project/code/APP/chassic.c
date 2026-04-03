/**
 * @file   chassic.c
 * @brief  科目1 — 惯性导航取点与回放应用层实现
 *
 * 状态机流程:
 *   IDLE ──StartRecord()──> RECORDING ──StopRecord()──> READY
 *   READY ──StartReplay()──> STABILIZING ──(2s后)──> REPLAYING ──(到终点)──> DONE
 *
 * Recording 阶段:
 *   - 关闭外部速度/航向控制 (ext_control = 0)
 *   - 电机速度目标 = 0，用手推着走
 *   - 每 2ms 调用 INS_RecordTick() 记录 {距离, yaw}
 *
 * Replay 阶段:
 *   - 先 STABILIZING: ext_control=1, speed=0, yaw=初始yaw，等 PID 平衡
 *   - 后 REPLAYING:   speed=CHASSIC_REPLAY_SPEED, yaw=INS回放查表值
 */

#include "chassic.h"
#include "ins_navigation.h"
#include "Motor.h"
#include "IMU_Deal.h"
#include <math.h>
#include <stdio.h>

// ─────────────────────────────────────────────────────────────────────────────
//  内部状态
// ─────────────────────────────────────────────────────────────────────────────

static chassic_state_t s_state = CHASSIC_IDLE;
static uint32_t s_stabilize_cnt = 0;   // STABILIZING 阶段计时 (2ms 计数)
static float s_yaw_replay_offset = 0.0f; // INS坐标系→Motor坐标系的yaw偏移

// ─────────────────────────────────────────────────────────────────────────────
//  接口实现
// ─────────────────────────────────────────────────────────────────────────────

void Chassic_Init(void)
{
    INS_Init();
    s_state = CHASSIC_IDLE;
    s_stabilize_cnt = 0;

    // 确保外部控制关闭
    Motor_Set_Ext_Control(0);
}

void Chassic_StartRecord(void)
{
    if (s_state != CHASSIC_IDLE && s_state != CHASSIC_DONE)
        return;

    // 重新初始化INS
    INS_Init();

    // 关闭外部控制 (手推车，PID仍维持平衡但速度目标=0)
    Motor_Set_Ext_Control(0);

    // 重置 PID 状态
    Motor_Reset_State();

    s_state = CHASSIC_RECORDING;
    printf("[Chassic] Recording started.\r\n");
}

void Chassic_StopRecord(void)
{
    if (s_state != CHASSIC_RECORDING)
        return;

    s_state = CHASSIC_READY;
    printf("[Chassic] Recording stopped. Points:%d, Dist:%.1f cm\r\n",
           INS_GetPointCount(), INS_GetTotalDistance());
}

void Chassic_StartReplay(void)
{
    if (s_state != CHASSIC_READY)
        return;

    if (INS_GetPointCount() == 0)
    {
        printf("[Chassic] No recorded points!\r\n");
        return;
    }

    // 准备回放
    INS_ReplayInit();

    // 重置PID状态
    Motor_Reset_State();

    // 计算坐标系偏移：将INS记录的连续yaw对齐到Motor的连续yaw
    // Motor坐标系从开机时开始累积，INS坐标系从recording开始时的raw yaw开始
    s_yaw_replay_offset = Motor_Get_Yaw_Continuous() - g_ins.points[0].yaw_deg;

    // 进入稳定阶段：启用外部控制，速度=0，yaw=起始yaw (Motor坐标系)
    Motor_Set_Ext_Control(1);
    Motor_Set_Ext_Speed(0.0f);
    Motor_Set_Ext_Yaw(g_ins.points[0].yaw_deg + s_yaw_replay_offset);

    s_stabilize_cnt = 0;
    s_state = CHASSIC_STABILIZING;
    printf("[Chassic] Stabilizing... (%.1fs)\r\n",
           CHASSIC_STABILIZE_MS / 1000.0f);
}

void Chassic_Stop(void)
{
    Motor_Set_Ext_Control(0);
    Motor_Set_Ext_Speed(0.0f);
    Motor_Reset_State();
    s_state = CHASSIC_IDLE;
    printf("[Chassic] Stopped.\r\n");
}

void Chassic_Tick_2ms(void)
{
    int16_t lrpm = Motor_Get_Left_Speed();
    int16_t rrpm = Motor_Get_Right_Speed();

    switch (s_state)
    {
        case CHASSIC_IDLE:
        case CHASSIC_READY:
        case CHASSIC_DONE:
            // 什么都不做
            break;

        case CHASSIC_RECORDING:
        {
            // 每 2ms 记录一个点
            uint8_t full = INS_RecordTick(lrpm, rrpm, imu_sys.yaw);
            if (full)
            {
                // 缓冲区满，自动停止
                Chassic_StopRecord();
            }
            break;
        }

        case CHASSIC_STABILIZING:
        {
            s_stabilize_cnt++;
            // 等待 CHASSIC_STABILIZE_MS 毫秒
            // 计数单位是 2ms，所以 cnt 阈值 = ms / 2
            if (s_stabilize_cnt >= (CHASSIC_STABILIZE_MS / 2))
            {
                // 平稳期结束，开始回放
                Motor_Set_Ext_Speed(CHASSIC_REPLAY_SPEED);
                s_state = CHASSIC_REPLAYING;
                printf("[Chassic] Replaying! Speed=%d RPM\r\n",
                       (int)CHASSIC_REPLAY_SPEED);
            }
            break;
        }

        case CHASSIC_REPLAYING:
        {
            // 每 2ms 推进回放，获取目标 yaw (INS坐标系，连续值)
            float target_yaw_ins = INS_ReplayTick(lrpm, rrpm);
            // 转换到Motor坐标系后送给转向PID
            Motor_Set_Ext_Yaw(target_yaw_ins + s_yaw_replay_offset);

            // 检查回放是否完成
            if (INS_IsReplayDone())
            {
                // 到达终点，停车
                Motor_Set_Ext_Speed(0.0f);
                s_state = CHASSIC_DONE;
                printf("[Chassic] Replay done!\r\n");
            }
            break;
        }
    }
}

chassic_state_t Chassic_GetState(void)
{
    return s_state;
}

uint16_t Chassic_GetRecordCount(void)
{
    return INS_GetPointCount();
}

float Chassic_GetRecordDistance(void)
{
    return INS_GetTotalDistance();
}

uint8_t Chassic_GetReplayProgress(void)
{
    if (INS_GetPointCount() == 0) return 0;

    float total = INS_GetTotalDistance();
    if (total <= 0.0f) return 0;

    float current = g_ins.replay_distance_cm;
    float pct = (current / total) * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)pct;
}
