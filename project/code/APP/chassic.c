/**
 * @file   chassic.c
 * @brief  科目1 — 惯性导航取点与回放应用层实现
 *
 * 科目1状态机:
 *   IDLE → RECORDING → READY → STABILIZING → REPLAYING → DONE
 *
 * Recording 阶段:
 *   - 关闭外部速度/航向控制 (ext_control = 0)
 *   - 电机速度目标 = 0，用手推着走
 *   - 每 1ms 调用 INS_RecordTick() 记录航迹
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
//  科目1 内部状态
// ─────────────────────────────────────────────────────────────────────────────

static chassic_state_t s_state = CHASSIC_IDLE;
static uint32_t s_stabilize_cnt = 0;   // STABILIZING 阶段计时 (1ms 计数)
static float s_yaw_replay_offset = 0.0f; // INS坐标系→Motor坐标系的yaw偏移
static float s_coast_accum_cm   = 0.0f; // COASTING 阶段累积距离 (cm)

// ─────────────────────────────────────────────────────────────────────────────
//  科目1 接口实现
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
    // printf("[Chassic] Recording started.\r\n");
}

void Chassic_StopRecord(void)
{
    if (s_state != CHASSIC_RECORDING)
        return;

    s_state = CHASSIC_READY;
    // printf("[Chassic] Recording stopped. Points:%d, Dist:%.1f cm\r\n",
        //    INS_GetPointCount(), INS_GetTotalDistance());
}

void Chassic_StartReplay(void)
{
    if (s_state != CHASSIC_READY)
        return;

    if (INS_GetPointCount() == 0)
    {
        // printf("[Chassic] No recorded points!\r\n");
        return;
    }

    // 准备回放
    INS_ReplayInit();

    // 重置PID状态
    Motor_Reset_State();

    // 计算坐标系偏移：将INS记录的连续yaw对齐到Motor的连续yaw
    s_yaw_replay_offset = Motor_Get_Yaw_Continuous() - g_ins.points[0].yaw_deg;

    // 进入稳定阶段：启用外部控制，速度=0，yaw=起始yaw (Motor坐标系)
    Motor_Set_Ext_Control(1);
    Motor_Set_Ext_Speed(0.0f);
    Motor_Set_Ext_Yaw(g_ins.points[0].yaw_deg + s_yaw_replay_offset);

    s_stabilize_cnt = 0;
    s_coast_accum_cm = 0.0f;
    s_state = CHASSIC_STABILIZING;
    // printf("[Chassic] Stabilizing... (%.1fs)\r\n",
    //        CHASSIC_STABILIZE_MS / 1000.0f);
}

void Chassic_Stop(void)
{
    Motor_Set_Ext_Control(0);
    Motor_Set_Ext_Speed(0.0f);
    Motor_Reset_State();
    s_state = CHASSIC_IDLE;
    // printf("[Chassic] Stopped.\r\n");
}

void Chassic_Tick_1ms(void)
{
    int16_t lrpm = Motor_Get_Left_Speed();
    int16_t rrpm = Motor_Get_Right_Speed();

    switch (s_state)
    {
        case CHASSIC_IDLE:
        case CHASSIC_READY:
        case CHASSIC_DONE:
            break;

        case CHASSIC_RECORDING:
        {
            uint8_t full = INS_RecordTick(lrpm, rrpm, imu_sys.yaw);
            if (full)
            {
                Chassic_StopRecord();
            }
            break;
        }

        case CHASSIC_STABILIZING:
        {
            s_stabilize_cnt++;
            if (s_stabilize_cnt >= CHASSIC_STABILIZE_MS)
            {
                Motor_Set_Ext_Speed(CHASSIC_REPLAY_SPEED);
                s_state = CHASSIC_REPLAYING;
                // printf("[Chassic] Replaying! Speed=%d RPM\r\n",
                    //    (int)CHASSIC_REPLAY_SPEED);
            }
            break;
        }

        case CHASSIC_REPLAYING:
        {
            float target_yaw_ins = INS_ReplayTick(lrpm, rrpm);
            Motor_Set_Ext_Yaw(target_yaw_ins + s_yaw_replay_offset);

            if (INS_IsReplayDone())
            {
                // 不立刻停车，进入滑行阶段：保持当前速度和yaw再走 CHASSIC_COAST_CM
                s_coast_accum_cm = 0.0f;
                s_state = CHASSIC_COASTING;
                // printf("[Chassic] Coasting %.0fcm...\r\n", CHASSIC_COAST_CM);
            }
            break;
        }

        case CHASSIC_COASTING:
        {
            // 用编码器速度累积滑行距离（与INS录制公式相同）
            float avg_rpm = ((float)lrpm + (float)rrpm) * 0.5f;
            if (avg_rpm < 0.0f) avg_rpm = -avg_rpm;
            float delta_cm = (avg_rpm / 60.0f) * (2.0f * 3.1415926f * INS_WHEEL_RADIUS)
                             * INS_SAMPLE_DT * 100.0f;
            s_coast_accum_cm += delta_cm;

            if (s_coast_accum_cm >= CHASSIC_COAST_CM)
            {
                // 到了10cm，立刻将速度目标设为0
                Motor_Set_Ext_Speed(0.0f);
                s_state = CHASSIC_DONE;
                // printf("[Chassic] Done! Coast=%.1fcm\r\n", s_coast_accum_cm);
            }
            // yaw 和 speed 维持回放结束时的值，不再更新
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
