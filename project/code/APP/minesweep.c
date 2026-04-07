/**
 * @file   minesweep.c
 * @brief  科目2 — 定点排雷 (纯惯导，固定PWM差速旋转)
 *
 * 状态机:
 *   IDLE → RECORDING → READY → STABILIZING → DRIVING → SPINNING → SPIN_ALIGN → ... → COASTING → DONE
 *
 * 核心思想: 每段直线都是独立的"迷你科目1"
 *   - 录制: 按键标记雷区中心点，记录 IMU raw yaw + 累积距离
 *   - 回放: 直线段 yaw PID 跟踪 → 到达雷区点 → 固定PWM差速转720°
 *           → 重新读取IMU yaw映射offset → 下一段直线
 *   - 首尾点不旋转
 */

#include "minesweep.h"
#include "ins_navigation.h"
#include "Motor.h"
#include "IMU_Deal.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
//  内部数据结构
// ─────────────────────────────────────────────────────────────────────────────

typedef struct {
    float dist_cm;       // 此标记点在总路径上的累积距离 (cm)
    float yaw_deg;       // 此标记点处的 IMU raw yaw (±180°)
} ms_marker_t;

// ─────────────────────────────────────────────────────────────────────────────
//  内部状态
// ─────────────────────────────────────────────────────────────────────────────

static ms_state_t   s_state          = MS_IDLE;
static ms_marker_t  s_markers[MS_MAX_MARKERS];
static uint8_t      s_marker_count   = 0;
static uint8_t      s_cur_marker     = 0;       // 当前正在前往的 marker 索引
static float        s_yaw_offset     = 0.0f;    // INS yaw → Motor yaw 映射偏移
static float        s_drive_accum_cm = 0.0f;    // 当前直线段累积行驶距离
static float        s_spin_accum_deg = 0.0f;    // 旋转累积角度
static float        s_spin_last_yaw  = 0.0f;    // 旋转时上次 raw yaw (用于差分)
static uint32_t     s_align_cnt      = 0;        // SPIN_ALIGN 稳定计时 (ms)
static float        s_coast_accum_cm = 0.0f;    // COASTING 累积距离
static uint32_t     s_stabilize_cnt  = 0;        // STABILIZING 计时 (ms)

// ─────────────────────────────────────────────────────────────────────────────
//  辅助函数
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  计算两个 marker 之间的距离 (cm)
 */
static float marker_segment_dist(uint8_t from_idx, uint8_t to_idx)
{
    return s_markers[to_idx].dist_cm - s_markers[from_idx].dist_cm;
}

/**
 * @brief  由左右轮 RPM 计算本 1ms 周期内行驶的距离 (cm)
 */
static float calc_delta_cm(int16_t lrpm, int16_t rrpm)
{
    float avg_rpm = ((float)lrpm + (float)rrpm) * 0.5f;
    if (avg_rpm < 0.0f) avg_rpm = -avg_rpm;
    return (avg_rpm / 60.0f) * (2.0f * 3.1415926f * INS_WHEEL_RADIUS) * INS_SAMPLE_DT * 100.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  公共接口实现
// ─────────────────────────────────────────────────────────────────────────────

void Minesweep_Init(void)
{
    s_state        = MS_IDLE;
    s_marker_count = 0;
    s_cur_marker   = 0;
    s_yaw_offset   = 0.0f;
    Motor_Set_Ext_Control(0);
    Motor_Set_Spin(0);
}

void Minesweep_StartRecord(void)
{
    if (s_state != MS_IDLE && s_state != MS_DONE)
        return;

    INS_Init();
    Motor_Set_Ext_Control(0);
    Motor_Reset_State();

    s_marker_count = 0;
    s_state        = MS_RECORDING;
}

void Minesweep_StopRecord(void)
{
    if (s_state != MS_RECORDING)
        return;

    // 至少需要 2 个标记点 (起点 + 终点)
    if (s_marker_count < 2)
    {
        // 点数不够，回到 IDLE
        s_state = MS_IDLE;
        return;
    }

    s_state = MS_READY;
}

void Minesweep_MarkPoint(void)
{
    if (s_state != MS_RECORDING)
        return;
    if (s_marker_count >= MS_MAX_MARKERS)
        return;

    s_markers[s_marker_count].dist_cm = INS_GetTotalDistance();
    s_markers[s_marker_count].yaw_deg = imu_sys.yaw;  // raw yaw (±180°)
    s_marker_count++;
}

void Minesweep_StartReplay(void)
{
    if (s_state != MS_READY)
        return;
    if (s_marker_count < 2)
        return;

    INS_ReplayInit();
    Motor_Reset_State();

    // 计算坐标系偏移：将 marker[0] 的 raw yaw 对齐到 Motor 的连续 yaw
    s_yaw_offset = Motor_Get_Yaw_Continuous() - s_markers[0].yaw_deg;

    s_cur_marker     = 0;
    s_drive_accum_cm = 0.0f;
    s_stabilize_cnt  = 0;
    s_coast_accum_cm = 0.0f;

    // 进入稳定阶段
    Motor_Set_Ext_Control(1);
    Motor_Set_Ext_Speed(0.0f);
    Motor_Set_Ext_Yaw(s_markers[0].yaw_deg + s_yaw_offset);

    s_state = MS_STABILIZING;
}

void Minesweep_Stop(void)
{
    Motor_Set_Ext_Control(0);
    Motor_Set_Ext_Speed(0.0f);
    Motor_Set_Spin(0);
    Motor_Reset_State();
    s_state = MS_IDLE;
}

void Minesweep_Tick_1ms(void)
{
    int16_t lrpm = Motor_Get_Left_Speed();
    int16_t rrpm = Motor_Get_Right_Speed();

    switch (s_state)
    {
        case MS_IDLE:
        case MS_READY:
        case MS_DONE:
            break;

        case MS_RECORDING:
            // 录制期间持续累积距离，供 Minesweep_MarkPoint() 读取
            INS_RecordTick(lrpm, rrpm, imu_sys.yaw);
            break;

        // ─── STABILIZING: 等待 PID 平衡 ──────────────────────────
        case MS_STABILIZING:
        {
            s_stabilize_cnt++;
            if (s_stabilize_cnt >= MS_STABILIZE_MS)
            {
                // 稳定完毕，开始第一段直线行驶
                Motor_Set_Ext_Speed(MS_REPLAY_SPEED);
                s_drive_accum_cm = 0.0f;
                s_state = MS_DRIVING;
            }
            break;
        }

        // ─── DRIVING: 直线段 yaw PID 跟踪 ────────────────────────
        case MS_DRIVING:
        {
            // 累积当前直线段距离
            s_drive_accum_cm += calc_delta_cm(lrpm, rrpm);

            // 目标 yaw = 当前 marker 的 raw yaw + offset（和科目1一样）
            Motor_Set_Ext_Yaw(s_markers[s_cur_marker].yaw_deg + s_yaw_offset);

            // 计算到下一个 marker 的距离
            uint8_t next_idx = s_cur_marker + 1;
            if (next_idx >= s_marker_count)
            {
                // 不应发生，但保险起见
                s_coast_accum_cm = 0.0f;
                s_state = MS_COASTING;
                break;
            }

            float seg_dist = marker_segment_dist(s_cur_marker, next_idx);

            if (s_drive_accum_cm >= seg_dist)
            {
                // 到达下一个 marker
                s_cur_marker = next_idx;

                if (s_cur_marker + 1 >= s_marker_count)
                {
                    // 到达最后一个点 → 滑行
                    s_coast_accum_cm = 0.0f;
                    s_state = MS_COASTING;
                }
                else
                {
                    // 中间点 → 旋转
                    s_spin_accum_deg = 0.0f;
                    s_spin_last_yaw  = imu_sys.yaw;
                    Motor_Set_Spin(MS_SPIN_PWM);
                    Motor_Set_Ext_Speed(0.0f);
                    s_state = MS_SPINNING;
                }
            }
            break;
        }

        // ─── SPINNING: 固定 PWM 差速旋转 720° ────────────────────
        case MS_SPINNING:
        {
            // 累加 yaw 变化量 (处理 ±180° 跳变)
            float delta = imu_sys.yaw - s_spin_last_yaw;
            if (delta >  180.0f) delta -= 360.0f;
            if (delta < -180.0f) delta += 360.0f;
            s_spin_accum_deg += delta;
            s_spin_last_yaw = imu_sys.yaw;

            if (s_spin_accum_deg >= MS_SPIN_TOTAL_DEG)
            {
                // 旋转完成
                Motor_Set_Spin(0);
                Motor_Set_Ext_Speed(0.0f);
                s_align_cnt = 0;
                s_state = MS_SPIN_ALIGN;
            }
            break;
        }

        // ─── SPIN_ALIGN: 等待 IMU 稳定，重新映射 yaw offset ────
        case MS_SPIN_ALIGN:
        {
            s_align_cnt++;
            if (s_align_cnt >= MS_SPIN_ALIGN_MS)
            {
                // 重新计算 yaw offset
                // 下一段要追踪的是 markers[s_cur_marker] 的 yaw
                s_yaw_offset = Motor_Get_Yaw_Continuous() - s_markers[s_cur_marker].yaw_deg;

                // 锁定当前 yaw 目标，防止突然转动
                Motor_Set_Ext_Yaw(Motor_Get_Yaw_Continuous());

                // 开始下一段直线
                s_drive_accum_cm = 0.0f;
                Motor_Set_Ext_Speed(MS_REPLAY_SPEED);
                s_state = MS_DRIVING;
            }
            break;
        }

        // ─── COASTING: 最后滑行 ──────────────────────────────────
        case MS_COASTING:
        {
            s_coast_accum_cm += calc_delta_cm(lrpm, rrpm);

            if (s_coast_accum_cm >= MS_COAST_CM)
            {
                Motor_Set_Ext_Speed(0.0f);
                Motor_Set_Ext_Control(0);
                s_state = MS_DONE;
            }
            // yaw 和 speed 维持不变
            break;
        }
    }
}

ms_state_t Minesweep_GetState(void)
{
    return s_state;
}

uint8_t Minesweep_GetMarkerCount(void)
{
    return s_marker_count;
}

float Minesweep_GetRecordDistance(void)
{
    return INS_GetTotalDistance();
}

uint8_t Minesweep_GetReplayProgress(void)
{
    if (s_marker_count < 2) return 0;

    // 进度 = 当前 marker 索引 / (总marker数 - 1) * 100
    float pct = ((float)s_cur_marker / (float)(s_marker_count - 1)) * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)pct;
}
