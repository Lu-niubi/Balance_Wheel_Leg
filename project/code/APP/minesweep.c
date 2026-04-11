/**
 * @file   minesweep.c
 * @brief  科目2 — 定点排雷 (INS录制与科目1相同，自动检测旋转方向)
 *
 * 核心思想：
 *   录制 —— 与科目1完全一致，Tick_1ms 每毫秒调用 INS_RecordTick，
 *            MarkRotation() 仅记录当前总距离作为"旋转触发点"。
 *   回放 —— 纯 INS_ReplayTick 驱动航向，replay_distance_cm 到达旋转点时
 *            自动计算旋转方向（前后INS点航向差）和目标yaw，
 *            用斜坡生成器平滑推进 Motor Ext_Yaw，避免阶跃导致微分爆炸。
 *   旋转完成 —— 实际 yaw 在目标 ±2° 内即恢复 DRIVING，INS_ReplayTick 继续。
 */

#include "minesweep.h"
#include "ins_navigation.h"
#include "Motor.h"
#include "IMU_Deal.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
//  内部状态
// ─────────────────────────────────────────────────────────────────────────────

static ms_state_t  s_state           = MS_IDLE;
static ms_marker_t s_markers[MS_MAX_MARKERS];
static uint8     s_marker_count    = 0;
static uint8     s_cur_marker      = 0;     // 下一个待检测的旋转点索引
static float       s_coast_accum_cm  = 0.0f;
static uint32    s_stabilize_cnt   = 0;

// SPIN_ALIGN 专用
static float       s_heading_target  = 0.0f;  // Motor 连续 yaw 的旋转终点

// ─────────────────────────────────────────────────────────────────────────────
//  内部辅助
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  将绝对目标 yaw (±180°) 归一化后写入 Motor_Set_Ext_Yaw
 *         每 tick 调用，Motor PID 误差始终在 ±180° 内，无需全局 offset
 */
static void set_motor_yaw_abs(float target_abs_yaw)
{
    float abs_err = target_abs_yaw - imu_sys.yaw;
    if (abs_err >  180.0f) abs_err -= 360.0f;
    if (abs_err < -180.0f) abs_err += 360.0f;
    Motor_Set_Ext_Yaw(Motor_Get_Yaw_Continuous() + abs_err);
}

/**
 * @brief  在旋转点处分析 INS 点判断旋转方向，并计算 Motor 连续 yaw 目标
 *
 * @param  spin_dist_cm  旋转点的总距离 (cm)
 * @return               Motor 连续 yaw 目标（直接赋给 s_heading_target）
 *
 * 方向判断：取旋转点前 MS_SPIN_LOOK_BEFORE 个点和后 MS_SPIN_LOOK_AHEAD 个点
 *   δ = normalize(yaw_after - yaw_before, ±180°)
 *   δ > 0 → 航向增大 → CW → Motor 连续 yaw 正增量
 *   δ < 0 → 航向减小 → CCW → Motor 连续 yaw 负增量
 *
 * 旋转量：按判断方向归一化到 [0°, 360°)，保证只朝一个方向转到目标
 */
static float calc_spin_target(float spin_dist_cm)
{
    uint16 pc = INS_GetPointCount();
    if (pc < 2)
        return Motor_Get_Yaw_Continuous();  // 无INS数据，原地不动

    uint16 idx_at = (uint16)(spin_dist_cm / INS_RECORD_SPACING_CM);
    if (idx_at >= pc) idx_at = pc - 1;

    // 前后各取若干点（加边界保护）
    uint16 idx_b = (idx_at >= MS_SPIN_LOOK_BEFORE)
                     ? (idx_at - MS_SPIN_LOOK_BEFORE) : 0;
    uint16 idx_a = idx_at + MS_SPIN_LOOK_AHEAD;
    if (idx_a >= pc) idx_a = pc - 1;

    float yaw_b = g_ins.points[idx_b].yaw_deg;  // 旋转点前的航向
    float yaw_a = g_ins.points[idx_a].yaw_deg;  // 旋转点后的目标航向

    // ±180° 归一化得到最短路径方向
    float delta = yaw_a - yaw_b;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    float cur_abs   = imu_sys.yaw;
    float cur_motor = Motor_Get_Yaw_Continuous();

    // 按录制方向（delta 正负）归一化旋转弧长，强制单向
    // 先将对齐误差归一化到单圈范围，再叠加规则要求的 720°（两整圈）
    float err = yaw_a - cur_abs;
    if (delta >= 0.0f)
    {
        // CW方向：对齐量归一化到 [0°, 360°)
        if (err < 0.0f) err += 360.0f;
        if (err >= 360.0f) err -= 360.0f;

        // RMUL 规则：雷区内必须旋转至少两整圈（720°）
        err += 720.0f;
    }
    else
    {
        // CCW方向：对齐量归一化到 (-360°, 0°]
        if (err > 0.0f) err -= 360.0f;
        if (err <= -360.0f) err += 360.0f;

        // RMUL 规则：逆时针同样叠加两整圈（负方向）
        err -= 720.0f;
    }

    // err 现在至少 ±720°，无需极小值保护
    return cur_motor + err;
}

// ─────────────────────────────────────────────────────────────────────────────
//  公共接口实现
// ─────────────────────────────────────────────────────────────────────────────

void Minesweep_Init(void)
{
    s_state        = MS_IDLE;
    s_marker_count = 0;
    s_cur_marker   = 0;
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

    if (INS_GetPointCount() < 2)
    {
        s_state = MS_IDLE;
        return;
    }

    s_state = MS_READY;
}

void Minesweep_MarkRotation(void)
{
    if (s_state != MS_RECORDING)
        return;
    if (s_marker_count >= MS_MAX_MARKERS)
        return;

    s_markers[s_marker_count].dist_cm = INS_GetTotalDistance();
    s_marker_count++;
}

void Minesweep_StartReplay(void)
{
    if (s_state != MS_READY)
        return;
    if (INS_GetPointCount() < 2)
        return;

    Motor_Reset_State();
    INS_ReplayInit();

    s_cur_marker     = 0;
    s_coast_accum_cm = 0.0f;
    s_stabilize_cnt  = 0;

    // STABILIZING 期间预对准第一段航向
    float first_yaw = g_ins.points[0].yaw_deg;
    Motor_Set_Ext_Control(1);
    Motor_Set_Ext_Speed(0.0f);
    set_motor_yaw_abs(first_yaw);

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

// ─────────────────────────────────────────────────────────────────────────────
//  1ms 状态机
// ─────────────────────────────────────────────────────────────────────────────

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

        // ─── RECORDING: 与科目1完全相同，每 tick 记录 INS ────────────────
        case MS_RECORDING:
        {
            INS_RecordTick(lrpm, rrpm, imu_sys.yaw);
            break;
        }

        // ─── STABILIZING: PID 平衡，预对准第一段航向 ─────────────────────
        case MS_STABILIZING:
        {
            float first_yaw = g_ins.points[0].yaw_deg;
            set_motor_yaw_abs(first_yaw);

            s_stabilize_cnt++;
            if (s_stabilize_cnt >= MS_STABILIZE_MS)
            {
                Motor_Set_Ext_Speed(MS_REPLAY_SPEED);
                s_state = MS_DRIVING;
            }
            break;
        }

        // ─── DRIVING: 纯 INS_ReplayTick 跟踪，按距离触发旋转点 ──────────
        case MS_DRIVING:
        {
            float target_abs = INS_ReplayTick(lrpm, rrpm);
            set_motor_yaw_abs(target_abs);

            // 检查是否到达下一个旋转标记点
            if (s_cur_marker < s_marker_count &&
                g_ins.replay_distance_cm >= s_markers[s_cur_marker].dist_cm)
            {
                float spin_dist = s_markers[s_cur_marker].dist_cm;
                s_cur_marker++;

                // 停车，计算旋转目标，进入斜坡PID旋转
                Motor_Set_Ext_Speed(0.0f);
                s_heading_target = calc_spin_target(spin_dist);
                // 注意：不立即写 Motor_Set_Ext_Yaw，由斜坡生成器从当前位置推进
                s_state = MS_SPIN_ALIGN;
                break;
            }

            // INS 回放结束 → 滑行
            if (INS_IsReplayDone())
            {
                Motor_Set_Ext_Speed(0.0f);
                s_coast_accum_cm = 0.0f;
                s_state = MS_COASTING;
            }
            break;
        }

        // ─── SPIN_ALIGN: 斜坡生成器 + 闭环 PID 旋转对齐 ─────────────────
        //
        //  每 tick 最多推进 MS_SPIN_STEP_DEG_PER_MS 度，
        //  Motor PID 误差始终 ≤ 0.072°，彻底消除阶跃输入引起的微分爆炸
        //
        case MS_SPIN_ALIGN:
        {
            float step_max      = MS_SPIN_STEP_DEG_PER_MS;
            float cur_ext_yaw   = Motor_Get_Ext_Yaw();
            float err_to_target = s_heading_target - cur_ext_yaw;

            if      (err_to_target >  step_max) cur_ext_yaw += step_max;
            else if (err_to_target < -step_max) cur_ext_yaw -= step_max;
            else                                cur_ext_yaw  = s_heading_target;

            Motor_Set_Ext_Yaw(cur_ext_yaw);

            // 完成判定：斜坡走完 AND 实际连续 yaw 跟随误差 < 2°
            float cur_cont = Motor_Get_Yaw_Continuous();
            if ((cur_ext_yaw == s_heading_target) &&
                (fabsf(cur_cont - s_heading_target) <= 2.0f))
            {
                // 锁定当前 yaw，防 PID 冲击，恢复行驶
                Motor_Set_Ext_Yaw(cur_cont);
                Motor_Set_Ext_Speed(MS_REPLAY_SPEED);
                s_state = MS_DRIVING;
            }
            break;
        }

        // ─── COASTING: 终点滑行 ──────────────────────────────────────────
        case MS_COASTING:
        {
            float avg_rpm = ((float)lrpm + (float)rrpm) * 0.5f;
            float delta_cm = fabsf((avg_rpm / 60.0f)
                             * (2.0f * 3.1415926f * INS_WHEEL_RADIUS)
                             * INS_SAMPLE_DT * 100.0f);
            s_coast_accum_cm += delta_cm;

            if (s_coast_accum_cm >= MS_COAST_CM)
            {
                Motor_Set_Ext_Speed(0.0f);
                Motor_Set_Ext_Control(0);
                s_state = MS_DONE;
            }
            break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  查询接口
// ─────────────────────────────────────────────────────────────────────────────

ms_state_t Minesweep_GetState(void)
{
    return s_state;
}

uint8 Minesweep_GetMarkerCount(void)
{
    return s_marker_count;
}

float Minesweep_GetRecordDistance(void)
{
    return INS_GetTotalDistance();
}

uint8 Minesweep_GetReplayProgress(void)
{
    if (g_ins.point_count == 0) return 0;

    float total_cm = (float)(g_ins.point_count - 1) * INS_RECORD_SPACING_CM;
    if (total_cm < 1.0f) return 0;

    float pct = (g_ins.replay_distance_cm / total_cm) * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8)pct;
}
