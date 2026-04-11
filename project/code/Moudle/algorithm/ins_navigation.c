/**
 * @file   ins_navigation.c
 * @brief  惯性导航 (INS) 航迹记录与回放算法实现
 *
 * 核心逻辑：
 *   Recording —— 每 1ms 由 RPM 计算本次行驶距离，累加到 accum_cm；
 *                当 accum_cm >= 2cm 时保存一个 {yaw_deg} 点，并将余量留给下次。
 *   Replay    —— O(1) 最近邻查找：idx = round(replay_dist / 2.0)，直接索引。
 *   路径绘制  —— 迭代器按均匀 2cm 间距推算(x,y)：
 *                  x += 2cm * sin(yaw_rad)  (东向)
 *                  y += 2cm * cos(yaw_rad)  (北向)
 *
 * yaw_deg 存储绝对 IMU 航向角 (±180°)，每次归一化后直接保存，无连续累积。
 *
 * RPM → 距离公式：
 *   v(m/s) = rpm / 60.0 * 2π * R
 *   Δd(m)  = v * dt
 *   Δd(cm) = Δd(m) * 100
 */

#include "ins_navigation.h"
#include <math.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
//  全局变量
// ─────────────────────────────────────────────────────────────────────────────

INS_State_t g_ins;

// ─────────────────────────────────────────────────────────────────────────────
//  内部辅助
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  由左右轮 RPM 计算本采样周期内行驶的距离 (cm)
 */
static float calc_delta_distance_cm(int16_t left_rpm, int16_t right_rpm)
{
    float avg_rpm = ((float)left_rpm + (float)right_rpm) * 0.5f;
    // 保留符号：前进为正，后退为负
    // v(m/s) = rpm/60 * 2π * R → Δd(cm) = v * dt * 100
    float delta_cm = (avg_rpm / 60.0f) * (2.0f * PI * INS_WHEEL_RADIUS) * INS_SAMPLE_DT * 100.0f;
    return delta_cm;
}

// 仅用于回放距离累积（始终为正，不管方向）
static float calc_delta_abs_cm(int16_t left_rpm, int16_t right_rpm)
{
    float delta_cm = calc_delta_distance_cm(left_rpm, right_rpm);
    return (delta_cm < 0.0f) ? -delta_cm : delta_cm;
}

// ─────────────────────────────────────────────────────────────────────────────
//  接口实现
// ─────────────────────────────────────────────────────────────────────────────

void INS_Init(void)
{
    memset(&g_ins, 0, sizeof(g_ins));
}

uint8_t INS_RecordTick(int16_t left_rpm, int16_t right_rpm, float yaw_deg)
{
    // 计算本次距离
    float delta_cm = calc_delta_distance_cm(left_rpm, right_rpm);
    g_ins.total_distance_cm += delta_cm;
    g_ins.accum_cm          += delta_cm;

    // 每累积 INS_RECORD_SPACING_CM，保存一个点（存绝对 yaw，±180°）
    while (g_ins.accum_cm >= INS_RECORD_SPACING_CM)
    {
        if (g_ins.point_count >= INS_MAX_POINTS)
            return 1;  // 缓冲区已满

        g_ins.points[g_ins.point_count].yaw_deg = yaw_deg;  // 绝对 ±180°
        g_ins.point_count++;
        g_ins.accum_cm -= INS_RECORD_SPACING_CM;
    }

    return 0;
}

uint16_t INS_GetPointCount(void)
{
    return g_ins.point_count;
}

float INS_GetTotalDistance(void)
{
    return g_ins.total_distance_cm;
}

// ─────────────────────────────────────────────────────────────────────────────
//  回放（科目一用，全局路径）
// ─────────────────────────────────────────────────────────────────────────────

void INS_ReplayInit(void)
{
    g_ins.replay_distance_cm = 0.0f;
    g_ins.replay_finished    = 0;

    if (g_ins.point_count > 0)
        g_ins.target_yaw_deg = g_ins.points[0].yaw_deg;
    else
    {
        g_ins.target_yaw_deg  = 0.0f;
        g_ins.replay_finished = 1;
    }
}

float INS_ReplayTick(int16_t left_rpm, int16_t right_rpm)
{
    if (g_ins.replay_finished || g_ins.point_count == 0)
        return g_ins.target_yaw_deg;

    // 累积回放距离（取绝对值，回放阶段只前进）
    float delta_cm = calc_delta_abs_cm(left_rpm, right_rpm);
    g_ins.replay_distance_cm += delta_cm;

    // 录制路径的实际终点距离
    float end_dist = (float)(g_ins.point_count - 1) * INS_RECORD_SPACING_CM;

    // 终止判定：比实际录制路径多走 50cm，确保车在终点后才切COASTING
    float stop_dist = end_dist + 50.0f;

    // 终点前 30cm 冻结 yaw（避免录制停止时末尾噪声点引起顿挫）
    float freeze_dist = end_dist - 30.0f;
    if (freeze_dist < 0.0f) freeze_dist = 0.0f;

    if (g_ins.replay_distance_cm < freeze_dist)
    {
        // 正常区间：O(1) 最近邻查找
        uint16_t idx = (uint16_t)(g_ins.replay_distance_cm / INS_RECORD_SPACING_CM + 0.5f);
        if (idx >= g_ins.point_count) idx = g_ins.point_count - 1;
        g_ins.target_yaw_deg = g_ins.points[idx].yaw_deg;
    }
    // freeze_dist 之后不再更新 target_yaw_deg，保持最后稳定航向

    if (g_ins.replay_distance_cm >= stop_dist)
        g_ins.replay_finished = 1;

    return g_ins.target_yaw_deg;
}

uint8_t INS_IsReplayDone(void)
{
    return g_ins.replay_finished;
}

// ─────────────────────────────────────────────────────────────────────────────
//  分段回放（科目二用）
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  按段查询目标绝对 yaw（±180°）
 * @param  dist_cm    当前段已行驶距离 (cm，从段起点算起)
 * @param  start_idx  该段在 g_ins.points[] 中的起始索引
 * @param  count      该段包含的点数
 * @retval 目标绝对 yaw (±180°)
 */
float INS_GetSegmentYaw(float dist_cm, uint16_t start_idx, uint16_t count)
{
    if (count == 0)
    {
        // 该段无 INS 点，返回起始点的 yaw（兜底）
        if (start_idx < g_ins.point_count)
            return g_ins.points[start_idx].yaw_deg;
        return 0.0f;
    }

    float end_dist    = (float)(count - 1) * INS_RECORD_SPACING_CM;
    float freeze_dist = (end_dist > 30.0f) ? (end_dist - 30.0f) : 0.0f;
    float query_dist  = (dist_cm > freeze_dist) ? freeze_dist : dist_cm;

    uint16_t offset = (uint16_t)(query_dist / INS_RECORD_SPACING_CM + 0.5f);
    if (offset >= count) offset = count - 1;

    return g_ins.points[start_idx + offset].yaw_deg;
}

// ─────────────────────────────────────────────────────────────────────────────
//  路径绘制迭代器
// ─────────────────────────────────────────────────────────────────────────────

void INS_PathDrawInit(void)
{
    g_ins.draw_idx = 0;
    g_ins.draw_x   = 0.0f;
    g_ins.draw_y   = 0.0f;
}

uint8_t INS_PathDrawNext(float *x, float *y)
{
    if (g_ins.draw_idx >= g_ins.point_count)
        return 0;

    if (g_ins.draw_idx == 0)
    {
        // 起点：位移为零
        *x = 0.0f;
        *y = 0.0f;
    }
    else
    {
        // 每步对应 INS_RECORD_SPACING_CM = 2cm 的位移
        float yaw_rad = g_ins.points[g_ins.draw_idx].yaw_deg * (PI / 180.0f);
        g_ins.draw_x += INS_RECORD_SPACING_CM * sinf(yaw_rad);  // 东向 (cm)
        g_ins.draw_y += INS_RECORD_SPACING_CM * cosf(yaw_rad);  // 北向 (cm)
        *x = g_ins.draw_x;
        *y = g_ins.draw_y;
    }

    g_ins.draw_idx++;
    return 1;
}
