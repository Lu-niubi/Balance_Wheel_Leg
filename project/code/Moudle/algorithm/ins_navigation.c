/**
 * @file   ins_navigation.c
 * @brief  惯性导航 (INS) 航迹记录与回放算法实现
 *
 * 核心逻辑：
 *   Recording —— 每 1ms 由 RPM 计算本次行驶距离，累加到 accum_cm；
 *                当 accum_cm >= 2cm 时保存一个 {yaw_deg} 点，并将余量留给下次。
 *   Replay    —— O(1) 最近邻查找：idx = round(replay_dist / 2.0)，直接索引。
 *   航点检测  —— 比较 replay_distance_cm 与 wp_dist_cm[replay_wp_idx]。
 *   路径绘制  —— 迭代器按均匀 2cm 间距推算(x,y)：
 *                  x += 2cm * sin(yaw_rad)  (东向)
 *                  y += 2cm * cos(yaw_rad)  (北向)
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

// 连续yaw累积 (解决±180跳变)
static float s_yaw_continuous = 0.0f;  // 连续累积航向角 (度)
static float s_yaw_last       = 0.0f;  // 上次原始yaw，用于计算差分
static uint8_t s_yaw_inited   = 0;     // yaw 是否已初始化

// ─────────────────────────────────────────────────────────────────────────────
//  内部辅助
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  由左右轮 RPM 计算本采样周期内行驶的距离 (cm)
 */
static float calc_delta_distance_cm(int16_t left_rpm, int16_t right_rpm)
{
    float avg_rpm = ((float)left_rpm + (float)right_rpm) * 0.5f;
    if (avg_rpm < 0.0f) avg_rpm = -avg_rpm;

    // v(m/s) = rpm/60 * 2π * R → Δd(cm) = v * dt * 100
    float delta_cm = (avg_rpm / 60.0f) * (2.0f * PI * INS_WHEEL_RADIUS) * INS_SAMPLE_DT * 100.0f;
    return delta_cm;
}

/**
 * @brief  更新连续yaw（消除±180跳变），返回更新后的连续值
 */
static float update_yaw_continuous(float yaw_deg)
{
    if (!s_yaw_inited)
    {
        s_yaw_last       = yaw_deg;
        s_yaw_continuous = yaw_deg;
        s_yaw_inited     = 1;
    }
    else
    {
        float delta = yaw_deg - s_yaw_last;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        s_yaw_continuous += delta;
        s_yaw_last = yaw_deg;
    }
    return s_yaw_continuous;
}

// ─────────────────────────────────────────────────────────────────────────────
//  接口实现
// ─────────────────────────────────────────────────────────────────────────────

void INS_Init(void)
{
    memset(&g_ins, 0, sizeof(g_ins));
    s_yaw_continuous = 0.0f;
    s_yaw_last       = 0.0f;
    s_yaw_inited     = 0;
}

uint8_t INS_RecordTick(int16_t left_rpm, int16_t right_rpm, float yaw_deg)
{
    // 更新连续yaw（每次都要调用，保持差分连续）
    float yaw_cont = update_yaw_continuous(yaw_deg);

    // 计算本次距离
    float delta_cm = calc_delta_distance_cm(left_rpm, right_rpm);
    g_ins.total_distance_cm += delta_cm;
    g_ins.accum_cm          += delta_cm;

    // 每累积 INS_RECORD_SPACING_CM，保存一个点
    while (g_ins.accum_cm >= INS_RECORD_SPACING_CM)
    {
        if (g_ins.point_count >= INS_MAX_POINTS)
            return 1;  // 缓冲区已满

        g_ins.points[g_ins.point_count].yaw_deg = yaw_cont;
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
//  回放
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

    // 累积回放距离
    float delta_cm = calc_delta_distance_cm(left_rpm, right_rpm);
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
