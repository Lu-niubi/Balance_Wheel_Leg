/**
 * @file   ins_navigation.c
 * @brief  惯性导航 (INS) 航迹记录与回放算法实现
 *
 * 核心逻辑：
 *   Recording —— 每 2ms 由 RPM 计算本次行驶距离 (cm)，累加到 total_distance_cm，
 *                同时保存 {cumulative_distance_cm, yaw_deg} 到缓冲区。
 *   Replay   —— 每 2ms 计算回放过程中的累积距离，按距离查找对应的目标 yaw。
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
 * @param  left_rpm   左轮 RPM
 * @param  right_rpm  右轮 RPM
 * @return 距离 (cm)，始终 >= 0
 */
static float calc_delta_distance_cm(int16_t left_rpm, int16_t right_rpm)
{
    // 取左右轮平均 RPM
    float avg_rpm = ((float)left_rpm + (float)right_rpm) / 2.0f;

    // 取绝对值：推车可能RPM为负
    if (avg_rpm < 0.0f) avg_rpm = -avg_rpm;

    // rpm → m/s → m/sample → cm/sample
    // v = rpm / 60 * 2π * R
    // Δd = v * dt * 100 (转 cm)
    float delta_cm = (avg_rpm / 60.0f) * (2.0f * PI * INS_WHEEL_RADIUS) * INS_SAMPLE_DT * 100.0f;

    return delta_cm;
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
    if (g_ins.point_count >= INS_MAX_POINTS)
    {
        return 1; // 缓冲区已满
    }

    // 计算本次行驶距离并累加
    float delta = calc_delta_distance_cm(left_rpm, right_rpm);
    g_ins.total_distance_cm += delta;

    // 保存航迹点
    g_ins.points[g_ins.point_count].distance_cm = g_ins.total_distance_cm;
    g_ins.points[g_ins.point_count].yaw_deg     = yaw_deg;
    g_ins.point_count++;

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

void INS_ReplayInit(void)
{
    g_ins.replay_index       = 0;
    g_ins.replay_distance_cm = 0.0f;
    g_ins.replay_finished    = 0;

    // 初始目标 yaw 设为第一个记录点的 yaw
    if (g_ins.point_count > 0)
    {
        g_ins.target_yaw_deg = g_ins.points[0].yaw_deg;
    }
    else
    {
        g_ins.target_yaw_deg = 0.0f;
        g_ins.replay_finished = 1;
    }
}

float INS_ReplayTick(int16_t left_rpm, int16_t right_rpm)
{
    if (g_ins.replay_finished || g_ins.point_count == 0)
    {
        return g_ins.target_yaw_deg;
    }

    // 累积回放距离
    float delta = calc_delta_distance_cm(left_rpm, right_rpm);
    g_ins.replay_distance_cm += delta;

    // 按距离在记录数组中向前查找（单调递增，所以只需向前扫描）
    while (g_ins.replay_index < g_ins.point_count - 1)
    {
        if (g_ins.replay_distance_cm >= g_ins.points[g_ins.replay_index + 1].distance_cm)
        {
            g_ins.replay_index++;
        }
        else
        {
            break;
        }
    }

    // 更新目标 yaw
    g_ins.target_yaw_deg = g_ins.points[g_ins.replay_index].yaw_deg;

    // 检查是否到达终点
    float end_distance = g_ins.points[g_ins.point_count - 1].distance_cm;
    if (g_ins.replay_distance_cm >= end_distance)
    {
        g_ins.replay_finished = 1;
    }

    return g_ins.target_yaw_deg;
}

uint8_t INS_IsReplayDone(void)
{
    return g_ins.replay_finished;
}
