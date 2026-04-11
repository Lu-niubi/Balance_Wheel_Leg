/**
 * @file   ins_navigation.h
 * @brief  惯性导航 (INS) 航迹记录与回放算法模块
 *
 * 功能：
 *   1. 取点 (Recording)：每 1ms 采样一次，每累积 2cm 保存一个航迹点
 *   2. 回放 (Replay)  ：O(1)最近邻查找，按累积距离索引对应航向角
 *   3. 分段回放        ：科目二专用，按段索引+段内距离查询绝对 yaw
 *   4. 路径绘制迭代器  ：回放完成后可迭代输出每个点的(x,y)坐标用于屏幕显示
 *
 * yaw_deg 含义：绝对 IMU 航向角 (±180°)，直接存储原始值，无连续累积。
 *
 * 距离计算：
 *   avg_rpm / 60.0 * 2π * WHEEL_RADIUS * dt  → 本次行驶距离 (m)
 *   累加后转换为 cm 存储
 *
 * 调用时机：
 *   Recording —— 在 1ms 定时器中断中调用 INS_RecordTick()
 *   Replay   —— 在 1ms 定时器中断中调用 INS_ReplayTick()
 */

#ifndef _INS_NAVIGATION_H_
#define _INS_NAVIGATION_H_

#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
//  参数宏
// ─────────────────────────────────────────────────────────────────────────────

#define INS_MAX_POINTS          5000     // 最大记录点数
#define INS_SAMPLE_DT           0.001f   // 采样周期 (秒) = 1ms
#define INS_WHEEL_RADIUS        0.034f   // 轮半径 (米)
#define INS_RECORD_SPACING_CM   2.0f     // 每 2cm 保存一个点
#ifndef PI
#define PI 3.14159265f
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  数据结构
// ─────────────────────────────────────────────────────────────────────────────

/** 单个导航点（均匀 2cm 间距） */
typedef struct
{
    float yaw_deg;       // 此刻的绝对航向角 (度，±180°，IMU raw)
} NavPoint_t;

/** INS 模块全局状态 */
typedef struct
{
    // --- 记录相关 ---
    NavPoint_t  points[INS_MAX_POINTS]; // 航迹点缓冲区 (均匀 2cm 间距)
    uint16_t    point_count;            // 已保存的点数
    float       total_distance_cm;      // 累积行驶总距离 (cm)
    float       accum_cm;               // 距离上次保存点的累积量 (未满2cm的余量)

    // --- 回放相关 (科目一全局路径) ---
    float       replay_distance_cm;     // 回放过程中的累积距离 (cm)
    float       target_yaw_deg;         // 当前应该追踪的目标 yaw (度，±180°)
    uint8_t     replay_finished;        // 回放是否已完成

    // --- 路径绘制迭代器 ---
    uint16_t    draw_idx;               // 当前迭代到的点索引
    float       draw_x;                 // 迭代累积的东向位移 (cm)
    float       draw_y;                 // 迭代累积的北向位移 (cm)
} INS_State_t;

// ─────────────────────────────────────────────────────────────────────────────
//  外部变量
// ─────────────────────────────────────────────────────────────────────────────

extern INS_State_t g_ins;

// ─────────────────────────────────────────────────────────────────────────────
//  接口函数
// ─────────────────────────────────────────────────────────────────────────────

/** 初始化 INS 模块，清空所有记录和状态 */
void INS_Init(void);

/**
 * @brief  记录模式下的 1ms 定时调用
 * @param  left_rpm   左轮 RPM
 * @param  right_rpm  右轮 RPM
 * @param  yaw_deg    当前 IMU 航向角 (度，±180°，直接存储)
 * @retval 0 = 正常记录, 1 = 缓冲区已满
 */
uint8_t INS_RecordTick(int16_t left_rpm, int16_t right_rpm, float yaw_deg);

/** 获取已保存的点数 */
uint16_t INS_GetPointCount(void);

/** 获取已记录的总距离 (cm) */
float INS_GetTotalDistance(void);

/** 回放初始化（开始回放前调用一次，科目一用） */
void INS_ReplayInit(void);

/**
 * @brief  回放模式下的 1ms 定时调用（科目一全局路径用）
 * @param  left_rpm   左轮 RPM
 * @param  right_rpm  右轮 RPM
 * @retval 当前应追踪的目标 yaw (度，±180°)。若回放结束返回最后冻结的 yaw
 */
float INS_ReplayTick(int16_t left_rpm, int16_t right_rpm);

/** 检查回放是否完成 */
uint8_t INS_IsReplayDone(void);

/**
 * @brief  按段查询目标绝对 yaw（科目二专用）
 * @param  dist_cm    当前段已行驶距离 (cm，从段起点算起)
 * @param  start_idx  该段在 g_ins.points[] 中的起始索引
 * @param  count      该段包含的点数（为0时返回 start_idx 处的 yaw）
 * @retval 目标绝对 yaw (±180°)，终点前 30cm 冻结防止末尾噪声
 */
float INS_GetSegmentYaw(float dist_cm, uint16_t start_idx, uint16_t count);

/**
 * @brief  初始化路径绘制迭代器（绘制完成后可再次调用重置）
 */
void INS_PathDrawInit(void);

/**
 * @brief  迭代获取下一个路径点的平面坐标
 * @param  x  [out] 东向位移 (cm)，以起点为原点
 * @param  y  [out] 北向位移 (cm)，以起点为原点
 * @retval 1 = 有数据, 0 = 迭代结束
 */
uint8_t INS_PathDrawNext(float *x, float *y);

#endif /* _INS_NAVIGATION_H_ */
