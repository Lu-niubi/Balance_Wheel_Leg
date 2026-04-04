/**
 * @file   ins_navigation.h
 * @brief  惯性导航 (INS) 航迹记录与回放算法模块
 *
 * 功能：
 *   1. 取点 (Recording)：每 2ms 采样一次，记录累积行驶距离 (cm) 与航向角 yaw (rad)
 *   2. 回放 (Replay)  ：按照累积距离索引已记录的航向角，配合固定速度复刻路径
 *
 * 距离计算：
 *   avg_rpm / 60.0 * 2π * WHEEL_RADIUS * dt  → 本次行驶距离 (m)
 *   累加后转换为 cm 存储
 *
 * 调用时机：
 *   Recording —— 在 2ms 定时器中断中调用 INS_RecordTick()
 *   Replay   —— 在 2ms 定时器中断中调用 INS_ReplayTick()
 */

#ifndef _INS_NAVIGATION_H_
#define _INS_NAVIGATION_H_

#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
//  参数宏
// ─────────────────────────────────────────────────────────────────────────────

#define INS_MAX_POINTS      10000     // 最大记录点数 (10000 * 2ms = 20s)
#define INS_SAMPLE_DT       0.002f   // 采样周期 (秒) = 2ms
#define INS_WHEEL_RADIUS    0.034f   // 轮半径 (米)

#ifndef PI
#define PI 3.14159265f
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  数据结构
// ─────────────────────────────────────────────────────────────────────────────

/** 单个导航点 */
typedef struct
{
    float distance_cm;   // 从起点到此刻的累积行驶距离 (cm)
    float yaw_deg;       // 此刻的航向角 (度，来自 imu_sys.yaw)
} NavPoint_t;

/** INS 模块全局状态 */
typedef struct
{
    // --- 记录相关 ---
    NavPoint_t  points[INS_MAX_POINTS]; // 航迹点缓冲区
    uint16_t    point_count;            // 已记录的点数
    float       total_distance_cm;      // 累积行驶距离 (cm)

    // --- 回放相关 ---
    uint16_t    replay_index;           // 当前回放到第几个点
    float       replay_distance_cm;     // 回放过程中的累积距离 (cm)
    float       target_yaw_deg;         // 当前应该追踪的目标 yaw (度)
    uint8_t     replay_finished;        // 回放是否已完成
} INS_State_t;

// ─────────────────────────────────────────────────────────────────────────────
//  外部变量
// ─────────────────────────────────────────────────────────────────────────────

extern INS_State_t g_ins;

// ─────────────────────────────────────────────────────────────────────────────
//  接口函数
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  初始化 INS 模块，清空所有记录和状态
 */
void INS_Init(void);

/**
 * @brief  记录模式下的 2ms 定时调用
 * @param  left_rpm   左轮 RPM (int16_t，已归一化为正值前进)
 * @param  right_rpm  右轮 RPM
 * @param  yaw_deg    当前 IMU 航向角 (度)
 * @retval 0 = 正常记录, 1 = 缓冲区已满
 */
uint8_t INS_RecordTick(int16_t left_rpm, int16_t right_rpm, float yaw_deg);

/**
 * @brief  获取已记录的点数
 */
uint16_t INS_GetPointCount(void);

/**
 * @brief  获取已记录的总距离 (cm)
 */
float INS_GetTotalDistance(void);

/**
 * @brief  回放初始化（开始回放前调用一次）
 */
void INS_ReplayInit(void);

/**
 * @brief  回放模式下的 2ms 定时调用
 * @param  left_rpm   左轮 RPM
 * @param  right_rpm  右轮 RPM
 * @retval  当前应追踪的目标 yaw (度)。若回放结束返回最后一个点的 yaw
 *
 * @note   调用后可通过 g_ins.replay_finished 判断是否已到达终点
 */
float INS_ReplayTick(int16_t left_rpm, int16_t right_rpm);

/**
 * @brief  检查回放是否完成
 * @retval 1 = 已完成, 0 = 进行中
 */
uint8_t INS_IsReplayDone(void);

#endif /* _INS_NAVIGATION_H_ */
