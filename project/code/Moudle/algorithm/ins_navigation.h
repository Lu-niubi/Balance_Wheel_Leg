/**
 * @file   ins_navigation.h
 * @brief  惯性导航 (INS) 航迹记录与回放算法模块
 *
 * 功能：
 *   1. 取点 (Recording)：每 1ms 采样一次，每累积 2cm 保存一个航迹点
 *   2. 回放 (Replay)  ：O(1)最近邻查找，按累积距离索引对应航向角
 *   3. 航点标记        ：取点时按键标记当前距离为"到达点"，回放时检测触发旋转
 *   4. 路径绘制迭代器  ：回放完成后可迭代输出每个点的(x,y)坐标用于屏幕显示
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
    float yaw_deg;       // 此刻的航向角 (度，连续累积，无±180跳变)
} NavPoint_t;

/** INS 模块全局状态 */
typedef struct
{
    // --- 记录相关 ---
    NavPoint_t  points[INS_MAX_POINTS]; // 航迹点缓冲区 (均匀 2cm 间距)
    uint16_t    point_count;            // 已保存的点数
    float       total_distance_cm;      // 累积行驶总距离 (cm)
    float       accum_cm;               // 距离上次保存点的累积量 (未满2cm的余量)

    // --- 回放相关 ---
    float       replay_distance_cm;     // 回放过程中的累积距离 (cm)
    float       target_yaw_deg;         // 当前应该追踪的目标 yaw (度)
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
 * @param  yaw_deg    当前 IMU 航向角 (度，原始 ±180 范围)
 * @retval 0 = 正常记录, 1 = 缓冲区已满
 */
uint8_t INS_RecordTick(int16_t left_rpm, int16_t right_rpm, float yaw_deg);

/** 获取已保存的点数 */
uint16_t INS_GetPointCount(void);

/** 获取已记录的总距离 (cm) */
float INS_GetTotalDistance(void);

/** 回放初始化（开始回放前调用一次） */
void INS_ReplayInit(void);

/**
 * @brief  回放模式下的 1ms 定时调用
 * @param  left_rpm   左轮 RPM
 * @param  right_rpm  右轮 RPM
 * @retval 当前应追踪的目标 yaw (度)。若回放结束返回最后一个点的 yaw
 */
float INS_ReplayTick(int16_t left_rpm, int16_t right_rpm);

/** 检查回放是否完成 */
uint8_t INS_IsReplayDone(void);

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
