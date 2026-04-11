/**
 * @file   minesweep.h
 * @brief  科目2 — 定点排雷 (与科目1相同的INS录制，自动检测旋转方向)
 *
 * 录制流程（与科目1一致）:
 *   StartRecord() → 手推车行驶（Tick_1ms 自动记录 INS）
 *   → 到达雷区 → MarkRotation()（记录当前总距离为旋转点）
 *   → 继续行驶 → ... → StopRecord()
 *
 * 回放流程:
 *   StartReplay() → STABILIZING → DRIVING（纯 INS_ReplayTick 跟踪）
 *   → replay_distance 达到旋转点 → SPIN_ALIGN
 *     (自动根据前后INS航向差判断旋转方向，斜坡PID对齐目标yaw)
 *   → DRIVING（下一段）→ ... → COASTING → DONE
 *
 * 自动旋转方向判断:
 *   取旋转点前后各若干个INS点，计算航向差δ(±180°归一化)
 *   δ > 0 → 航向增大 → 按代码约定为CW(顺时针)
 *   δ < 0 → 航向减小 → CCW(逆时针)
 *   将对应方向的弧长设为Motor连续yaw目标，斜坡生成器平滑推进
 */

#ifndef _MINESWEEP_H_
#define _MINESWEEP_H_

#include "zf_common_headfile.h"

// ─────────────────────────────────────────────────────────────────────────────
//  参数宏
// ─────────────────────────────────────────────────────────────────────────────

#define MS_REPLAY_SPEED         200.0f   // 直线段速度 (RPM)
#define MS_STABILIZE_MS         2000     // 回放前 PID 平衡等待 (ms)
#define MS_COAST_CM             10.0f    // 终点后滑行距离 (cm)
#define MS_SPIN_STEP_DEG_PER_MS 0.072f   // 斜坡生成器每 ms 最大步进 (度/ms = 72°/s)
#define MS_SPIN_LOOK_AHEAD      10       // 检测旋转方向时向后查的 INS 点数
#define MS_SPIN_LOOK_BEFORE     3        // 检测旋转方向时向前查的 INS 点数
#define MS_MAX_MARKERS          20       // 最多旋转标记点数

// ─────────────────────────────────────────────────────────────────────────────
//  数据结构
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  旋转标记点（仅记录触发距离）
 */
typedef struct {
    float dist_cm;   // 录制时的总行驶距离 (cm)，到达此处回放时触发旋转
} ms_marker_t;

// ─────────────────────────────────────────────────────────────────────────────
//  状态枚举
// ─────────────────────────────────────────────────────────────────────────────

typedef enum
{
    MS_IDLE = 0,       // 空闲
    MS_RECORDING,      // 录制中 (INS 自动记录)
    MS_READY,          // 录制完成，等待启动
    MS_STABILIZING,    // PID 平衡阶段 (速度=0)
    MS_DRIVING,        // 直线段行驶 (INS_ReplayTick 跟踪)
    MS_SPIN_ALIGN,     // 斜坡 PID 对齐旋转 (自动方向)
    MS_COASTING,       // 终点后滑行
    MS_DONE,           // 完全停止
} ms_state_t;

// ─────────────────────────────────────────────────────────────────────────────
//  接口函数
// ─────────────────────────────────────────────────────────────────────────────

/** 初始化科目2模块 */
void Minesweep_Init(void);

/** 开始录制（INS_Init + 状态置 RECORDING，Tick_1ms 自动记录 INS） */
void Minesweep_StartRecord(void);

/** 停止录制 */
void Minesweep_StopRecord(void);

/**
 * @brief  标记旋转点（录制中单键调用）
 *         保存当前总距离，回放时在此处自动原地旋转对齐下一段航向
 */
void Minesweep_MarkRotation(void);

/** 开始回放 */
void Minesweep_StartReplay(void);

/** 停止/取消 */
void Minesweep_Stop(void);

/** 1ms 定时调用 (放在 PIT 中断中) */
void Minesweep_Tick_1ms(void);

/** 获取当前状态 */
ms_state_t Minesweep_GetState(void);

/** 获取已标记的旋转点数 */
uint8_t Minesweep_GetMarkerCount(void);

/** 获取录制总距离 (cm) */
float Minesweep_GetRecordDistance(void);

/** 获取回放进度百分比 0~100 */
uint8_t Minesweep_GetReplayProgress(void);

#endif /* _MINESWEEP_H_ */
