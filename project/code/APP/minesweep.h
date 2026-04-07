/**
 * @file   minesweep.h
 * @brief  科目2 — 定点排雷 (纯惯导)
 *
 * 状态机:
 *   IDLE → RECORDING → READY → STABILIZING → DRIVING → SPINNING → SPIN_ALIGN → ... → COASTING → DONE
 *
 * 录制: 手推车，按键标记雷区中心点 (记录 IMU raw yaw + 累积距离)
 * 回放: 直线段 yaw PID 跟踪 → 到达雷区点 → 固定PWM差速旋转两圈 → 重新映射yaw → 下一段直线
 */

#ifndef _MINESWEEP_H_
#define _MINESWEEP_H_

#include "zf_common_headfile.h"

// ─────────────────────────────────────────────────────────────────────────────
//  参数宏
// ─────────────────────────────────────────────────────────────────────────────

#define MS_REPLAY_SPEED       200.0f    // 直线段速度 (RPM)
#define MS_STABILIZE_MS       2000      // 回放前 PID 平衡等待 (ms)
#define MS_COAST_CM           10.0f     // 最后一个点后滑行距离 (cm)
#define MS_SPIN_PWM           400       // 旋转差速 PWM (正值=顺时针, 需调试)
#define MS_SPIN_TOTAL_DEG     720.0f    // 旋转总量 (两圈)
#define MS_SPIN_ALIGN_MS      500       // 旋转后 IMU 稳定等待 (ms)
#define MS_MAX_MARKERS        20        // 最多标记雷区数

// ─────────────────────────────────────────────────────────────────────────────
//  状态枚举
// ─────────────────────────────────────────────────────────────────────────────

typedef enum
{
    MS_IDLE = 0,           // 空闲
    MS_RECORDING,          // 取点中 (手推车, 按键标记)
    MS_READY,              // 取点完成, 等待启动
    MS_STABILIZING,        // PID 平衡阶段 (速度=0)
    MS_DRIVING,            // 直线段行驶 (yaw PID 跟踪)
    MS_SPINNING,           // 原地旋转两圈 (固定 PWM 差速)
    MS_SPIN_ALIGN,         // 旋转后稳定 + yaw 重新映射
    MS_COASTING,           // 最后滑行
    MS_DONE,               // 完全停止
} ms_state_t;

// ─────────────────────────────────────────────────────────────────────────────
//  接口函数
// ─────────────────────────────────────────────────────────────────────────────

/** 初始化科目2模块 */
void Minesweep_Init(void);

/** 开始取点 (手推车, 按键标记雷区中心) */
void Minesweep_StartRecord(void);

/** 停止取点 */
void Minesweep_StopRecord(void);

/** 标记当前车位置为一个雷区中心点 (录制中调用) */
void Minesweep_MarkPoint(void);

/** 开始回放 */
void Minesweep_StartReplay(void);

/** 停止/取消 */
void Minesweep_Stop(void);

/**
 * @brief  1ms 定时调用 (放在 PIT 中断中)
 */
void Minesweep_Tick_1ms(void);

/** 获取当前状态 */
ms_state_t Minesweep_GetState(void);

/** 获取已标记的雷区数 */
uint8_t Minesweep_GetMarkerCount(void);

/** 获取录制总距离 (cm) */
float Minesweep_GetRecordDistance(void);

/** 获取回放进度百分比 0~100 */
uint8_t Minesweep_GetReplayProgress(void);

#endif /* _MINESWEEP_H_ */
