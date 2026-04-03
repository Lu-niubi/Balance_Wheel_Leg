/**
 * @file   chassic.h
 * @brief  科目1 — 惯性导航取点与回放应用层
 *
 * 状态机:
 *   IDLE → RECORDING → READY → STABILIZING → REPLAYING → DONE
 *
 * 外部调用:
 *   - Chassic_Init()          : 上电初始化
 *   - Chassic_StartRecord()   : 开始取点 (从菜单触发)
 *   - Chassic_StopRecord()    : 停止取点 (从菜单触发)
 *   - Chassic_StartReplay()   : 开始回放 (从菜单触发)
 *   - Chassic_Tick_2ms()      : 2ms 定时调用 (放在 PIT 中断)
 *   - Chassic_GetState()      : 获取当前状态 (菜单显示用)
 */

#ifndef _CHASSIC_H_
#define _CHASSIC_H_

#include "zf_common_headfile.h"

// ─────────────────────────────────────────────────────────────────────────────
//  参数宏
// ─────────────────────────────────────────────────────────────────────────────

#define CHASSIC_REPLAY_SPEED    200.0f   // 回放时的固定目标速度 (RPM)
#define CHASSIC_STABILIZE_MS    2000     // 回放前PID平稳等待时间 (ms)

// ─────────────────────────────────────────────────────────────────────────────
//  状态枚举
// ─────────────────────────────────────────────────────────────────────────────

typedef enum
{
    CHASSIC_IDLE = 0,       // 空闲
    CHASSIC_RECORDING,      // 取点中 (手推车, PID关闭)
    CHASSIC_READY,          // 取点完成, 等待启动回放
    CHASSIC_STABILIZING,    // PID 平稳阶段 (速度=0, 等待平衡)
    CHASSIC_REPLAYING,      // 回放航迹中
    CHASSIC_DONE,           // 回放完成
} chassic_state_t;

// ─────────────────────────────────────────────────────────────────────────────
//  接口函数
// ─────────────────────────────────────────────────────────────────────────────

/** 初始化科目1模块 */
void Chassic_Init(void);

/** 开始取点 (禁用PID，手推车记录轨迹) */
void Chassic_StartRecord(void);

/** 停止取点 */
void Chassic_StopRecord(void);

/** 开始回放 (先平稳再按轨迹行驶) */
void Chassic_StartReplay(void);

/** 停止/取消回放 */
void Chassic_Stop(void);

/**
 * @brief  2ms 定时调用 (必须放在 PIT 中断中)
 * @note   内部根据状态自动执行 Recording / Replay 逻辑
 */
void Chassic_Tick_2ms(void);

/** 获取当前状态 */
chassic_state_t Chassic_GetState(void);

/** 获取已记录点数 (菜单显示用) */
uint16_t Chassic_GetRecordCount(void);

/** 获取已记录总距离 cm (菜单显示用) */
float Chassic_GetRecordDistance(void);

/** 获取回放进度百分比 0~100 */
uint8_t Chassic_GetReplayProgress(void);

#endif /* _CHASSIC_H_ */
