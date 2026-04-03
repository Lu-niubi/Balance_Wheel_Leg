/**
 * @file   Menu.h
 * @brief  串口屏菜单系统
 *
 * 按键映射 (P20_0~P20_3 = KEY_1~KEY_4):
 *   KEY_1 (P20_0) -- 上 / 减小
 *   KEY_2 (P20_1) -- 下 / 增大
 *   KEY_3 (P20_2) -- 确认 / 进入 / 开始运行
 *   KEY_4 (P20_3) -- 返回 / 切换参数(启动阶段)
 *
 * 启动阶段 (STATE_STARTUP_PID):
 *   上电后先进入PID预调界面，KEY_1/KEY_2调整参数，KEY_4切换参数，
 *   KEY_3确认写入并开始正式运行 (Menu_IsStartupDone() 返回 1)
 *
 * 菜单结构 (启动完成后):
 *   主菜单
 *   ├─ 0: IMU 数据
 *   ├─ 1: PID 参数
 *   │    ├─ 0: GYRO_KP  (步进 10)
 *   │    ├─ 1: ANG_KP   (步进 0.2)
 *   │    ├─ 2: ANG_KI   (步进 0.001)
 *   │    └─ 3: ANG_KD   (步进 0.01)
 *   └─ 2: 小车速度
 */

#ifndef _MENU_H_
#define _MENU_H_

// 启动PID预调阶段开关
// 注释掉此行可跳过上电预调，直接进入主菜单开始运行
// #define MENU_STARTUP_PID_TUNE

#include <stdint.h>

void    Menu_Init(void);
void    Menu_Process(void);
uint8_t Menu_IsStartupDone(void);

#endif /* _MENU_H_ */
