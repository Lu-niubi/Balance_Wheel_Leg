#ifndef _GPS_H_
#define _GPS_H_

#include "zf_common_headfile.h"

// GPS 模块封装头文件
// 底层驱动由 zf_device_gnss 提供，本模块提供业务层的初始化与数据访问接口

// 直接复用 zf_device_gnss 的全局结构体和标志位
// extern gnss_info_struct gnss;   -- 已在 zf_device_gnss.h 中声明
// extern uint8 gnss_flag;         -- 已在 zf_device_gnss.h 中声明

void GPS_Init(void);
uint8 GPS_Update(void);

#endif
