#ifndef _IPS200_MODULE_H_
#define _IPS200_MODULE_H_

#include "zf_common_headfile.h"

// IPS200 显示模块封装头文件
// 底层驱动由 zf_device_ips200 提供，本模块提供业务层的初始化与 GNSS 数据显示接口

// 接口模式选择：使用并口（Parallel 8080）获得最高刷新率
#define IPS200_MODULE_TYPE  (IPS200_TYPE_SPI)

void IPS200_Init(void);
void IPS200_ShowGNSS(void);

#endif
