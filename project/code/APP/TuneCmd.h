#ifndef _TUNECMD_H_
#define _TUNECMD_H_

#include "stdint.h"
#include "zf_common_typedef.h"

// 喂给解析器一个字节（在 uart0_isr 里调用）
void TuneCmd_Feed(uint8 byte);

// 在主循环或低优先级任务里调用，处理已接收的命令
void TuneCmd_Process(void);

#endif // _TUNECMD_H_
