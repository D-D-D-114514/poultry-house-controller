#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <stdint.h>

/* 手动控制模式 (按键直接控制执行器) */

extern volatile uint8_t g_manual_active;

void MANUAL_Init(void);
void MANUAL_Scan(void);         // 每 10ms 调用

#endif
