#ifndef AUTO_CONTROL_H
#define AUTO_CONTROL_H

/* 自动控制逻辑 (喂水/喂料/温控/湿控/通风/光照) */

void AUTO_Control_Init(void);
void AUTO_Control_Run(void);    // 每 200ms 调用

#endif
