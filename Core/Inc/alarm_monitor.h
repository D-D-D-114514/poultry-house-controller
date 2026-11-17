#ifndef ALARM_MONITOR_H
#define ALARM_MONITOR_H

/* 报警监控 (水位/配送/供料/温湿度超时) */

void ALARM_Init(void);
void ALARM_Monitor(void);       // 每 500ms 调用

#endif
