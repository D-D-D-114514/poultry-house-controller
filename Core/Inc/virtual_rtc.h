#ifndef VIRTUAL_RTC_H
#define VIRTUAL_RTC_H

#include <stdint.h>

/* 软件 RTC，由任务周期性调用 Update */

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} RTC_Time_t;

void VirtualRTC_Init(void);
void VirtualRTC_Update(void);       // 每秒调用一次
RTC_Time_t* VirtualRTC_GetTime(void);
uint8_t VirtualRTC_GetHour(void);
uint8_t VirtualRTC_GetMinute(void);

/**
 * 判断当前时间是否在 start_h:start_m 到 end_h:end_m 之间 (含两端)
 * 支持跨天 (如 20:00 ~ 8:00)
 */
uint8_t VirtualRTC_IsBetween(uint8_t start_h, uint8_t start_m,
                             uint8_t end_h, uint8_t end_m);

#endif
