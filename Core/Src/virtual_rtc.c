#include "virtual_rtc.h"
#include "task_scheduler.h"
#include <string.h>

static RTC_Time_t s_rtc;
static uint32_t s_last_tick = 0;

void VirtualRTC_Init(void)
{
    memset(&s_rtc, 0, sizeof(s_rtc));
    s_rtc.year  = 2026;
    s_rtc.month = 6;
    s_rtc.day   = 9;
    s_rtc.hour  = 0;
    s_rtc.minute = 0;
    s_rtc.second = 0;
    s_last_tick = TaskScheduler_GetTick();
}

void VirtualRTC_Update(void)
{
    uint32_t now = TaskScheduler_GetTick();
    uint32_t elapsed = now - s_last_tick;

    /* 每1000ms快进1秒 */
    while (elapsed >= 1000) {
        s_rtc.second++;
        if (s_rtc.second >= 60) {
            s_rtc.second = 0;
            s_rtc.minute++;
            if (s_rtc.minute >= 60) {
                s_rtc.minute = 0;
                s_rtc.hour++;
                if (s_rtc.hour >= 24) {
                    s_rtc.hour = 0;
                    s_rtc.day++;
                    if (s_rtc.day > 30) {  // 简化：每月30天
                        s_rtc.day = 1;
                        s_rtc.month++;
                        if (s_rtc.month > 12) {
                            s_rtc.month = 1;
                            s_rtc.year++;
                        }
                    }
                }
            }
        }
        elapsed -= 1000;
        s_last_tick += 1000;
    }
}

RTC_Time_t* VirtualRTC_GetTime(void)
{
    return &s_rtc;
}

uint8_t VirtualRTC_GetHour(void)
{
    return s_rtc.hour;
}

uint8_t VirtualRTC_GetMinute(void)
{
    return s_rtc.minute;
}

uint8_t VirtualRTC_IsBetween(uint8_t start_h, uint8_t start_m,
                             uint8_t end_h, uint8_t end_m)
{
    uint16_t now_min = (uint16_t)s_rtc.hour * 60 + s_rtc.minute;
    uint16_t start_min = (uint16_t)start_h * 60 + start_m;
    uint16_t end_min = (uint16_t)end_h * 60 + end_m;

    if (start_min <= end_min) {
        /* 不跨天: 如 6:00 ~ 20:00 */
        return (now_min >= start_min && now_min <= end_min);
    } else {
        /* 跨天: 如 20:00 ~ 8:00 */
        return (now_min >= start_min || now_min <= end_min);
    }
}
