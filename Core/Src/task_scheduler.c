#include "task_scheduler.h"
#include "main.h"
#include <string.h>

/* uwTick 定时由 HAL 的 SysTick_Handler 递增 */
extern __IO uint32_t uwTick;

/*====================================================================*/
/* 调度器配置                                                          */
/*====================================================================*/
#define MAX_TASKS           8

/*====================================================================*/
/* 内部变量                                                            */
/*====================================================================*/
static Task_t s_tasks[MAX_TASKS];
static uint8_t s_task_count = 0;

/*====================================================================*/
/* API 实现                                                            */
/*====================================================================*/

void TaskScheduler_Init(void)
{
    s_task_count = 0;
    memset(s_tasks, 0, sizeof(s_tasks));
}

int TaskScheduler_Register(TaskFunc_t func, uint32_t period_ms,
                           uint8_t priority, const char *name)
{
    if (s_task_count >= MAX_TASKS || func == NULL)
        return -1;

    s_tasks[s_task_count].func       = func;
    s_tasks[s_task_count].period_ms  = period_ms;
    s_tasks[s_task_count].last_run   = uwTick;
    s_tasks[s_task_count].priority   = priority;
    s_tasks[s_task_count].name       = name;

    s_task_count++;
    return 0;
}

void TaskScheduler_Run(void)
{
    uint32_t now = uwTick;

    /* 按优先级从高到低扫描 */
    for (int prio = 4; prio >= 0; prio--) {
        for (int i = 0; i < s_task_count; i++) {
            if (s_tasks[i].priority != (uint8_t)prio)
                continue;

            /* 判断是否到执行时间 */
            uint32_t elapsed = now - s_tasks[i].last_run;
            if (elapsed >= s_tasks[i].period_ms) {
                /* 加 period_ms 而非设为 now: 消除累积漂移 */
                s_tasks[i].last_run += s_tasks[i].period_ms;
                s_tasks[i].func();
            }
        }
    }
}

uint32_t TaskScheduler_GetTick(void)
{
    return uwTick;
}

void TaskScheduler_Delay(uint32_t ms)
{
    uint32_t start = uwTick;
    while ((uwTick - start) < ms) {
        /* 等待 */
    }
}
