#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <stdint.h>

/*====================================================================*/
/* 协作式任务调度器                                                    */
/* 利用 SysTick 实现固定周期轮询调度                                    */
/*====================================================================*/

/* 任务函数原型 */
typedef void (*TaskFunc_t)(void);

/* 任务优先级 (数值越大优先级越高) */
typedef enum {
    TASK_PRIO_OLED = 0,      // 最低
    TASK_PRIO_ALARM = 1,
    TASK_PRIO_AUTO = 2,
    TASK_PRIO_MANUAL = 3,
    TASK_PRIO_MODBUS = 4     // 最高
} TaskPriority_t;

/* 任务控制块 */
typedef struct {
    TaskFunc_t    func;          // 任务函数
    uint32_t      period_ms;     // 执行周期 (ms)
    uint32_t      last_run;      // 上次执行时刻 (ms)
    uint8_t       priority;      // 优先级 (0-4)
    const char   *name;          // 任务名
} Task_t;

/*====================================================================*/
/* API                                                                 */
/*====================================================================*/

/**
 * @brief  初始化任务调度器
 * @note   必须在 HAL_Init() 之后调用
 */
void TaskScheduler_Init(void);

/**
 * @brief  注册一个周期性任务
 * @param  func      任务函数
 * @param  period_ms 周期 (ms)
 * @param  priority  优先级 (0-4)
 * @param  name      任务名称
 * @retval 0=成功, -1=失败
 */
int TaskScheduler_Register(TaskFunc_t func, uint32_t period_ms,
                           uint8_t priority, const char *name);

/**
 * @brief  运行调度器 (在主循环中无限调用)
 */
void TaskScheduler_Run(void);

/**
 * @brief  获取系统当前运行毫秒数
 */
uint32_t TaskScheduler_GetTick(void);

/**
 * @brief  延时指定毫秒 (阻塞)
 */
void TaskScheduler_Delay(uint32_t ms);

#endif /* TASK_SCHEDULER_H */
