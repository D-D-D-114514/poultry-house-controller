/*
 * stepper_test.c - 步进电机自动扫频测试 (往复版)
 *
 * 目的: 验证「疑似堵转」是否为失步。STEPPER_Start 一上来就全速 2000Hz,
 *       无加减速斜坡。本测试从低频逐级升到高频, 找出该电机实际失步临界频率。
 *
 * 流程(只测 X 电机 / TIM2 / PA0):
 *   每个频率档位先正转一段, 再反转同样时长 -> 同频同时间, 每档净位移≈0,
 *   不累积漂移, 不会跑出机械行程。扫完最高档 -> 停机 2s -> 从头循环。
 *
 * OLED: 行0 STEP n/N, 行1 FREQ:xxxx Hz, 行2 DIR FWD/REV, 行3 RUN/STOP。
 * PB0(日光灯脚) 翻转做运行心跳。调用周期 10ms。
 */

#include "main.h"
#include "project_config.h"
#include "stepper.h"
#include "ssd1306.h"
#include <stdio.h>

/* 频率档位表 (Hz, 升序) */
static const uint16_t s_freq_steps[] = {
    200, 400, 600, 800, 1000, 1200, 1500, 1800, 2000, 2500, 3000
};
#define FREQ_STEP_COUNT  (sizeof(s_freq_steps) / sizeof(s_freq_steps[0]))

/* 时序 (单位: 10ms 调用次数) */
#define DIR_TICKS        150   /* 每个方向运行 1.5s (原 3s 单向差点超程, 减半) */
#define IDLE_TICKS       200   /* 一轮结束停机 2s */

/* 测试状态机 */
typedef enum { TS_SWEEP, TS_IDLE } TestState_t;
static TestState_t s_state    = TS_SWEEP;
static uint8_t     s_step_idx = 0;       /* 当前频率档位下标 */
static uint8_t     s_dir      = 1;       /* 当前方向 1=正转, 0=反转 */
static uint32_t    s_tick     = 0;       /* 当前阶段已计 10ms 数 */
static uint8_t     s_started  = 0;       /* 电机是否已启动 */
static uint8_t     s_disp_div = 0;       /* OLED 刷新降频计数 */

/* 进入新的(档位, 方向)阶段: 设频率与方向, 首次启动 PWM */
static void apply(void)
{
    if (!s_started) {
        STEPPER_Start(&g_x_motor, s_dir, 0);   /* 连续运行(内部会设 RUN_HZ) */
        s_started = 1;
    }
    /* 在 Start 之后设频率, 覆盖其内部默认值, 让扫频档位生效 */
    STEPPER_SetFreq(&g_x_motor, s_freq_steps[s_step_idx]);
    STEPPER_SetDirection(&g_x_motor, s_dir);   /* 换向: PWM 持续, 仅翻 DIR */
}

static void draw_oled(void)
{
    char line[17];

    /* 降频: 每 ~200ms 刷一次 (20 * 10ms) */
    if (++s_disp_div < 20)
        return;
    s_disp_div = 0;

    SSD1306_Clear();

    if (s_state == TS_SWEEP) {
        uint16_t freq = s_freq_steps[s_step_idx];

        snprintf(line, sizeof(line), "STEP %u/%u",
                 (unsigned)(s_step_idx + 1), (unsigned)FREQ_STEP_COUNT);
        SSD1306_DrawString(0, 0, line);

        snprintf(line, sizeof(line), "FREQ:%u Hz", (unsigned)freq);
        SSD1306_DrawString(0, 16, line);

        snprintf(line, sizeof(line), "DIR:%s", s_dir ? "FWD" : "REV");
        SSD1306_DrawString(0, 32, line);

        SSD1306_DrawString(0, 48, "RUN");
    } else {
        SSD1306_DrawString(0, 0,  "SWEEP DONE");
        SSD1306_DrawString(0, 16, "MOTOR STOP");
        SSD1306_DrawString(0, 32, "restart...");
        SSD1306_DrawString(0, 48, "STOP");
    }

    SSD1306_Display();
    HAL_GPIO_TogglePin(FLUORESCENT_PORT, FLUORESCENT_PIN);  /* PB0 心跳 */
}

void STEPPER_RunTest(void)
{
    switch (s_state) {
    case TS_SWEEP:
        /* 进入新阶段时设置频率与方向 */
        if (s_tick == 0)
            apply();

        if (++s_tick >= DIR_TICKS) {
            s_tick = 0;
            if (s_dir) {
                /* 正转结束 -> 同档反转(回程) */
                s_dir = 0;
            } else {
                /* 反转结束(本档往复完成) -> 切下一档, 方向复位为正 */
                s_dir = 1;
                if (s_step_idx + 1 < FREQ_STEP_COUNT) {
                    s_step_idx++;
                } else {
                    STEPPER_Stop(&g_x_motor);
                    s_started = 0;
                    s_state = TS_IDLE;
                }
            }
        }
        break;

    case TS_IDLE:
        if (++s_tick >= IDLE_TICKS) {
            s_tick = 0;
            s_step_idx = 0;
            s_dir = 1;
            s_state = TS_SWEEP;        /* 从头循环 */
        }
        break;
    }

    draw_oled();
}
