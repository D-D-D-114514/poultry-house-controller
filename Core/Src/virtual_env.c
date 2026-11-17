#include "virtual_env.h"
#include "task_scheduler.h"

/*====================================================================*/
/* 虚拟环境传感器 (温湿度随机游走)                                      */
/*====================================================================*/

static int16_t s_temp_x10 = 280;   // 28.0°C
static int16_t s_humi_x10 = 550;   // 55.0%
static uint32_t s_last_update = 0;

/* 简单伪随机数 (线性反馈移位寄存器) */
static uint32_t s_lfsr = 0xABCDEF;

static uint32_t lfsr_rand(void)
{
    /* LFSR: x^32 + x^22 + x^2 + x^1 + 1 */
    uint32_t bit = (s_lfsr ^ (s_lfsr >> 1) ^ (s_lfsr >> 21) ^ (s_lfsr >> 31)) & 1;
    s_lfsr = (s_lfsr >> 1) | (bit << 31);
    return s_lfsr;
}

void VIRTUAL_ENV_Init(void)
{
    s_temp_x10 = 280;   // 28.0°C (起始)
    s_humi_x10 = 550;   // 55.0% (起始)
    s_last_update = TaskScheduler_GetTick();

    /* LFSR 种子使用 uwTick 增加随机性 */
    s_lfsr = TaskScheduler_GetTick() | 0xABCDEF00;
}

void VIRTUAL_ENV_Update(void)
{
    uint32_t now = TaskScheduler_GetTick();
    uint32_t elapsed = now - s_last_update;

    /* 每 5 分钟 (300000ms) 变化一次 */
    if (elapsed < 300000)
        return;

    s_last_update = now;

    /* 温度: ±0.5°C (±5 x10), 范围 26.0~36.0°C */
    int16_t delta_temp = (int16_t)(lfsr_rand() % 11) - 5;  // -5 ~ +5
    s_temp_x10 += delta_temp;
    if (s_temp_x10 < 260) s_temp_x10 = 260;
    if (s_temp_x10 > 360) s_temp_x10 = 360;

    /* 湿度: ±1.0% (±10 x10), 范围 40.0~70.0% */
    int16_t delta_humi = (int16_t)(lfsr_rand() % 21) - 10; // -10 ~ +10
    s_humi_x10 += delta_humi;
    if (s_humi_x10 < 400) s_humi_x10 = 400;
    if (s_humi_x10 > 700) s_humi_x10 = 700;
}

int16_t VIRTUAL_ENV_GetTemperature(void)
{
    return s_temp_x10;
}

int16_t VIRTUAL_ENV_GetHumidity(void)
{
    return s_humi_x10;
}
