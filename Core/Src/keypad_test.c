/*
 * keypad_test.c - 按键 ADC 自动标定程序
 *
 * 算法: RAW > IDLE_RAW = 空闲。跌破即累加"稳定段"采样(相邻差<=STABLE_DELTA),
 *       松开时算平均 RAW, 并按 ADC 值自动归类到 12 个槽位 (S1~S11 + KEY13)。
 *       按一圈即可自动填满, 无需手抄。
 *
 * OLED 显示: 滚动列出已标定的键和其平均值。
 * 调用周期 10ms。PB0 闪烁=运行中。
 */

#include "main.h"
#include "project_config.h"
#include "ssd1306.h"
#include "task_scheduler.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;

#define IDLE_RAW        3980   // RAW > 此值视为空闲 (S11上限也是3980)
#define STABLE_DELTA    25     // 相邻采样差 <= 视为稳定
#define MIN_SAMPLES     10     // 至少稳定 100ms 才算有效

/* 12 个键槽位: 按 ADC 升序排列, 槽位0~11 = S1~S12 (物理键对齐) */
#define SLOT_COUNT      12
static uint16_t s_slot_avg[SLOT_COUNT];   // 各键标定的平均值, 0=未测
static const char* s_slot_name[SLOT_COUNT] = {
    "S1","S2","S3","S4","S5","S6","S7","S8","S9","S10","S11","S12"
};

/* ADC 读取 (PC4, IN14) */
static uint16_t test_read_adc(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        return 0;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint16_t val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return val;
    }
    HAL_ADC_Stop(&hadc1);
    return 0;
}

/* 按平均 ADC 值归类到槽位 (ADC升序, 槽0=最低=S1) */
static int classify(uint16_t avg)
{
    /* 实测分布(ADC升序): S1=0 S2=2152 S3=3022 S4=3382 S5=3573 S6=3691
       S7=3792 S8=3860 S9=3900 S10=3938 S11=3966 S12=3969           */
    if (avg <= 150)                  return 0;   // S1  (KEY13, ADC≈0)
    if (avg >= 1900 && avg <= 2400)  return 1;   // S2  (2152)
    if (avg >= 2850 && avg <= 3150)  return 2;   // S3  (3022)
    if (avg >= 3250 && avg <= 3460)  return 3;   // S4  (3382)
    if (avg >= 3500 && avg <= 3630)  return 4;   // S5  (3573)
    if (avg >= 3640 && avg <= 3740)  return 5;   // S6  (3691)
    if (avg >= 3750 && avg <= 3825)  return 6;   // S7  (3792)
    if (avg >= 3830 && avg <= 3880)  return 7;   // S8  (3860)
    if (avg >= 3885 && avg <= 3918)  return 8;   // S9  (3900)
    if (avg >= 3922 && avg <= 3952)  return 9;   // S10 (3938)
    if (avg >= 3955 && avg <= 3980)  return 10;  // S11 (3966, 含S12 3969 合并)
    return -1;  // 落在间隙, 无法归类
}

/* 状态机变量 */
static uint8_t  s_recording = 0;
static uint32_t s_sum = 0;
static uint32_t s_count = 0;
static uint16_t s_prev_raw = 0;

/* 最近一次结果 */
static uint16_t s_last_avg = 0;
static int      s_last_slot = -1;

/* 显示滚动起点 */
static uint8_t  s_disp_page = 0;

void KEYPAD_RunTest(void)
{
    char line[17];
    uint16_t adc_val = test_read_adc();

    /* ---- 状态机: 只累加稳定段 ---- */
    if (adc_val <= IDLE_RAW) {
        if (!s_recording) {
            s_recording = 1;
            s_sum = 0;
            s_count = 0;
        } else {
            int diff = (int)adc_val - (int)s_prev_raw;
            if (diff < 0) diff = -diff;
            if (diff <= STABLE_DELTA) {
                s_sum += adc_val;
                s_count++;
            } else {
                s_sum = 0;
                s_count = 0;
            }
        }
    } else {
        if (s_recording && s_count >= MIN_SAMPLES) {
            uint16_t avg = (uint16_t)(s_sum / s_count);
            int slot = classify(avg);
            s_last_avg = avg;
            s_last_slot = slot;
            if (slot >= 0) {
                s_slot_avg[slot] = avg;   // 存入对应槽位
            }
        }
        s_recording = 0;
        s_sum = 0;
        s_count = 0;
    }
    s_prev_raw = adc_val;

    /* ---- OLED 显示降频 (每200ms) ---- */
    static uint8_t disp_div = 0;
    if (++disp_div < 20)
        return;
    disp_div = 0;

    SSD1306_Clear();

    /* 第0行: 当前状态 + 最近识别结果 */
    if (s_recording) {
        snprintf(line, sizeof(line), "REC N:%lu", (unsigned long)s_count);
    } else if (s_last_slot >= 0) {
        snprintf(line, sizeof(line), "OK %s=%d",
                 s_slot_name[s_last_slot], s_last_avg);
    } else if (s_last_avg > 0) {
        snprintf(line, sizeof(line), "?? AVG=%d", s_last_avg);
    } else {
        snprintf(line, sizeof(line), "PRESS A KEY");
    }
    SSD1306_DrawString(0, 0, line);

    /* 第1~3行: 显示当前页的 3 个槽位 (12槽分4页, 每页3个) */
    uint8_t base = s_disp_page * 3;
    for (int i = 0; i < 3; i++) {
        int idx = base + i;
        if (idx >= SLOT_COUNT) break;
        if (s_slot_avg[idx] > 0)
            snprintf(line, sizeof(line), "%s:%d", s_slot_name[idx], s_slot_avg[idx]);
        else
            snprintf(line, sizeof(line), "%s:----", s_slot_name[idx]);
        SSD1306_DrawString(0, 16 + i*16, line);
    }

    /* 页轮换: 约每 1.6 秒换一页 (8 次显示刷新 x 200ms) */
    static uint8_t page_div = 0;
    if (++page_div >= 8) {
        page_div = 0;
        s_disp_page = (uint8_t)((s_disp_page + 1) % 4);
    }

    SSD1306_Display();

    HAL_GPIO_TogglePin(FLUORESCENT_PORT, FLUORESCENT_PIN);
}
