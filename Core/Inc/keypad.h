#ifndef KEYPAD_H
#define KEYPAD_H

#include <stdint.h>

/* ADC 按键扫描 (PC4, ADC_IN14, 13x10K 电阻分压) */

typedef enum {
    KEY_NONE = 0,
    KEY_S1,   // Z电机正转(主)    ADC≈0
    KEY_S2,   // X电机正转        ADC 2152
    KEY_S3,   // 电磁阀1 开关切换  ADC 3022
    KEY_S4,   // 电磁阀2 开关切换  ADC 3382
    KEY_S5,   // 加湿阀 开关切换   ADC 3573
    KEY_S6,   // 电磁阀3 开关切换  ADC 3691
    KEY_S7,   // OLED下翻页        ADC 3792
    KEY_S8,   // OLED上翻页        ADC 3860
    KEY_S9,   // 手动模式开关      ADC 3900
    KEY_S10,  // Z电机反转(主)    ADC 3938
    KEY_S11,  // X电机反转        ADC 3966
    KEY_S12   // 弃用(与S11撞车)  ADC 3969
} KeyCode_t;

void KEYPAD_Init(void);

/**
 * @brief  扫描按键 (含消抖)
 * @retval 当前按下的键码, KEY_NONE = 无按键
 */
KeyCode_t KEYPAD_Scan(void);

/**
 * @brief  检测是否有手动控制按键按下
 *         手动键: S3,S4,S5,S6,S9,S10,S11,S12 (+ S1,S2备用)
 */
uint8_t KEYPAD_IsAnyManualKey(void);

/**
 * @brief  检测翻页按键
 */
uint8_t KEYPAD_IsPageUp(void);
uint8_t KEYPAD_IsPageDown(void);

#endif
