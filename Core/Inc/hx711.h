#ifndef HX711_H
#define HX711_H

#include <stdint.h>

/* HX711 3通道分时读取 (共享 SCK) */

#define HX711_CHANNEL_COUNT    3

void HX711_Init(void);

/**
 * @brief  读取指定通道的原始值
 * @param  channel 0,1,2
 * @retval 24位有符号原始值, 0=失败
 */
int32_t HX711_ReadChannel(uint8_t channel);

/**
 * @brief  获取指定通道的重量 (克)
 * @param  channel 0,1,2
 * @retval 重量 (克)
 */
float HX711_GetGrams(uint8_t channel);

/**
 * @brief  获取模拟重量值 (用于虚拟测试)
 *         实际部署时替换为真实 HX711_ReadChannel
 */
float HX711_GetSimGrams(uint8_t channel);

/**
 * @brief  设置模拟料位 (用于测试)
 */
void HX711_SetSimGrams(uint8_t channel, float grams);

#endif
