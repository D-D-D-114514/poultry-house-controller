#ifndef VIRTUAL_ENV_H
#define VIRTUAL_ENV_H

#include <stdint.h>

/* 虚拟环境传感器 (温湿度) */

void VIRTUAL_ENV_Init(void);

/**
 * @brief  更新虚拟数据 (每200ms调用)
 */
void VIRTUAL_ENV_Update(void);

/**
 * @brief  获取温度 (x10, 如 265 = 26.5℃)
 */
int16_t VIRTUAL_ENV_GetTemperature(void);

/**
 * @brief  获取湿度 (x10, 如 550 = 55.0%)
 */
int16_t VIRTUAL_ENV_GetHumidity(void);

#endif
