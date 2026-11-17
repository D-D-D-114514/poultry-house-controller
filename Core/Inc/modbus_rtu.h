#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "main.h"
#include <stdint.h>

/* Modbus RTU Slave, 双串口 (USART1 + USART2) */

#define MODBUS_REG_COUNT    35
#define MODBUS_RX_BUF_SIZE  64

void MODBUS_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2,
                 uint8_t slave_id);

/**
 * @brief  处理 Modbus 请求 (由任务周期性调用)
 */
void MODBUS_Process(void);

/**
 * @brief  USART 接收中断回调 (在 HAL_UART_RxCpltCallback 中调用)
 */
void MODBUS_UART_RxCallback(UART_HandleTypeDef *huart);

/**
 * @brief  USART 错误(溢出ORE等)恢复, 重启接收链 (在 HAL_UART_ErrorCallback 中调用)
 */
void MODBUS_UART_ErrorRestart(UART_HandleTypeDef *huart);

/**
 * @brief  读写寄存器
 */
void MODBUS_SetRegister(uint16_t reg_idx, uint16_t value);
uint16_t MODBUS_GetRegister(uint16_t reg_idx);

#endif
