#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"
#include <stdint.h>

/* 步进电机控制 (X轴: TIM2, Z轴: TIM3) */

typedef enum { MOTOR_STOPPED, MOTOR_RUNNING } MotorState_t;

typedef struct {
    TIM_HandleTypeDef *htim;     // TIM2 或 TIM3
    uint16_t           channel;  // TIM_CHANNEL_1
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    MotorState_t       state;
    uint8_t            direction; // 1=正转, 0=反转
    uint32_t           target_pulses;  // 0=无限
    uint32_t           pulse_count;
} StepperMotor_t;

extern StepperMotor_t g_x_motor;
extern StepperMotor_t g_z_motor;

void STEPPER_Init(StepperMotor_t *motor, TIM_HandleTypeDef *htim,
                  GPIO_TypeDef *dir_port, uint16_t dir_pin);

/**
 * @brief  启动电机
 * @param  motor     电机实例
 * @param  direction 1=正转, 0=反转
 * @param  pulses    脉冲数 (0=连续运行)
 */
void STEPPER_Start(StepperMotor_t *motor, uint8_t direction, uint32_t pulses);
void STEPPER_Stop(StepperMotor_t *motor);
uint8_t STEPPER_IsRunning(StepperMotor_t *motor);
void STEPPER_SetDirection(StepperMotor_t *motor, uint8_t forward);

/**
 * @brief  运行时设置 PWM 步进频率 (保持 PSC 不变, 仅改 ARR), 50% 占空比
 * @param  motor   电机实例
 * @param  freq_hz 目标频率 (Hz), 内部夹到 [50, 80000]
 * @note   计数时钟固定 80kHz (8MHz / (PSC=99+1)). ARR = 80000/freq_hz - 1
 */
void STEPPER_SetFreq(StepperMotor_t *motor, uint16_t freq_hz);

#endif
