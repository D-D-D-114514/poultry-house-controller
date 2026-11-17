#include "stepper.h"
#include "project_config.h"

/*====================================================================*/
/* 步进电机驱动                                                        */
/*====================================================================*/

/* 全局电机实例 */
StepperMotor_t g_x_motor;
StepperMotor_t g_z_motor;

void STEPPER_Init(StepperMotor_t *motor, TIM_HandleTypeDef *htim,
                  GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    motor->htim   = htim;
    motor->channel = TIM_CHANNEL_1;
    motor->dir_port = dir_port;
    motor->dir_pin  = dir_pin;
    motor->state     = MOTOR_STOPPED;
    motor->direction = 1;
    motor->target_pulses = 0;
    motor->pulse_count   = 0;

    /* 方向引脚初始化为 LOW (正转) */
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
}

void STEPPER_Start(StepperMotor_t *motor, uint8_t direction, uint32_t pulses)
{
    motor->direction    = direction;
    motor->target_pulses = pulses;
    motor->pulse_count   = 0;
    motor->state = MOTOR_RUNNING;

    /* 设置方向 */
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                      direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 设为安全运行频率(避免直接全速 2000Hz 导致失步/假堵转) */
    STEPPER_SetFreq(motor, STEPPER_RUN_HZ);

    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

void STEPPER_Stop(StepperMotor_t *motor)
{
    motor->state = MOTOR_STOPPED;
    motor->target_pulses = 0;
    motor->pulse_count   = 0;

    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
}

uint8_t STEPPER_IsRunning(StepperMotor_t *motor)
{
    return (motor->state == MOTOR_RUNNING);
}

void STEPPER_SetDirection(StepperMotor_t *motor, uint8_t forward)
{
    motor->direction = forward;
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                      forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void STEPPER_SetFreq(StepperMotor_t *motor, uint16_t freq_hz)
{
    /* 计数时钟 = 8MHz / (PSC=99+1) = 80kHz (与 TIM2/TIM3 初始化一致) */
    const uint32_t cnt_clk = 80000U;

    /* 频率夹位: 下限防除零/ARR溢出(65535), 上限防 ARR=0 */
    if (freq_hz < 50)    freq_hz = 50;       /* ARR=1599 < 65535, 安全 */
    if (freq_hz > 40000) freq_hz = 40000;    /* ARR=1, 仍可用 */

    uint32_t arr = (cnt_clk / freq_hz);
    if (arr == 0) arr = 1;
    arr -= 1;

    /* 仅改 ARR 与比较值, 50% 占空比. 不改 PSC */
    __HAL_TIM_SET_AUTORELOAD(motor->htim, arr);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (arr + 1) / 2);
}
