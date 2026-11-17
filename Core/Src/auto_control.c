#include "auto_control.h"
#include "project_config.h"
#include "virtual_env.h"
#include "virtual_rtc.h"
#include "hx711.h"
#include "stepper.h"
#include "task_scheduler.h"

/*====================================================================*/
/* 自动控制逻辑                                                        */
/*====================================================================*/

/* 外部全局变量 (来自项目配置和应用模块) */
extern volatile uint8_t g_manual_active;

/* 配送系统全局变量 (定义) */
volatile uint8_t g_delivery_active = 0;
volatile uint8_t g_delivery_target = 0;

/* ADC 水位值 (由 ADC 读取更新) */
uint16_t g_adc_water[3] = {4095, 4095, 4095};

/* 电磁阀状态 */
uint8_t g_water_valve[3] = {0, 0, 0};

/* 配送状态机 */
static uint8_t s_delivery_phase = 0;  // 0=空闲/去程, 1=投料, 2=回程
static uint32_t s_phase_tick = 0;

/*====================================================================*/
/* 读取 3 路水位 ADC                                                  */
/*====================================================================*/
extern ADC_HandleTypeDef hadc1;

static void read_water_levels(void)
{
    uint32_t ch_channels[4] = {ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_14};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* 注意: 同时读取按键 ADC, 但这里只读水位 */
    for (int i = 0; i < 3; i++) {
        sConfig.Channel = ch_channels[i];
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            return;

        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            g_adc_water[i] = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
}

/*====================================================================*/
/* 执行器控制 (内部)                                                   */
/*====================================================================*/

static void set_valve(uint8_t idx, uint8_t on)
{
    if (idx >= 3) return;
    g_water_valve[idx] = on;

    switch (idx) {
    case 0: on ? ACT_ON(VALVE1) : ACT_OFF(VALVE1); break;
    case 1: on ? ACT_ON(VALVE2) : ACT_OFF(VALVE2); break;
    case 2: on ? ACT_ON(VALVE3) : ACT_OFF(VALVE3); break;
    }
}

static void set_water_led(uint8_t idx, uint8_t on)
{
    switch (idx) {
    case 0: on ? ACT_ON(WATER_LED1) : ACT_OFF(WATER_LED1); break;
    case 1: on ? ACT_ON(WATER_LED2) : ACT_OFF(WATER_LED2); break;
    case 2: on ? ACT_ON(WATER_LED3) : ACT_OFF(WATER_LED3); break;
    }
}

static void set_feed_led(uint8_t idx, uint8_t on)
{
    switch (idx) {
    case 0: on ? ACT_ON(FEED_LED1) : ACT_OFF(FEED_LED1); break;
    case 1: on ? ACT_ON(FEED_LED2) : ACT_OFF(FEED_LED2); break;
    case 2: on ? ACT_ON(FEED_LED3) : ACT_OFF(FEED_LED3); break;
    }
}

static void set_cooling_fan(uint8_t on)
{
    on ? ACT_ON(COOLING_FAN) : ACT_OFF(COOLING_FAN);
}

static void set_incandescent(uint8_t on)
{
    on ? ACT_ON(INCANDESCENT) : ACT_OFF(INCANDESCENT);
}

static void set_vent_fan(uint8_t on)
{
    on ? ACT_ON(VENT_FAN) : ACT_OFF(VENT_FAN);
}

static void set_humidifier(uint8_t on)
{
    on ? ACT_ON(HUMIDIFIER) : ACT_OFF(HUMIDIFIER);
}

static void set_fluorescent(uint8_t on)
{
    on ? ACT_ON(FLUORESCENT) : ACT_OFF(FLUORESCENT);
}

/*====================================================================*/
/* 舵机控制 (TIM1_CH1 PWM 脉宽调整)                                    */
/*====================================================================*/
extern TIM_HandleTypeDef htim1;

static void set_servo_pulse(uint16_t pulse_us)
{
    /* TIM1 频率 = 1MHz (8MHz / 8) */
    /* 脉宽 = pulse_us, 范围 500~2000 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

/*====================================================================*/
/* API                                                                 */
/*====================================================================*/

void AUTO_Control_Init(void)
{
    g_delivery_active = 0;
    g_delivery_target = 0;
    s_delivery_phase = 0;
}

void AUTO_Control_Run(void)
{
    if (g_manual_active)
        return;  // 手动模式暂停自动输出

    /* ---- 1. 喂水系统 ---- */
    read_water_levels();
    for (int i = 0; i < 3; i++) {
        if (g_adc_water[i] < ADC_WATER_LOW_THRESHOLD) {
            set_valve(i, 1);
            set_water_led(i, 1);
        } else {
            set_valve(i, 0);
            set_water_led(i, 0);
        }
    }

    /* ---- 2. 喂料配送系统 ---- */
    if (g_delivery_active) {
        /* 配送状态机 */
        uint32_t now = TaskScheduler_GetTick();

        switch (s_delivery_phase) {
        case 0: /* 去程: 检查到位传感器 */
            if (g_delivery_target == 0 && IS_ARRIVED(DELIVERY_SENSOR1)) {
                STEPPER_Stop(&g_x_motor);
                STEPPER_Stop(&g_z_motor);
                s_delivery_phase = 1;
                s_phase_tick = now;
            } else if (g_delivery_target == 1 && IS_ARRIVED(DELIVERY_SENSOR2)) {
                STEPPER_Stop(&g_x_motor);
                STEPPER_Stop(&g_z_motor);
                s_delivery_phase = 1;
                s_phase_tick = now;
            } else if (g_delivery_target == 2 && IS_ARRIVED(DELIVERY_SENSOR3)) {
                STEPPER_Stop(&g_x_motor);
                STEPPER_Stop(&g_z_motor);
                s_delivery_phase = 1;
                s_phase_tick = now;
            }
            /* 超时 90s */
            if ((now - s_phase_tick) > DELIVERY_TIMEOUT_MS) {
                s_delivery_phase = 2;
                s_phase_tick = now;
                /* 掉头回程 */
                STEPPER_SetDirection(&g_x_motor, 0);
                STEPPER_SetDirection(&g_z_motor, 0);
                STEPPER_Start(&g_x_motor, 0, 0);
                STEPPER_Start(&g_z_motor, 0, 0);
            }
            break;

        case 1: /* 投料: 停留 1s */
            if ((now - s_phase_tick) > DELIVERY_DWELL_MS) {
                /* 模拟投料: 设置料位为 500g */
                HX711_SetSimGrams(g_delivery_target, 500.0f);
                /* 掉头回程 */
                STEPPER_SetDirection(&g_x_motor, 0);
                STEPPER_SetDirection(&g_z_motor, 0);
                STEPPER_Start(&g_x_motor, 0, 0);
                STEPPER_Start(&g_z_motor, 0, 0);
                s_delivery_phase = 2;
                s_phase_tick = now;
            }
            break;

        case 2: /* 回程: 3s 后停止 */
            if ((now - s_phase_tick) > DELIVERY_RETURN_MS) {
                STEPPER_Stop(&g_x_motor);
                STEPPER_Stop(&g_z_motor);
                g_delivery_active = 0;
                s_delivery_phase = 0;
                set_feed_led(g_delivery_target, 0);
            }
            break;
        }
    } else {
        /* 非配送中: 检查料位 */
        for (int i = 0; i < 3; i++) {
            float feed = HX711_GetGrams(i);
            if (feed < HX711_FEED_LOW_THRESHOLD) {
                set_feed_led(i, 1);

                /* 启动配送 */
                g_delivery_active = 1;
                g_delivery_target = i;
                s_delivery_phase = 0;
                s_phase_tick = TaskScheduler_GetTick();

                /* X/Y 电机正转去程 */
                STEPPER_SetDirection(&g_x_motor, 1);
                STEPPER_SetDirection(&g_z_motor, 1);
                STEPPER_Start(&g_x_motor, 1, 0);   // 连续
                STEPPER_Start(&g_z_motor, 1, 0);

                break;  // 一次只处理一个
            }
        }
    }

    /* ---- 3. 温度控制 ---- */
    int16_t temp = VIRTUAL_ENV_GetTemperature();
    if (temp > TEMP_HIGH_THRESHOLD) {
        set_cooling_fan(1);
        set_incandescent(0);
    } else if (temp < TEMP_LOW_THRESHOLD) {
        set_incandescent(1);
        set_cooling_fan(0);
    } else {
        set_cooling_fan(0);
        set_incandescent(0);
    }

    /* ---- 4. 湿度控制 ---- */
    int16_t humi = VIRTUAL_ENV_GetHumidity();
    if (humi > HUMI_HIGH_THRESHOLD) {
        set_vent_fan(1);
        set_humidifier(0);
    } else if (humi < HUMI_LOW_THRESHOLD) {
        set_humidifier(1);
        set_vent_fan(0);
    } else {
        set_vent_fan(0);
        set_humidifier(0);
    }

    /* ---- 5. 通风窗舵机 ---- */
    if (VirtualRTC_IsBetween(6, 0, 20, 0)) {
        set_servo_pulse(2000);  // 2.0ms = 开窗
    } else {
        set_servo_pulse(500);   // 0.5ms = 关窗
    }

    /* ---- 6. 光照控制 ---- */
    uint8_t is_night = (VirtualRTC_GetHour() >= 20 || VirtualRTC_GetHour() < 8);
    if (is_night || IS_LIGHT_DARK) {
        set_fluorescent(1);
    } else {
        set_fluorescent(0);
    }
}
