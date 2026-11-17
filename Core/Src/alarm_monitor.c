#include "alarm_monitor.h"
#include "project_config.h"
#include "task_scheduler.h"

/*====================================================================*/
/* 报警监控 (超时检测)                                                  */
/*====================================================================*/

typedef struct {
    uint32_t  start_tick;     // 0 = 条件未激活
    uint8_t   active;         // 1 = 已报警
} AlarmTimer_t;

static AlarmTimer_t s_water_alarm[3];
static AlarmTimer_t s_delivery_alarm;
static AlarmTimer_t s_feed_alarm;
static AlarmTimer_t s_temp_high;
static AlarmTimer_t s_temp_low;
static AlarmTimer_t s_humi_high;
static AlarmTimer_t s_humi_low;

/* 外部变量 */
extern uint16_t g_adc_water[3];
extern volatile uint8_t g_delivery_active;
extern volatile uint8_t g_delivery_target;
extern int16_t VIRTUAL_ENV_GetTemperature(void);
extern int16_t VIRTUAL_ENV_GetHumidity(void);
extern float HX711_GetSimGrams(uint8_t channel);

/*====================================================================*/
/* 报警超时处理                                                        */
/*====================================================================*/

static void check_alarm(AlarmTimer_t *alarm, uint8_t condition,
                         GPIO_TypeDef *port, uint16_t pin, uint32_t timeout_ms)
{
    uint32_t now = TaskScheduler_GetTick();

    if (condition) {
        if (alarm->start_tick == 0) {
            alarm->start_tick = now;
        } else if (!alarm->active && (now - alarm->start_tick) >= timeout_ms) {
            alarm->active = 1;
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        }
    } else {
        if (alarm->active) {
            alarm->active = 0;
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        }
        alarm->start_tick = 0;
    }
}

/*====================================================================*/
/* API                                                                 */
/*====================================================================*/

void ALARM_Init(void)
{
    int i;
    for (i = 0; i < 3; i++) {
        s_water_alarm[i].start_tick = 0;
        s_water_alarm[i].active = 0;
    }
    s_delivery_alarm.start_tick = 0;
    s_delivery_alarm.active = 0;
    s_feed_alarm.start_tick = 0;
    s_feed_alarm.active = 0;
    s_temp_high.start_tick = 0;
    s_temp_high.active = 0;
    s_temp_low.start_tick = 0;
    s_temp_low.active = 0;
    s_humi_high.start_tick = 0;
    s_humi_high.active = 0;
    s_humi_low.start_tick = 0;
    s_humi_low.active = 0;
}

void ALARM_Monitor(void)
{
    uint32_t now = TaskScheduler_GetTick();
    (void)now;

    /* 1. 供水故障: 水位低超 30s */
    for (int i = 0; i < 3; i++) {
        uint8_t low = (g_adc_water[i] < ADC_WATER_LOW_THRESHOLD);
        GPIO_TypeDef *port;
        uint16_t pin;
        if (i == 0) { port = WATER_FAULT_PORT; pin = WATER_FAULT_PIN; }
        else if (i == 1) { port = WATER_FAULT_PORT; pin = WATER_FAULT_PIN; }   // 共享
        else { port = WATER_FAULT_PORT; pin = WATER_FAULT_PIN; }
        check_alarm(&s_water_alarm[i], low, port, pin, ALARM_TIMEOUT_MS);
    }

    /* 2. 配送故障: 配送超 90s (报警超时) */
    /*   配送去程超时由 auto_control 处理, 这里检测超 90s 仍不到位 */
    check_alarm(&s_delivery_alarm, g_delivery_active,
                DELIVERY_FAULT_PORT, DELIVERY_FAULT_PIN, DELIVERY_TIMEOUT_MS);

    /* 3. 供料故障: 到位后料仍低超 30s (简化: 配送中且料仍低) */
    uint8_t feed_still_low = 0;
    if (g_delivery_active) {
        float grams = HX711_GetSimGrams(g_delivery_target);
        if (grams < HX711_FEED_LOW_THRESHOLD)
            feed_still_low = 1;
    }
    check_alarm(&s_feed_alarm, feed_still_low,
                FEED_FAULT_PORT, FEED_FAULT_PIN, ALARM_TIMEOUT_MS);

    /* 4. 高温报警 (虚拟数据在正常范围不会触发) */
    int16_t temp = VIRTUAL_ENV_GetTemperature();
    check_alarm(&s_temp_high, (temp > TEMP_HIGH_THRESHOLD),
                TEMP_HIGH_WARN_PORT, TEMP_HIGH_WARN_PIN, ALARM_TIMEOUT_MS);

    /* 5. 低温报警 */
    check_alarm(&s_temp_low, (temp < TEMP_LOW_THRESHOLD),
                TEMP_LOW_WARN_PORT, TEMP_LOW_WARN_PIN, ALARM_TIMEOUT_MS);

    /* 6. 高湿报警 */
    int16_t humi = VIRTUAL_ENV_GetHumidity();
    check_alarm(&s_humi_high, (humi > HUMI_HIGH_THRESHOLD),
                HUMI_HIGH_WARN_PORT, HUMI_HIGH_WARN_PIN, ALARM_TIMEOUT_MS);

    /* 7. 低湿报警 */
    check_alarm(&s_humi_low, (humi < HUMI_LOW_THRESHOLD),
                HUMI_LOW_WARN_PORT, HUMI_LOW_WARN_PIN, ALARM_TIMEOUT_MS);
}
