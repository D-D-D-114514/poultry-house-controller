#include "modbus_data.h"
#include "modbus_rtu.h"
#include "project_config.h"
#include "virtual_env.h"
#include "hx711.h"
#include "stepper.h"

/*====================================================================*/
/* Modbus 寄存器 ↔ 系统状态同步                                       */
/*====================================================================*/

/* 外部变量 */
extern uint16_t g_adc_water[3];
extern volatile uint8_t g_delivery_active;
extern volatile uint8_t g_delivery_target;
extern volatile uint8_t g_manual_active;
extern uint8_t g_water_valve[3];

/*====================================================================*/
/* 从传感器/状态更新 Modbus 寄存器 (每 50ms)                           */
/*====================================================================*/

void MODBUS_DATA_SyncRegisters(void)
{
    /* 40001 (0): TEMP - 温度 ×10 */
    int16_t temp = VIRTUAL_ENV_GetTemperature();
    MODBUS_SetRegister(0, (uint16_t)temp);

    /* 40002 (1): HUMI - 湿度 ×10 */
    int16_t humi = VIRTUAL_ENV_GetHumidity();
    MODBUS_SetRegister(1, (uint16_t)humi);

    /* 40003 (2): LIGHT - 光照 0=亮 1=暗 */
    MODBUS_SetRegister(2, IS_LIGHT_DARK ? 1 : 0);

    /* 40004-40006 (3-5): 水位低标志 */
    for (int i = 0; i < 3; i++) {
        MODBUS_SetRegister(3 + i, (g_adc_water[i] < ADC_WATER_LOW_THRESHOLD) ? 1 : 0);
    }

    /* 40007-40009 (6-8): 电磁阀状态 */
    extern uint8_t g_water_valve[3];
    for (int i = 0; i < 3; i++) {
        MODBUS_SetRegister(6 + i, g_water_valve[i]);
    }

    /* 40010 (9): WATER_WARNING */
    /* 对齐旧程序 sys.water_warning 语义: 有故障(任一水位低)->1, 正常->0 */
    uint8_t any_low = 0;
    for (int i = 0; i < 3; i++) {
        if (g_adc_water[i] < ADC_WATER_LOW_THRESHOLD) {
            any_low = 1;
            break;
        }
    }
    MODBUS_SetRegister(9, any_low ? 1 : 0);  // 1=报警 0=正常 (同旧程序)

    /* 40011-40013 (10-12): 料位低 */
    for (int i = 0; i < 3; i++) {
        float grams = HX711_GetSimGrams(i);
        MODBUS_SetRegister(10 + i, (grams < HX711_FEED_LOW_THRESHOLD) ? 1 : 0);
    }

    /* 40014 (13): 电机 X 状态 */
    MODBUS_SetRegister(13, STEPPER_IsRunning(&g_x_motor) ? 1 : 0);

    /* 40015 (14): 电机 Z 状态 */
    MODBUS_SetRegister(14, STEPPER_IsRunning(&g_z_motor) ? 1 : 0);

    /* 40016-40018 (15-17): 配送到位 */
    MODBUS_SetRegister(15, IS_ARRIVED(DELIVERY_SENSOR1) ? 1 : 0);
    MODBUS_SetRegister(16, IS_ARRIVED(DELIVERY_SENSOR2) ? 1 : 0);
    MODBUS_SetRegister(17, IS_ARRIVED(DELIVERY_SENSOR3) ? 1 : 0);

    /* 40019 (18): Hopper (已废除, 保留) */
    MODBUS_SetRegister(18, 0);

    /* 40020 (19): 配送故障 */
    MODBUS_SetRegister(19, g_delivery_active ? 1 : 0);

    /* 40021 (20): 供料故障 (到位后料仍低) */
    uint8_t feed_fault = 0;
    if (g_delivery_active) {
        float grams = HX711_GetSimGrams(g_delivery_target);
        if (grams < HX711_FEED_LOW_THRESHOLD)
            feed_fault = 1;
    }
    MODBUS_SetRegister(20, feed_fault);

    /* 40022 (21): 通风窗 (舵机状态简化为 0=关 1=开) */
    extern uint8_t VirtualRTC_IsBetween(uint8_t, uint8_t, uint8_t, uint8_t);
    uint8_t vent_open = VirtualRTC_IsBetween(6, 0, 20, 0);
    MODBUS_SetRegister(21, vent_open ? 1 : 0);

    /* 40023 (22): 预留 */
    MODBUS_SetRegister(22, 0);

    /* 40024 (23): 通风风扇 */
    MODBUS_SetRegister(23, HAL_GPIO_ReadPin(VENT_FAN_PORT, VENT_FAN_PIN) == GPIO_PIN_SET ? 1 : 0);

    /* 40025 (24): 加湿阀 (由寄存器写入) */
    /* 保持当前值 (写入时已更新) */

    /* 40026 (25): 高湿报警 */
    int16_t humidity = VIRTUAL_ENV_GetHumidity();
    MODBUS_SetRegister(25, (humidity > HUMI_HIGH_THRESHOLD) ? 1 : 0);

    /* 40027 (26): 低湿报警 */
    MODBUS_SetRegister(26, (humidity < HUMI_LOW_THRESHOLD) ? 1 : 0);

    /* 40028 (27): 预留 (显式置0, 保证整块读取时该位干净) */
    MODBUS_SetRegister(27, 0);

    /* 40029 (28): 降温风扇 */
    MODBUS_SetRegister(28, HAL_GPIO_ReadPin(COOLING_FAN_PORT, COOLING_FAN_PIN) == GPIO_PIN_SET ? 1 : 0);

    /* 40030 (29): 白炽灯 */
    MODBUS_SetRegister(29, HAL_GPIO_ReadPin(INCANDESCENT_PORT, INCANDESCENT_PIN) == GPIO_PIN_SET ? 1 : 0);

    /* 40031 (30): 高温报警 */
    int16_t temperature = VIRTUAL_ENV_GetTemperature();
    MODBUS_SetRegister(30, (temperature > TEMP_HIGH_THRESHOLD) ? 1 : 0);

    /* 40032 (31): 低温报警 */
    MODBUS_SetRegister(31, (temperature < TEMP_LOW_THRESHOLD) ? 1 : 0);

    /* 40033 (32): 日光灯 */
    MODBUS_SetRegister(32, HAL_GPIO_ReadPin(FLUORESCENT_PORT, FLUORESCENT_PIN) == GPIO_PIN_SET ? 1 : 0);

    /* 40034 (33): 光照值 */
    MODBUS_SetRegister(33, IS_LIGHT_DARK ? 1 : 0);

    /* 40035 (34): 清洗(已废除) */
    MODBUS_SetRegister(34, 0);

    /* 40036-40040 (35-39): 预留 */
    for (int i = 35; i < MODBUS_REG_COUNT; i++) {
        MODBUS_SetRegister(i, 0);
    }
}

/*====================================================================*/
/* 从 Modbus 写入 → 应用到硬件                                          */
/*====================================================================*/

void MODBUS_DATA_ApplyWrites(void)
{
    /* 仅在寄存器值"变化"时才写 GPIO (边沿触发), 避免每50ms无条件
       覆盖手动/自动控制的阀门状态. 手动 toggle 直接改GPIO不动寄存器,
       此处看不到变化就不干涉 */
    static uint16_t prev6 = 0xFFFF, prev7 = 0xFFFF;
    static uint16_t prev8 = 0xFFFF, prev24 = 0xFFFF;

    /* 电磁阀1 (40007, reg6) */
    uint16_t v1 = MODBUS_GetRegister(6);
    if (v1 != prev6) {
        if (v1 == 1) { ACT_ON(VALVE1); g_water_valve[0] = 1; }
        else         { ACT_OFF(VALVE1); g_water_valve[0] = 0; }
        prev6 = v1;
    }

    /* 电磁阀2 (40008, reg7) */
    uint16_t v2 = MODBUS_GetRegister(7);
    if (v2 != prev7) {
        if (v2 == 1) { ACT_ON(VALVE2); g_water_valve[1] = 1; }
        else         { ACT_OFF(VALVE2); g_water_valve[1] = 0; }
        prev7 = v2;
    }

    /* 电磁阀3 (40009, reg8) */
    uint16_t v3 = MODBUS_GetRegister(8);
    if (v3 != prev8) {
        if (v3 == 1) { ACT_ON(VALVE3); g_water_valve[2] = 1; }
        else         { ACT_OFF(VALVE3); g_water_valve[2] = 0; }
        prev8 = v3;
    }

    /* 加湿阀 (40025, reg24) */
    uint16_t hv = MODBUS_GetRegister(24);
    if (hv != prev24) {
        if (hv == 1) ACT_ON(HUMIDIFIER);
        else         ACT_OFF(HUMIDIFIER);
        prev24 = hv;
    }
}
