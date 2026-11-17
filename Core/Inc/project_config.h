#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include "main.h"

/*====================================================================*/
/* 系统时钟                                                        */
/*====================================================================*/
#define SYS_CLOCK_HZ            8000000

/*====================================================================*/
/* ADC 阈值                                                          */
/*====================================================================*/
#define ADC_WATER_LOW_THRESHOLD     2048    // 12bit ADC, <2048 = 低水位
#define ADC_REF_VOLTAGE             3300    // mV

/*====================================================================*/
/* HX711 阈值                                                        */
/*====================================================================*/
#define HX711_FEED_LOW_THRESHOLD    200     // 克, <200g = 低料位

/*====================================================================*/
/* 温湿度阈值 (x10)                                                   */
/*====================================================================*/
#define TEMP_HIGH_THRESHOLD         320     // > 32.0℃ -> 降温风扇ON
#define TEMP_LOW_THRESHOLD          200     // < 20.0℃ -> 白炽灯ON
#define HUMI_HIGH_THRESHOLD         750     // > 75.0% -> 通风风扇ON
#define HUMI_LOW_THRESHOLD          350     // < 35.0% -> 加湿阀ON

/*====================================================================*/
/* 定时参数 (ms)                                                      */
/*====================================================================*/
#define ALARM_TIMEOUT_MS            30000   // 报警超时
#define DELIVERY_TIMEOUT_MS         90000   // 配送去程超时
#define DELIVERY_DWELL_MS           1000    // 投料停留
#define DELIVERY_RETURN_MS          3000    // 回程时间

/*====================================================================*/
/* 步进电机参数                                                      */
/*====================================================================*/
#define STEPPER_PULSE_HZ            2000    // 2000 Hz
#define STEPPER_RUN_HZ             600     // 实际运行频率(实测≤600Hz不失步)
#define STEPPER_PULSES_PER_MM       100     // 100 脉冲/mm
#define STEPPER_SPEED_MMS           20      // 20 mm/s
#define TRAVEL_DISTANCE_MM          800     // 800mm
#define TRAVEL_PULSES               (TRAVEL_DISTANCE_MM * STEPPER_PULSES_PER_MM)  // 80000

/*====================================================================*/
/* Modbus                                                             */
/*====================================================================*/
#define MODBUS_REG_COUNT            35
#define MODBUS_BAUDRATE             9600
#define MODBUS_SLAVE_ID             1

/*====================================================================*/
/* 任务周期 (ms)                                                      */
/*====================================================================*/
#define TASK_MODBUS_PERIOD_MS       50
#define TASK_MANUAL_PERIOD_MS       10
#define TASK_AUTO_PERIOD_MS         200
#define TASK_ALARM_PERIOD_MS        500
#define TASK_OLED_PERIOD_MS         500

/*====================================================================*/
/* IO 引脚宏定义                                                      */
/*====================================================================*/

/* ---- Port A ---- */
// PA0: X轴电机脉冲 (TIM2_CH1)
#define X_PULSE_PORT                GPIOA
#define X_PULSE_PIN                 GPIO_PIN_0
#define X_DIR_PORT                  GPIOA
#define X_DIR_PIN                   GPIO_PIN_1
// PA2: USART2_TX
// PA3: USART2_RX
// PA4: 电磁阀1
#define VALVE1_PORT                 GPIOA
#define VALVE1_PIN                  GPIO_PIN_4
// PA5: 电磁阀2
#define VALVE2_PORT                 GPIOA
#define VALVE2_PIN                  GPIO_PIN_5
// PA6: Z轴电机脉冲 (TIM3_CH1)
#define Z_PULSE_PORT                GPIOA
#define Z_PULSE_PIN                 GPIO_PIN_6
#define Z_DIR_PORT                  GPIOA
#define Z_DIR_PIN                   GPIO_PIN_7
// PA8: 通风窗舵机 (TIM1_CH1)
#define SERVO_PORT                  GPIOA
#define SERVO_PIN                   GPIO_PIN_8
// PA9: USART1_TX
// PA10: USART1_RX
// PA11: 配送到位传感器1
#define DELIVERY_SENSOR1_PORT       GPIOA
#define DELIVERY_SENSOR1_PIN        GPIO_PIN_11
// PA12: 配送到位传感器2
#define DELIVERY_SENSOR2_PORT       GPIOA
#define DELIVERY_SENSOR2_PIN        GPIO_PIN_12
// PA15: 配送到位传感器3
#define DELIVERY_SENSOR3_PORT       GPIOA
#define DELIVERY_SENSOR3_PIN        GPIO_PIN_15

/* ---- Port B ---- */
// PB0: 日光灯
#define FLUORESCENT_PORT            GPIOB
#define FLUORESCENT_PIN             GPIO_PIN_0
// PB1: 白炽灯
#define INCANDESCENT_PORT           GPIOB
#define INCANDESCENT_PIN            GPIO_PIN_1
// PB2: 低湿报警灯
#define HUMI_LOW_WARN_PORT          GPIOB
#define HUMI_LOW_WARN_PIN           GPIO_PIN_2
// PB3: HX711 SCK (shared)
#define HX711_SCK_PORT              GPIOB
#define HX711_SCK_PIN               GPIO_PIN_3
// PB4: HX711 数据1
#define HX711_DATA1_PORT            GPIOB
#define HX711_DATA1_PIN             GPIO_PIN_4
// PB5: HX711 数据3
#define HX711_DATA3_PORT            GPIOB
#define HX711_DATA3_PIN             GPIO_PIN_5
// PB6: OLED SCL (I2C1)
// PB7: OLED SDA (I2C1)
// PB8: 光敏传感器
#define LIGHT_SENSOR_PORT           GPIOB
#define LIGHT_SENSOR_PIN            GPIO_PIN_8
// PB9: 加湿电磁阀
#define HUMIDIFIER_PORT             GPIOB
#define HUMIDIFIER_PIN              GPIO_PIN_9
// PB11: 清洗电磁阀
#define CLEAN_VALVE_PORT            GPIOB
#define CLEAN_VALVE_PIN             GPIO_PIN_11
// PB12: 降温风扇 LED
#define COOLING_FAN_PORT            GPIOB
#define COOLING_FAN_PIN             GPIO_PIN_12
// PB13: 通风风扇 LED
#define VENT_FAN_PORT               GPIOB
#define VENT_FAN_PIN                GPIO_PIN_13
// PB15: HX711 数据2
#define HX711_DATA2_PORT            GPIOB
#define HX711_DATA2_PIN             GPIO_PIN_15

/* ---- Port C ---- */
// PC0: 水位传感器1 (ADC_IN10)
// PC1: 水位传感器2 (ADC_IN11)
// PC2: 水位传感器3 (ADC_IN12)
// PC3: 低水位提示灯1
#define WATER_LED1_PORT             GPIOC
#define WATER_LED1_PIN              GPIO_PIN_3
// PC4: ADC按键 (ADC_IN14)
// PC5: 低水位提示灯3
#define WATER_LED3_PORT             GPIOC
#define WATER_LED3_PIN              GPIO_PIN_5
// PC6: 供水故障报警灯
#define WATER_FAULT_PORT            GPIOC
#define WATER_FAULT_PIN             GPIO_PIN_6
// PC7: 低料位提示灯1
#define FEED_LED1_PORT              GPIOC
#define FEED_LED1_PIN               GPIO_PIN_7
// PC8: 低料位提示灯2
#define FEED_LED2_PORT              GPIOC
#define FEED_LED2_PIN               GPIO_PIN_8
// PC9: 低料位提示灯3
#define FEED_LED3_PORT              GPIOC
#define FEED_LED3_PIN               GPIO_PIN_9
// PC10: 供料故障报警灯
#define FEED_FAULT_PORT             GPIOC
#define FEED_FAULT_PIN              GPIO_PIN_10
// PC11: 配送故障报警灯
#define DELIVERY_FAULT_PORT         GPIOC
#define DELIVERY_FAULT_PIN          GPIO_PIN_11
// PC12: 高湿报警灯
#define HUMI_HIGH_WARN_PORT         GPIOC
#define HUMI_HIGH_WARN_PIN          GPIO_PIN_12
// PC13: 电磁阀3
#define VALVE3_PORT                 GPIOC
#define VALVE3_PIN                  GPIO_PIN_13
// PC14: 高温报警灯
#define TEMP_HIGH_WARN_PORT         GPIOC
#define TEMP_HIGH_WARN_PIN          GPIO_PIN_14
// PC15: 低温报警灯
#define TEMP_LOW_WARN_PORT          GPIOC
#define TEMP_LOW_WARN_PIN           GPIO_PIN_15

/* ---- Port D ---- */
// PD2: 低水位提示灯2
#define WATER_LED2_PORT             GPIOD
#define WATER_LED2_PIN              GPIO_PIN_2

/*====================================================================*/
/* 执行器控制宏                                                       */
/*====================================================================*/
#define ACT_ON(x)       HAL_GPIO_WritePin(x##_PORT, x##_PIN, GPIO_PIN_SET)
#define ACT_OFF(x)      HAL_GPIO_WritePin(x##_PORT, x##_PIN, GPIO_PIN_RESET)
#define ACT_TOGGLE(x)   HAL_GPIO_TogglePin(x##_PORT, x##_PIN)
#define ACT_READ(x)     HAL_GPIO_ReadPin(x##_PORT, x##_PIN)

/* 传感器到位判定 (LOW = 到位) */
#define IS_ARRIVED(x)   (HAL_GPIO_ReadPin(x##_PORT, x##_PIN) == GPIO_PIN_RESET)

/* 光敏传感器 (HIGH = 亮, LOW = 暗) */
#define IS_LIGHT_BRIGHT (HAL_GPIO_ReadPin(LIGHT_SENSOR_PORT, LIGHT_SENSOR_PIN) == GPIO_PIN_SET)
#define IS_LIGHT_DARK   (HAL_GPIO_ReadPin(LIGHT_SENSOR_PORT, LIGHT_SENSOR_PIN) == GPIO_PIN_RESET)

/*====================================================================*/
/* 全局标志                                                        */
/*====================================================================*/
extern volatile uint8_t g_manual_active;
extern volatile uint8_t g_delivery_active;
extern volatile uint8_t g_delivery_target;    // 0,1,2

#endif /* PROJECT_CONFIG_H */
