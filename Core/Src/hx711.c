#include "hx711.h"
#include "project_config.h"

/*====================================================================*/
/* HX711 位脉冲驱动 (3通道, 共享SCK)                                   */
/*====================================================================*/

/* 模拟料位值 (用于虚拟测试) */
static float s_sim_grams[HX711_CHANNEL_COUNT] = {0.0f, 0.0f, 0.0f};

/* SCK 脉冲延迟 (8MHz 下约 1.5μs HIGH + 1.5μs LOW) */
static void pulse_delay(void)
{
    volatile int i;
    for (i = 0; i < 6; i++);   // ~12 NOP ≈ 1.5μs @ 8MHz
}

void HX711_Init(void)
{
    /* SCK 初始为 LOW */
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
}

int32_t HX711_ReadChannel(uint8_t channel)
{
    uint32_t value = 0;
    int i;

    if (channel >= HX711_CHANNEL_COUNT)
        return 0;

    /* 选择对应的数据引脚 */
    GPIO_TypeDef *data_port;
    uint16_t data_pin_num;

    switch (channel) {
        case 0: data_port = HX711_DATA1_PORT; data_pin_num = HX711_DATA1_PIN; break;
        case 1: data_port = HX711_DATA2_PORT; data_pin_num = HX711_DATA2_PIN; break;
        case 2: data_port = HX711_DATA3_PORT; data_pin_num = HX711_DATA3_PIN; break;
        default: return 0;
    }

    /* 等待 DOUT 拉低 (HX711 就绪) */
    int timeout = 10000;
    while (HAL_GPIO_ReadPin(data_port, data_pin_num) == GPIO_PIN_SET) {
        if (--timeout <= 0)
            return 0;   // 超时
    }

    /* 读取 24 位数据 (MSB 先) */
    for (i = 0; i < 24; i++) {
        /* SCK HIGH */
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
        pulse_delay();

        value <<= 1;
        if (HAL_GPIO_ReadPin(data_port, data_pin_num) == GPIO_PIN_SET)
            value |= 1;

        /* SCK LOW */
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
        pulse_delay();
    }

    /* 第 25 个脉冲: 通道 A, 增益 128 */
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    pulse_delay();
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    pulse_delay();

    /* 24位有符号转换 (二进制补码) */
    if (value & 0x800000)
        value |= 0xFF000000;

    return (int32_t)value;
}

float HX711_GetGrams(uint8_t channel)
{
    if (channel >= HX711_CHANNEL_COUNT)
        return 0.0f;

    int32_t raw = HX711_ReadChannel(channel);
    if (raw == 0)
        return s_sim_grams[channel];  // 读取失败时返回模拟值

    /* 简化的转换 (实际需标定: grams = (raw - offset) / scale) */
    int32_t offset = 800000;   // 典型偏移
    float scale = 400.0f;       // 典型比例
    float grams = (float)(raw - offset) / scale;

    if (grams < 0.0f) grams = 0.0f;
    return grams;
}

float HX711_GetSimGrams(uint8_t channel)
{
    if (channel >= HX711_CHANNEL_COUNT)
        return 0.0f;
    return s_sim_grams[channel];
}

void HX711_SetSimGrams(uint8_t channel, float grams)
{
    if (channel >= HX711_CHANNEL_COUNT)
        return;
    s_sim_grams[channel] = grams;
}
