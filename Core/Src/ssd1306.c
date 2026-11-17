#include "ssd1306.h"
#include "main.h"
#include "project_config.h"
#include "hx711.h"
#include <string.h>
#include <stdio.h>

/*====================================================================*/
/* SSD1306 OLED 128x64 I2C 驱动 (标准 5x7 ASCII 字库)                  */
/*====================================================================*/

extern I2C_HandleTypeDef hi2c1;

extern volatile uint8_t g_manual_active;
extern volatile uint8_t g_delivery_active;
extern uint8_t g_water_valve[3];
extern uint16_t g_adc_water[3];
extern int16_t VIRTUAL_ENV_GetTemperature(void);
extern int16_t VIRTUAL_ENV_GetHumidity(void);
extern uint8_t VirtualRTC_GetHour(void);
extern uint8_t VirtualRTC_GetMinute(void);

/* 帧缓冲 (128 x 64 / 8 = 1024 字节) */
static uint8_t s_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* 当前显示页 */
volatile uint8_t g_oled_current_page = 0;

/* 标准 5x7 ASCII 字库 (每字符5列, 列式, bit0在顶部), 从 0x20 空格开始 */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 0x20 空格
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x05,0x03,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00}, // )
    {0x14,0x08,0x3E,0x08,0x14}, // *
    {0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x50,0x30,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00}, // ;
    {0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14}, // =
    {0x00,0x41,0x22,0x14,0x08}, // >
    {0x02,0x01,0x51,0x09,0x06}, // ?
    {0x32,0x49,0x79,0x41,0x3E}, // @
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x07,0x08,0x70,0x08,0x07}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
};

/* 字库支持范围: 0x20 ~ 0x5A (空格到Z) */
#define FONT_FIRST  0x20
#define FONT_LAST   0x5A

/*====================================================================*/
/* I2C 写命令/数据                                                     */
/*====================================================================*/
static void write_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data, 2, 50);
}

/*====================================================================*/
/* SSD1306 API                                                         */
/*====================================================================*/

void SSD1306_Init(void)
{
    write_cmd(0xAE); write_cmd(0xD5); write_cmd(0x80);
    write_cmd(0xA8); write_cmd(0x3F); write_cmd(0xD3); write_cmd(0x00);
    write_cmd(0x40); write_cmd(0x8D); write_cmd(0x14);
    write_cmd(0x20); write_cmd(0x00); write_cmd(0xA1); write_cmd(0xC8);
    write_cmd(0xDA); write_cmd(0x12); write_cmd(0x81); write_cmd(0xCF);
    write_cmd(0xD9); write_cmd(0xF1); write_cmd(0xDB); write_cmd(0x40);
    write_cmd(0xA4); write_cmd(0xA6); write_cmd(0x2E); write_cmd(0xAF);

    SSD1306_Clear();
    SSD1306_Display();
}

void SSD1306_Clear(void)
{
    memset(s_buffer, 0x00, sizeof(s_buffer));
}

void SSD1306_Display(void)
{
    uint8_t buf[1 + SSD1306_WIDTH];   // 控制字节 + 一页(128字节)
    write_cmd(0x21); write_cmd(0); write_cmd(127);
    write_cmd(0x22); write_cmd(0); write_cmd(7);

    /* 分页批量发送: 8次事务(每次129字节), 而非1024次单字节, 避免I2C阻塞卡死 */
    for (int page = 0; page < 8; page++) {
        buf[0] = 0x40;   // 数据模式
        memcpy(&buf[1], &s_buffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, buf, 1 + SSD1306_WIDTH, 50);
    }
}

/* 绘制字符: 5x7 字体, 占 6 像素宽 x 1 页(8像素高) */
void SSD1306_DrawChar(uint8_t x, uint8_t y, char c)
{
    if (c < FONT_FIRST || c > FONT_LAST) c = ' ';
    const uint8_t *glyph = font5x7[c - FONT_FIRST];
    uint8_t page = y / 8;

    for (int col = 0; col < 5; col++) {
        if (x + col < SSD1306_WIDTH)
            s_buffer[page * SSD1306_WIDTH + x + col] = glyph[col];
    }
    /* 第6列留空作字符间隔 */
    if (x + 5 < SSD1306_WIDTH)
        s_buffer[page * SSD1306_WIDTH + x + 5] = 0x00;
}

void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str)
{
    while (*str && x < SSD1306_WIDTH) {
        SSD1306_DrawChar(x, y, *str);
        x += 6;
        str++;
    }
}

void SSD1306_DrawPage(uint8_t page)
{
    char line[22];

    switch (page) {
    case 0: /* 温湿度 + 时间 */
    {
        int16_t temp = VIRTUAL_ENV_GetTemperature();
        int16_t humi = VIRTUAL_ENV_GetHumidity();
        snprintf(line, sizeof(line), "TEMP:%d.%dC",
                 temp/10, temp%10);
        SSD1306_DrawString(0, 0, line);
        snprintf(line, sizeof(line), "HUM:%d.%d%%",
                 humi/10, humi%10);
        SSD1306_DrawString(0, 16, line);
        snprintf(line, sizeof(line), "TIME:%02d:%02d",
                 VirtualRTC_GetHour(), VirtualRTC_GetMinute());
        SSD1306_DrawString(0, 32, line);
        break;
    }
    case 1: /* 水位 + 电磁阀 */
        SSD1306_DrawString(0, 0,  "WATER MONITOR");
        snprintf(line, sizeof(line), "W1:%d W2:%d",
                 g_adc_water[0], g_adc_water[1]);
        SSD1306_DrawString(0, 16, line);
        snprintf(line, sizeof(line), "W3:%d VLV:%d%d%d",
                 g_adc_water[2],
                 g_water_valve[0], g_water_valve[1], g_water_valve[2]);
        SSD1306_DrawString(0, 32, line);
        break;
    case 2: /* 料位 */
        SSD1306_DrawString(0, 0,  "FEED MONITOR");
        snprintf(line, sizeof(line), "F1:%dG F2:%dG",
                 (int)HX711_GetSimGrams(0), (int)HX711_GetSimGrams(1));
        SSD1306_DrawString(0, 16, line);
        snprintf(line, sizeof(line), "F3:%dG",
                 (int)HX711_GetSimGrams(2));
        SSD1306_DrawString(0, 32, line);
        break;
    case 3: /* 系统状态 */
        SSD1306_DrawString(0, 0, "SYSTEM STATUS");
        snprintf(line, sizeof(line), "MODE:%s",
                 g_manual_active ? "MANUAL" : "AUTO");
        SSD1306_DrawString(0, 16, line);
        snprintf(line, sizeof(line), "DEL:%s",
                 g_delivery_active ? "ACTIVE" : "IDLE");
        SSD1306_DrawString(0, 32, line);
        break;
    }
}

void OLED_NextPage(void)
{
    g_oled_current_page++;
    if (g_oled_current_page >= OLED_PAGE_COUNT)
        g_oled_current_page = 0;
    /* 仅改页号, 不立即刷屏 (由 Task_OLEDDisplay 每500ms统一刷新,
       防止快速连按翻页时大量 I2C 刷新累积阻塞卡死) */
}

void OLED_PrevPage(void)
{
    if (g_oled_current_page == 0)
        g_oled_current_page = OLED_PAGE_COUNT - 1;
    else
        g_oled_current_page--;
    /* 仅改页号, 不立即刷屏 */
}

void OLED_UpdateDisplay(void)
{
    SSD1306_Clear();
    SSD1306_DrawPage(g_oled_current_page);
    SSD1306_Display();
}
