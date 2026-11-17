#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

/* SSD1306 OLED 128x64 I2C */

#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
#define SSD1306_PAGES           8       // 每页 8 像素高
#define SSD1306_I2C_ADDR        0x78    // 7-bit 0x3C << 1

/* 逻辑页定义 (4页, 每页4行) */
#define OLED_PAGE_TEMP          0   // 温湿度
#define OLED_PAGE_WATER         1   // 水位
#define OLED_PAGE_FEED          2   // 料位
#define OLED_PAGE_SYSTEM        3   // 系统状态
#define OLED_PAGE_COUNT         4

extern volatile uint8_t g_oled_current_page;

void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_Display(void);          // 刷缓冲到 OLED

/* 绘制文本 (直接写缓冲) */
void SSD1306_DrawChar(uint8_t x, uint8_t y, char c);
void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str);
void SSD1306_DrawPage(uint8_t page);  // 绘制指定逻辑页

void OLED_NextPage(void);
void OLED_PrevPage(void);
void OLED_UpdateDisplay(void);       // 绘制当前页 + 刷屏

#endif
