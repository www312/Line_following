/**
 * 0.96" 128x64 OLED — I2C SSD1306
 * API 风格仿桌面参考 oled.c（NewFrame / ShowFrame / SetPixel / 线框 / ASCII）
 * 使用前请在 CubeMX 中配置 I2C，并保证存在 hi2c1（或改 oled096.c 中 OLED096_I2C 宏）
 */
#ifndef OLED096_H
#define OLED096_H

#include <stdint.h>
#include <string.h>
#include "stm32f1xx_hal.h"

/* 常见 SSD1306 模块为 0x3C（HAL 传 8 位地址 0x78）；若为 0x3D 则在包含本头文件前定义 0x3Du */
#ifndef OLED096_I2C_ADDR_7BIT
#define OLED096_I2C_ADDR_7BIT (0x3Cu)
#endif

/**
 * SSD1306 硬件扫描方向（须与本板 OLED 模块一致，勿随意改，否则可能整屏不亮）：
 *   本工程模块已验证：SEG=0xA1，COM=0xC8
 * 画面上下/左右颠倒时，改 OLED096_ROTATE_180（软件翻转），不要只改 SEG/COM。
 */
#ifndef OLED096_SEG_REMAP
#define OLED096_SEG_REMAP (0xA1u)
#endif
#ifndef OLED096_COM_SCAN
#define OLED096_COM_SCAN (0xC8u)
#endif
/** 1=在 SetPixel 中对坐标做 180° 翻转（推荐，不影响 I2C 初始化） */
#ifndef OLED096_ROTATE_180
#define OLED096_ROTATE_180 1
#endif

typedef enum {
  OLED096_COLOR_NORMAL = 0,
  OLED096_COLOR_REVERSED
} OLED096_ColorMode;

typedef struct {
  uint8_t h;
  uint8_t w;
  const uint8_t *chars;
} OLED096_ASCIIFont;

extern const OLED096_ASCIIFont oled096_afont8x6;

void OLED096_Init(void);
void OLED096_DisplayOn(void);
void OLED096_DisplayOff(void);

/** 将 128×64 显存（GRAM）全部清为 0（黑），每帧绘图前应调用一次再画 */
void OLED096_NewFrame(void);

/** 分步刷新：开始一帧传输（重置页计数器） */
void OLED096_ShowFrameStart(void);
/** 分步刷新：传输下一页，返回 1=帧完成，0=仍有页待传 */
uint8_t OLED096_ShowFrameStep(void);

/** 一次性刷新全帧（阻塞较大，推荐用 Start/Step 分步） */
void OLED096_ShowFrame(void);
void OLED096_SetPixel(uint8_t x, uint8_t y, OLED096_ColorMode color);

void OLED096_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED096_ColorMode color);
void OLED096_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED096_ColorMode color);
void OLED096_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED096_ColorMode color);

void OLED096_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const OLED096_ASCIIFont *font, OLED096_ColorMode color);
void OLED096_PrintASCIIString(uint8_t x, uint8_t y, char *str, const OLED096_ASCIIFont *font, OLED096_ColorMode color);

#endif
