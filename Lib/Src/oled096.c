/**
 * 0.96" 128x64 SSD1306 — I2C（HAL）
 * 显存布局与参考 oled.c 一致：GRAM[page][col]，SetPixel 位序 LSB=页内顶像素
 */
#include "oled096.h"
#include <stdlib.h>
#include <string.h>

/* 与 CubeMX 生成的句柄名一致；若使用 I2C2，改为 hi2c2 并在此处改 extern */
extern I2C_HandleTypeDef hi2c1;

#define OLED096_I2C_ADDR8 ((uint16_t)((OLED096_I2C_ADDR_7BIT) << 1u))

#define OLED096_PAGE 8u
#define OLED096_COL 128u
#define OLED096_ROW 64u

static uint8_t s_gram[OLED096_PAGE][OLED096_COL];

#define OLED096_I2C_TIMEOUT_MS 10u

/* 传输失败时 DeInit+Init 恢复总线，避免后续 ShowFrame 全部静默失败、屏冻在首帧 */
static HAL_StatusTypeDef oled096_i2c_tx(uint8_t *data, uint16_t len) {
  HAL_StatusTypeDef s = HAL_I2C_Master_Transmit(&hi2c1, OLED096_I2C_ADDR8, data, len, OLED096_I2C_TIMEOUT_MS);
  if (s != HAL_OK) {
    (void)HAL_I2C_DeInit(&hi2c1);
    (void)HAL_I2C_Init(&hi2c1);
    s = HAL_I2C_Master_Transmit(&hi2c1, OLED096_I2C_ADDR8, data, len, OLED096_I2C_TIMEOUT_MS);
  }
  return s;
}

static void oled096_wr_cmd(uint8_t cmd) {
  uint8_t buf[2] = {0x00u, cmd};
  (void)oled096_i2c_tx(buf, 2u);
}

static void oled096_wr_data(const uint8_t *data, uint16_t len) {
  /* 首字节 0x40：连续 GDDRAM 数据 */
  static uint8_t s_buf[1 + OLED096_COL];
  if (len > OLED096_COL) {
    len = OLED096_COL;
  }
  s_buf[0] = 0x40u;
  memcpy(s_buf + 1u, data, len);
  (void)oled096_i2c_tx(s_buf, (uint16_t)(len + 1u));
}

void OLED096_Init(void) {
  HAL_Delay(20);

  oled096_wr_cmd(0xAEu); /* display off */
  oled096_wr_cmd(0xD5u);
  oled096_wr_cmd(0x80u); /* clock */
  oled096_wr_cmd(0xA8u);
  oled096_wr_cmd(0x3Fu); /* multiplex 64 */
  oled096_wr_cmd(0xD3u);
  oled096_wr_cmd(0x00u); /* display offset */
  oled096_wr_cmd(0x40u); /* start line 0 */
  oled096_wr_cmd(0x8Du);
  oled096_wr_cmd(0x14u); /* charge pump on */
  oled096_wr_cmd(0x20u);
  oled096_wr_cmd(0x02u); /* page addressing */
  oled096_wr_cmd(OLED096_SEG_REMAP);
  oled096_wr_cmd(OLED096_COM_SCAN);
  oled096_wr_cmd(0xDAu);
  oled096_wr_cmd(0x12u); /* COM pins 128x64 */
  oled096_wr_cmd(0x81u);
  oled096_wr_cmd(0xCFu); /* contrast */
  oled096_wr_cmd(0xD9u);
  oled096_wr_cmd(0xF1u); /* pre-charge */
  oled096_wr_cmd(0xDBu);
  oled096_wr_cmd(0x40u); /* VCOMH */
  oled096_wr_cmd(0xA4u); /* resume to RAM */
  oled096_wr_cmd(0xA6u); /* normal (non-inverted) */

  OLED096_NewFrame();
  OLED096_ShowFrame();
  oled096_wr_cmd(0xAFu); /* display on */
}

void OLED096_DisplayOn(void) {
  oled096_wr_cmd(0x8Du);
  oled096_wr_cmd(0x14u);
  oled096_wr_cmd(0xAFu);
}

void OLED096_DisplayOff(void) {
  oled096_wr_cmd(0x8Du);
  oled096_wr_cmd(0x10u);
  oled096_wr_cmd(0xAEu);
}

void OLED096_NewFrame(void) { memset(s_gram, 0, sizeof(s_gram)); }

static uint8_t s_show_page;

void OLED096_ShowFrameStart(void)
{
	s_show_page = 0U;
}

uint8_t OLED096_ShowFrameStep(void)
{
	if (s_show_page >= OLED096_PAGE) {
		return 1U;
	}
	oled096_wr_cmd((uint8_t)(0xB0u + s_show_page));
	oled096_wr_cmd(0x00u);
	oled096_wr_cmd(0x10u);
	oled096_wr_data(s_gram[s_show_page], OLED096_COL);
	s_show_page++;
	return (s_show_page >= OLED096_PAGE) ? 1U : 0U;
}

void OLED096_ShowFrame(void) {
  for (uint8_t page = 0u; page < OLED096_PAGE; page++) {
    oled096_wr_cmd((uint8_t)(0xB0u + page));
    oled096_wr_cmd(0x00u); /* col 0 low */
    oled096_wr_cmd(0x10u); /* col 0 high */
    oled096_wr_data(s_gram[page], OLED096_COL);
  }
}

void OLED096_SetPixel(uint8_t x, uint8_t y, OLED096_ColorMode color) {
#if OLED096_ROTATE_180
  x = (uint8_t)(OLED096_COL - 1u - x);
  y = (uint8_t)(OLED096_ROW - 1u - y);
#endif
  if (x >= OLED096_COL || y >= OLED096_ROW) {
    return;
  }
  if (color == OLED096_COLOR_NORMAL) {
    s_gram[y / 8u][x] |= (uint8_t)(1u << (y % 8u));
  } else {
    s_gram[y / 8u][x] &= (uint8_t) ~(1u << (y % 8u));
  }
}

static void oled096_set_byte_fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, OLED096_ColorMode color) {
  uint8_t temp;
  if (page >= OLED096_PAGE || column >= OLED096_COL) {
    return;
  }
  if (color == OLED096_COLOR_REVERSED) {
    data = (uint8_t)~data;
  }
  temp = (uint8_t)(data | (uint8_t)(0xFFu << (end + 1u)) | (uint8_t)(0xFFu >> (8u - start)));
  s_gram[page][column] &= temp;
  temp = (uint8_t)(data & (uint8_t) ~(0xFFu << (end + 1u)) & (uint8_t) ~(0xFFu >> (8u - start)));
  s_gram[page][column] |= temp;
}

static void oled096_set_bits_fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len, OLED096_ColorMode color) {
  uint8_t page = (uint8_t)(y / 8u);
  uint8_t bit = (uint8_t)(y % 8u);
  if ((uint16_t)bit + len > 8u) {
    oled096_set_byte_fine(page, x, (uint8_t)(data << bit), bit, 7u, color);
    oled096_set_byte_fine((uint8_t)(page + 1u), x, (uint8_t)(data >> (8u - bit)), 0u, (uint8_t)(len + bit - 1u - 8u), color);
  } else {
    oled096_set_byte_fine(page, x, (uint8_t)(data << bit), bit, (uint8_t)(bit + len - 1u), color);
  }
}

static void oled096_set_bits(uint8_t x, uint8_t y, uint8_t data, OLED096_ColorMode color) {
  uint8_t page = (uint8_t)(y / 8u);
  uint8_t bit = (uint8_t)(y % 8u);
  oled096_set_byte_fine(page, x, (uint8_t)(data << bit), bit, 7u, color);
  if (bit != 0u) {
    oled096_set_byte_fine((uint8_t)(page + 1u), x, (uint8_t)(data >> (8u - bit)), 0u, (uint8_t)(bit - 1u), color);
  }
}

static void oled096_set_block(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, OLED096_ColorMode color) {
  uint8_t full_row = (uint8_t)(h / 8u);
  uint8_t part_bit = (uint8_t)(h % 8u);
  for (uint8_t i = 0u; i < w; i++) {
    for (uint8_t j = 0u; j < full_row; j++) {
      oled096_set_bits((uint8_t)(x + i), (uint8_t)(y + j * 8u), data[i + j * w], color);
    }
  }
  if (part_bit != 0u) {
    uint16_t full_num = (uint16_t)w * full_row;
    for (uint8_t i = 0u; i < w; i++) {
      oled096_set_bits_fine((uint8_t)(x + i), (uint8_t)(y + (full_row * 8u)), data[full_num + i], part_bit, color);
    }
  }
}

void OLED096_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED096_ColorMode color) {
  uint8_t temp = 0u;
  if (x1 == x2) {
    if (y1 > y2) {
      temp = y1;
      y1 = y2;
      y2 = temp;
    }
    for (uint8_t y = y1; y <= y2; y++) {
      OLED096_SetPixel(x1, y, color);
    }
  } else if (y1 == y2) {
    if (x1 > x2) {
      temp = x1;
      x1 = x2;
      x2 = temp;
    }
    for (uint8_t x = x1; x <= x2; x++) {
      OLED096_SetPixel(x, y1, color);
    }
  } else {
    int16_t dx = (int16_t)x2 - (int16_t)x1;
    int16_t dy = (int16_t)y2 - (int16_t)y1;
    int16_t ux = (int16_t)(((dx > 0) ? 1u : 0u) * 2 - 1);
    int16_t uy = (int16_t)(((dy > 0) ? 1u : 0u) * 2 - 1);
    int16_t x = (int16_t)x1;
    int16_t y = (int16_t)y1;
    int16_t eps = 0;
    dx = (int16_t)abs((int)dx);
    dy = (int16_t)abs((int)dy);
    if (dx > dy) {
      for (; x != (int16_t)x2; x = (int16_t)(x + ux)) {
        OLED096_SetPixel((uint8_t)x, (uint8_t)y, color);
        eps = (int16_t)(eps + dy);
        if ((eps * 2) >= dx) {
          y = (int16_t)(y + uy);
          eps = (int16_t)(eps - dx);
        }
      }
    } else {
      for (; y != (int16_t)y2; y = (int16_t)(y + uy)) {
        OLED096_SetPixel((uint8_t)x, (uint8_t)y, color);
        eps = (int16_t)(eps + dx);
        if ((eps * 2) >= dy) {
          x = (int16_t)(x + ux);
          eps = (int16_t)(eps - dy);
        }
      }
    }
  }
  OLED096_SetPixel(x2, y2, color);
}

void OLED096_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED096_ColorMode color) {
  OLED096_DrawLine(x, y, (uint8_t)(x + w), y, color);
  OLED096_DrawLine(x, (uint8_t)(y + h), (uint8_t)(x + w), (uint8_t)(y + h), color);
  OLED096_DrawLine(x, y, x, (uint8_t)(y + h), color);
  OLED096_DrawLine((uint8_t)(x + w), y, (uint8_t)(x + w), (uint8_t)(y + h), color);
}

void OLED096_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED096_ColorMode color) {
  for (uint8_t i = 0u; i < h; i++) {
    OLED096_DrawLine(x, (uint8_t)(y + i), (uint8_t)(x + w), (uint8_t)(y + i), color);
  }
}

void OLED096_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const OLED096_ASCIIFont *font, OLED096_ColorMode color) {
  if (font == NULL || font->chars == NULL) {
    return;
  }
  int idx = (int)ch - 32;
  if (idx < 0 || idx > 91) {
    idx = 0;
  }
  uint8_t bytes_per_col = (uint8_t)(((font->h + 7u) / 8u) * font->w);
  oled096_set_block(x, y, font->chars + (uint16_t)idx * bytes_per_col, font->w, font->h, color);
}

void OLED096_PrintASCIIString(uint8_t x, uint8_t y, char *str, const OLED096_ASCIIFont *font, OLED096_ColorMode color) {
  uint8_t x0 = x;
  if (str == NULL || font == NULL) {
    return;
  }
  while (*str != '\0') {
    OLED096_PrintASCIIChar(x0, y, *str, font, color);
    x0 = (uint8_t)(x0 + font->w);
    str++;
  }
}
