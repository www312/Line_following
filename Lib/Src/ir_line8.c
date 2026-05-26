/**
 * @file ir_line8.c
 * @brief 8-ch grayscale sensor — USART2, direct ISR read (matches official bsp_usart.c)
 */
#include "ir_line8.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

#define IR_LINE8_PKG_SIZE 100U

static uint8_t s_rx_buf[IR_LINE8_PKG_SIZE];
static uint8_t s_pkg[IR_LINE8_PKG_SIZE];
static uint8_t s_rx_active;
static uint8_t s_rx_step;
static volatile uint32_t s_frame_count;
static uint32_t s_last_frame_ms;

uint8_t IR_Line8_Value[IR_LINE8_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1};
volatile uint8_t IR_Line8_FrameReady;

/* ---- parser (identical to official Deal_Usart_Data) ---- */
static void ir_line8_parse_digital(void)
{
	if (s_pkg[1] != 'D') return;
	for (uint8_t i = 0; i < IR_LINE8_COUNT; i++)
		IR_Line8_Value[i] = (uint8_t)(s_pkg[6U + i * 5U] - (uint8_t)'0');
	s_frame_count++;
	s_last_frame_ms = HAL_GetTick();
	IR_Line8_FrameReady = 1U;
	memset(s_pkg, 0, IR_LINE8_PKG_SIZE);
}

/* ---- frame state machine (identical to official Deal_IR_Usart) ---- */
void IR_Line8_FeedByte(uint8_t byte)
{
	if (byte == '$') {
		s_rx_active  = 1U;
		s_rx_step    = 0U;
		s_rx_buf[0]  = byte;
		s_rx_step    = 1U;
		return;
	}
	if (s_rx_active == 0U) return;
	s_rx_buf[s_rx_step++] = byte;
	if (byte == '#') {
		s_rx_active = 0U;
		s_rx_step   = 0U;
		memcpy(s_pkg, s_rx_buf, IR_LINE8_PKG_SIZE);
		ir_line8_parse_digital();
		memset(s_rx_buf, 0, IR_LINE8_PKG_SIZE);
	} else if (s_rx_step >= IR_LINE8_PKG_SIZE) {
		s_rx_active = 0U;
		s_rx_step   = 0U;
		memset(s_rx_buf, 0, IR_LINE8_PKG_SIZE);
	}
}

/* ---- command transmit ---- */
static void ir_line8_send_cmd(uint8_t adjust, uint8_t analog, uint8_t digital)
{
	uint8_t buf[8] = "$0,0,0#";
	buf[1] = (adjust  != 0U) ? '1' : '0';
	buf[3] = (analog  != 0U) ? '1' : '0';
	buf[5] = (digital != 0U) ? '1' : '0';
	HAL_UART_Transmit(&huart2, buf, (uint16_t)strlen((const char *)buf), 100U);
}

/* ---- main-loop: clear ORE so RXNE keeps firing ---- */
void IR_Line8_PollRx(void)
{
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
		(void)huart2.Instance->SR;
		(void)huart2.Instance->DR;
	}
}

/* ---- init: switch sensor to digital mode, then enable RXNE directly ---- */
void IR_Line8_Init(void)
{
	IR_Line8_FrameReady = 0U;
	s_rx_active  = 0U;
	s_rx_step    = 0U;
	memset(s_rx_buf, 0, IR_LINE8_PKG_SIZE);
	memset(s_pkg,    0, IR_LINE8_PKG_SIZE);

	HAL_Delay(50);
	IR_Line8_EnableDigitalMode();
	HAL_Delay(100);

	/* flush stale flags before arming (same as official USART ITConfig after init) */
	(void)huart2.Instance->SR;
	(void)huart2.Instance->DR;
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* ---- convenience wrappers ---- */
void IR_Line8_EnableDigitalMode(void)  { ir_line8_send_cmd(0U, 0U, 1U); }
void IR_Line8_EnableAnalogMode(void)   { ir_line8_send_cmd(0U, 1U, 0U); }
void IR_Line8_Calibrate(void)          { ir_line8_send_cmd(1U, 0U, 0U); }
void IR_Line8_ClearFrameFlag(void)     { IR_Line8_FrameReady = 0U; }
uint32_t IR_Line8_GetFrameCount(void)  { return s_frame_count; }

uint32_t IR_Line8_GetMsSinceLastFrame(void)
{
	return (s_last_frame_ms == 0U) ? 0xFFFFFFFFU
				       : (uint32_t)(HAL_GetTick() - s_last_frame_ms);
}
