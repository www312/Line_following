/**
 * @file f302_radar_uart.c
 * @brief F302 雷达 USART1 中断收包 + 主循环解析
 */

#include "f302_radar_uart.h"

#define F302_UART_RX_RING_MASK 255u

static UART_HandleTypeDef *s_huart;
static uint8_t s_uart_rx_byte;
static uint8_t s_rb[256];
static volatile uint8_t s_rb_w;
static volatile uint8_t s_rb_r;

static volatile uint8_t s_linked;
static f302_target_t s_target;
static volatile uint32_t s_dbg_rx_bytes;
static volatile uint32_t s_dbg_errors;
static volatile uint32_t s_dbg_frames;

static void f302_radar_rx_restart(void)
{
	if (s_huart != NULL) {
		(void)HAL_UART_Receive_IT(s_huart, &s_uart_rx_byte, 1u);
	}
}

static void f302_radar_rb_drain(void)
{
	while (s_rb_r != s_rb_w) {
		uint8_t b = s_rb[s_rb_r];
		f302_target_t t;

		s_rb_r = (uint8_t)((s_rb_r + 1u) & F302_UART_RX_RING_MASK);
		if (f302_feed_byte(b, &t) != 0u) {
			s_linked = 1u;
			s_target = t;
		}
	}
}

void F302_Radar_Init(UART_HandleTypeDef *huart)
{
	s_huart = huart;
	s_rb_w = 0u;
	s_rb_r = 0u;
	s_linked = 0u;
	s_target.range_0p01m = 0u;
	s_target.angle_0p01deg = 0;
	s_target.have_target = 0u;
	f302_parser_reset();
	f302_radar_rx_restart();
}

void F302_Radar_Poll(void)
{
	f302_radar_rb_drain();
}

uint8_t F302_Radar_IsLinked(void)
{
	return s_linked;
}

void F302_Radar_GetTarget(f302_target_t *out)
{
	if (out != NULL) {
		*out = s_target;
	}
}

void F302_Radar_UartRxCplt(UART_HandleTypeDef *huart)
{
	if (s_huart == NULL || huart->Instance != USART1) {
		return;
	}
	{
		const uint8_t nw = (uint8_t)((s_rb_w + 1u) & F302_UART_RX_RING_MASK);

		if (nw != s_rb_r) {
			s_rb[s_rb_w] = s_uart_rx_byte;
			s_rb_w = nw;
		}
	}
	f302_radar_rx_restart();
}

void F302_Radar_UartError(UART_HandleTypeDef *huart)
{
	if (s_huart == NULL || huart->Instance != USART1) {
		return;
	}
	{
		volatile uint32_t sr = huart->Instance->SR;
		volatile uint32_t dr = huart->Instance->DR;

		(void)sr;
		(void)dr;
	}
	(void)HAL_UART_AbortReceive(huart);
	f302_radar_rx_restart();
}
