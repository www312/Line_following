/**
 * @file f302_radar_uart.h
 * @brief F302 雷达：USART1 中断收包 + 主循环解析（115200 8N1，PA9/PA10）
 */

#ifndef F302_RADAR_UART_H
#define F302_RADAR_UART_H

#include "f302_radar.h"
#include "stm32f1xx_hal.h"

void F302_Radar_Init(UART_HandleTypeDef *huart);
void F302_Radar_Poll(void);

uint8_t F302_Radar_IsLinked(void);
void F302_Radar_GetTarget(f302_target_t *out);
void F302_Radar_UartRxCplt(UART_HandleTypeDef *huart);
void F302_Radar_UartError(UART_HandleTypeDef *huart);

#endif
