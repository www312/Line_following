/**
 * @file delay_dwt.h
 * @brief DWT microsecond delay + HAL millisecond delay
 */
#ifndef DELAY_DWT_H
#define DELAY_DWT_H

#include <stdint.h>

void delay_init(void);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);

#endif
