#include "delay_dwt.h"
#include "stm32f1xx_hal.h"

void delay_init(void)
{
	SystemCoreClockUpdate();
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t nus)
{
	uint32_t clk = SystemCoreClock;
	if (clk == 0U) {
		clk = 72000000U;
	}
	uint32_t cycles = (nus * clk) / 1000000U;
	if (cycles == 0U) {
		cycles = 1U;
	}
	uint32_t start = DWT->CYCCNT;
	while ((DWT->CYCCNT - start) < cycles) {
	}
}

void delay_ms(uint16_t nms)
{
	HAL_Delay(nms);
}
