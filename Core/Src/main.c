/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay_dwt.h"
#include "ir_line8.h"
#include "line_follow.h"
#include "motion_car.h"
#include "motor_driver.h"
#include "u8g2.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UI_STEP_MS 50U
#define OLED_I2C_ADDR 0x3CU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t s_ui_last_tick;
static uint32_t s_btn_last_tick;
static u8g2_t s_u8g2;
static uint8_t s_radar_mode; /* 0=follow, 1=radar */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void UI_DrawSensorScreen(void);
static void UI_DrawRadarScreen(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- u8g2 HAL callbacks ---- */
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	(void)u8x8; (void)arg_ptr;
	switch (msg) {
	case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
	case U8X8_MSG_DELAY_10MICRO:
	case U8X8_MSG_GPIO_RESET:
		break;
	default: return 0;
	}
	return 1;
}

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	(void)u8x8;
	static uint8_t buf[128];
	static uint8_t idx;

	switch (msg) {
	case U8X8_MSG_BYTE_START_TRANSFER:
		idx = 0U;
		break;
	case U8X8_MSG_BYTE_SEND:
		memcpy(buf + idx, arg_ptr, arg_int);
		idx = (uint8_t)(idx + arg_int);
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(OLED_I2C_ADDR << 1U),
					buf, idx, 100U);
		break;
	case U8X8_MSG_BYTE_INIT:
	case U8X8_MSG_BYTE_SET_DC:
		break;
	default: return 0;
	}
	return 1;
}

/* ---- OLED display: sensor dashboard ---- */
static void UI_DrawSensorScreen(void)
{
	char buf[32];
	uint8_t i;

	u8g2_ClearBuffer(&s_u8g2);

	/* Title */
	u8g2_SetFont(&s_u8g2, u8g2_font_helvB08_tr);
	u8g2_DrawStr(&s_u8g2, 24, 10, "GRAY SENSOR");

	/* 8 indicator blocks: 14x22 px, 2px gap, S8 left→S1 right */
	for (i = 0; i < 8; i++) {
		uint8_t x = (uint8_t)(1U + (7U - i) * 16U);
		if (IR_Line8_Value[i] == 0U)
			u8g2_DrawBox(&s_u8g2, x, 18, 14, 22);
		else
			u8g2_DrawFrame(&s_u8g2, x, 18, 14, 22);
	}

	/* S1-S8 labels under blocks */
	u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);
	for (i = 0; i < 8; i++) {
		buf[0] = 'S'; buf[1] = (char)('1' + i); buf[2] = '\0';
		u8g2_DrawStr(&s_u8g2, (uint8_t)(3U + (7U - i) * 16U), 50, buf);
	}

	/* Status line: show DET/CLE groups */
	buf[0] = 'D'; buf[1] = ':';
	for (i = 0; i < 8; i++)
		buf[2 + i] = (char)('0' + (IR_Line8_Value[i] & 1U));
	buf[10] = '\0';
	u8g2_DrawStr(&s_u8g2, 0, 62, buf);

	u8g2_SendBuffer(&s_u8g2);
}

/* ---- OLED display: radar mode ---- */
static void UI_DrawRadarScreen(void)
{
	const u8g2_uint_t cx = 64, cy = 28;

	u8g2_ClearBuffer(&s_u8g2);

	/* 3 concentric circles */
	u8g2_DrawCircle(&s_u8g2, cx, cy,  9, U8G2_DRAW_ALL);
	u8g2_DrawCircle(&s_u8g2, cx, cy, 18, U8G2_DRAW_ALL);
	u8g2_DrawCircle(&s_u8g2, cx, cy, 27, U8G2_DRAW_ALL);

	/* crosshair */
	u8g2_DrawHLine(&s_u8g2, cx - 27, cy, 54);
	u8g2_DrawVLine(&s_u8g2, cx, cy - 27, 54);

	/* dummy target at 45 deg, r=15 */
	u8g2_DrawPixel(&s_u8g2, cx + 11, cy - 11);

	/* angle labels */
	u8g2_SetFont(&s_u8g2, u8g2_font_u8glib_4_tr);
	u8g2_DrawStr(&s_u8g2, 94, 29, "0");
	u8g2_DrawStr(&s_u8g2, 87,  9, "45");
	u8g2_DrawStr(&s_u8g2, 60,  2, "90");
	u8g2_DrawStr(&s_u8g2, 36,  9, "135");
	u8g2_DrawStr(&s_u8g2, 28, 29, "180");
	u8g2_DrawStr(&s_u8g2, 36, 50, "225");
	u8g2_DrawStr(&s_u8g2, 58, 60, "270");
	u8g2_DrawStr(&s_u8g2, 87, 50, "315");

	/* status */
	u8g2_DrawStr(&s_u8g2, 0, 62, "D:--cm A:--");

	u8g2_SendBuffer(&s_u8g2);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  delay_init();
  IR_Line8_Init();
  IIC_Motor_Init();
  Set_Motor(2);
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&s_u8g2, U8G2_R0,
      u8x8_byte_hw_i2c, u8x8_gpio_and_delay);
  u8g2_InitDisplay(&s_u8g2);
  u8g2_SetPowerSave(&s_u8g2, 0);
  s_ui_last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IR_Line8_PollRx();
    LineWalking();

    /* button: PB12=radar toggle, PB13=left↔right, 200ms debounce */
    if ((uint32_t)(HAL_GetTick() - s_btn_last_tick) >= 200U) {
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
        s_radar_mode = (uint8_t)(!s_radar_mode);
        s_btn_last_tick = HAL_GetTick();
      } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) {
        if (!s_radar_mode)
          LineFollow_SetMode(LineFollow_GetMode() == MODE_LEFT ? MODE_RIGHT : MODE_LEFT);
        s_btn_last_tick = HAL_GetTick();
      }
    }

    if (!s_radar_mode)
      LineWalking();

    if ((uint32_t)(HAL_GetTick() - s_ui_last_tick) >= UI_STEP_MS) {
      s_ui_last_tick = HAL_GetTick();
      if (s_radar_mode)
        UI_DrawRadarScreen();
      else
        UI_DrawSensorScreen();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_SCL_Pin|MOTOR_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MOTOR_SCL_Pin MOTOR_SDA_Pin */
  GPIO_InitStruct.Pin = MOTOR_SCL_Pin|MOTOR_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
