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
#include "f302_radar_uart.h"
#include "u8g2.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UI_STEP_MS            50U
#define OLED_I2C_ADDR        0x3CU

#define SUPPRESS_MS          1500U   /* ignore PB12 for this window after any turn */
#define RADAR_MODE_TIMEOUT_MS 5000U   /* quit RADAR_MODE if no all_black */
#define STOP_DURATION_MS     3000U   /* stop & poll radar for 3s */
#define STARTUP_IGNORE_MS    2000U   /* ignore PB12 for first 2s after power-on */
#define LED_BLINK_MS         3000U   /* PA6 LED blinks for this long */
#define AVOID_TIMEOUT_MS    15000U   /* max avoid duration */
#define RADAR_DIST_THRESH_CM   60U   /* max valid obstacle range */
#define RADAR_ANGLE_LIMIT     6000   /* ±60 deg in 0.01deg units */
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
static u8g2_t s_u8g2;

/* ---- state machine ---- */
static uint8_t         s_avoid_active;
static uint32_t        s_avoid_tick;
static LineFollow_Mode s_pre_mode;       /* saved mode before entering avoid */

static uint8_t         s_radar_mode;      /* 1 = RADAR_MODE, waiting for all_black */
static uint32_t        s_radar_mode_tick; /* entry tick for 5s timeout */

static uint8_t         s_stop_active;     /* 1 = STOP_1S, car halted reading radar */
static uint32_t        s_stop_tick;       /* entry tick for 1s duration */
static uint8_t         s_stop_has_target; /* 1 = saw at least one valid target */
static int16_t         s_stop_angle;      /* last valid angle during stop (-60..+60 deg * 100) */

static uint8_t         s_pb12_prev;       /* previous PB12 for rising-edge detect */
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
/* ---- radar polar coordinate helpers (same as before) ---- */
#define RADAR_CX 64
#define RADAR_CY 50
#define RADAR_R  28
#define RADAR_TARGET_R 40
#define RADAR_RANGE_MAX 300

static const int16_t SIN256[121] = {
	-222, -219, -217, -215, -212, -210, -207, -204, -202, -199, -196, -193, -190, -187,
	-184, -181, -178, -175, -171, -168, -165, -161, -158, -154, -150, -147, -143, -139,
	-136, -132, -128, -124, -120, -116, -112, -108, -104, -100, -96,  -92,  -88,  -83,
	-79,  -75,  -71,  -66,  -62,  -58,  -53,  -49,  -44,  -40,  -36,  -31,  -27,  -22,
	-18,  -13,  -9,   -4,   0,
	4,    9,    13,   18,   22,   27,   31,   36,   40,   44,   49,   53,   58,   62,
	66,   71,   75,   79,   83,   88,   92,   96,   100,  104,  108,  112,  116,  120,
	124,  128,  132,  136,  139,  143,  147,  150,  154,  158,  161,  165,  168,  171,
	175,  178,  181,  184,  187,  190,  193,  196,  199,  202,  204,  207,  210,  212,
	215,  217,  219,  222,
};
static const int16_t COS256[121] = {
	128, 132, 136, 139, 143, 147, 150, 154, 158, 161, 165, 168, 171, 175, 178, 181, 184,
	187, 190, 193, 196, 199, 202, 204, 207, 210, 212, 215, 217, 219, 222, 224, 226, 228,
	230, 232, 234, 236, 237, 239, 241, 242, 243, 245, 246, 247, 248, 249, 250, 251, 252,
	253, 254, 254, 255, 255, 255, 256, 256, 256, 256, 256, 256, 256, 255, 255, 255, 254,
	254, 253, 252, 251, 250, 249, 248, 247, 246, 245, 243, 242, 241, 239, 237, 236, 234,
	232, 230, 228, 226, 224, 222, 219, 217, 215, 212, 210, 207, 204, 202, 199, 196, 193,
	190, 187, 184, 181, 178, 175, 171, 168, 165, 161, 158, 154, 150, 147, 143, 139, 136,
	132, 128,
};

static void radar_polar(int16_t deg, int16_t r_px, int16_t *dx, int16_t *dy)
{
	int idx = (int)deg + 60;
	if (idx < 0)  idx = 0;
	if (idx > 120) idx = 120;
	*dx = (int16_t)((r_px * SIN256[idx]) / 256);
	*dy = (int16_t)(-(r_px * COS256[idx]) / 256);
}

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

/* ---- OLED: sensor dashboard (NORMAL mode) ---- */
static void UI_DrawSensorScreen(void)
{
	char buf[32];
	uint8_t i;

	u8g2_ClearBuffer(&s_u8g2);

	/* Title */
	u8g2_SetFont(&s_u8g2, u8g2_font_helvB08_tr);
	u8g2_DrawStr(&s_u8g2, 24, 10, "GRAY SENSOR");

	/* 8 indicator blocks: S8 leftmost → S1 rightmost */
	for (i = 0; i < 8; i++) {
		uint8_t x = (uint8_t)(1U + (7U - i) * 16U);
		if (IR_Line8_Value[i] == 0U)
			u8g2_DrawBox(&s_u8g2, x, 18, 14, 22);
		else
			u8g2_DrawFrame(&s_u8g2, x, 18, 14, 22);
	}

	/* S1-S8 labels */
	u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);
	for (i = 0; i < 8; i++) {
		buf[0] = 'S'; buf[1] = (char)('1' + i); buf[2] = '\0';
		u8g2_DrawStr(&s_u8g2, (uint8_t)(3U + (7U - i) * 16U), 50, buf);
	}

	/* Status line: current mode + suppressed indicator */
	{
		uint32_t now = HAL_GetTick();
		uint32_t lt = LineFollow_GetLastTurnTick();
		uint8_t  suppressed = (lt != 0U && (uint32_t)(now - lt) < SUPPRESS_MS);
		LineFollow_Mode m = LineFollow_GetMode();

		if (suppressed) {
			uint32_t remain = SUPPRESS_MS - (uint32_t)(now - lt);
			snprintf(buf, sizeof(buf), "%s SU%lu.%lus",
				 (m == MODE_RIGHT) ? "R" : "L",
				 (unsigned long)(remain / 1000U),
				 (unsigned long)((remain / 100U) % 10U));
		} else {
			snprintf(buf, sizeof(buf), "FOLLOW %s",
				 (m == MODE_RIGHT) ? "R" : "L");
		}
	}
	u8g2_DrawStr(&s_u8g2, 0, 62, buf);

	u8g2_SendBuffer(&s_u8g2);
}

/* ---- OLED: radar mode (RADAR_MODE / STOP_1S / AVOID) ---- */
static void UI_DrawRadarScreen(void)
{
	f302_target_t rd;
	uint8_t linked = F302_Radar_IsLinked();
	char buf[32];
	uint32_t now = HAL_GetTick();
	int16_t fan_deg[] = { -60, -40, -20, 0, 20, 40, 60 };
	uint8_t i;

	u8g2_ClearBuffer(&s_u8g2);

	/* top info: distance + angle */
	u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);
	if (!linked) {
		u8g2_DrawStr(&s_u8g2, 0, 10, "F302 WAIT");
	} else {
		F302_Radar_GetTarget(&rd);
		if (rd.have_target) {
			snprintf(buf, sizeof(buf), "R %u.%02um",
				 (unsigned)(rd.range_0p01m / 100),
				 (unsigned)(rd.range_0p01m % 100));
			u8g2_DrawStr(&s_u8g2, 0, 10, buf);
			snprintf(buf, sizeof(buf), "A %d",
				 (int)(rd.angle_0p01deg / 100));
			u8g2_DrawStr(&s_u8g2, 80, 10, buf);
		} else {
			u8g2_DrawStr(&s_u8g2, 0, 10, "CLEAR");
		}
	}

	/* fan rays */
	for (i = 0; i < 7; i++) {
		int16_t dx, dy;
		radar_polar(fan_deg[i], RADAR_R, &dx, &dy);
		u8g2_DrawLine(&s_u8g2, RADAR_CX, RADAR_CY,
			      (u8g2_uint_t)(RADAR_CX + dx),
			      (u8g2_uint_t)(RADAR_CY + dy));
	}

	/* 3 concentric circles */
	u8g2_DrawCircle(&s_u8g2, RADAR_CX, RADAR_CY,  9, U8G2_DRAW_ALL);
	u8g2_DrawCircle(&s_u8g2, RADAR_CX, RADAR_CY, 19, U8G2_DRAW_ALL);
	u8g2_DrawCircle(&s_u8g2, RADAR_CX, RADAR_CY, 28, U8G2_DRAW_ALL);

	/* center dot */
	u8g2_DrawBox(&s_u8g2, RADAR_CX - 1, RADAR_CY - 1, 3, 3);

	/* target */
	if (linked) {
		F302_Radar_GetTarget(&rd);
		if (rd.have_target) {
			int16_t deg  = (int16_t)(rd.angle_0p01deg / 100);
			int32_t rpx  = ((int32_t)rd.range_0p01m * RADAR_TARGET_R)
				       / RADAR_RANGE_MAX;
			int16_t dx, dy;

			if (rpx < 2)  rpx = 2;
			if (rpx > RADAR_TARGET_R) rpx = RADAR_TARGET_R;
			radar_polar(deg, (int16_t)rpx, &dx, &dy);
			u8g2_uint_t tx = (u8g2_uint_t)(RADAR_CX + dx);
			u8g2_uint_t ty = (u8g2_uint_t)(RADAR_CY + dy);

			if (tx >= 2) tx -= 2; else tx = 0;
			if (ty >= 2) ty -= 2; else ty = 0;
			u8g2_DrawBox(&s_u8g2, tx, ty, 4, 4);
		}
	}

	/* status bar */
	u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);
	if (s_stop_active) {
		uint32_t remain = STOP_DURATION_MS - (uint32_t)(now - s_stop_tick);
		snprintf(buf, sizeof(buf), "STOP %lu.%lus",
			 (unsigned long)(remain / 1000U),
			 (unsigned long)((remain / 100U) % 10U));
	} else if (s_radar_mode) {
		uint32_t remain = 0U;
		if (now < s_radar_mode_tick + RADAR_MODE_TIMEOUT_MS)
			remain = (s_radar_mode_tick + RADAR_MODE_TIMEOUT_MS - now) / 1000U;
		snprintf(buf, sizeof(buf), "RADAR %lus", (unsigned long)remain);
	} else if (s_avoid_active) {
		uint32_t remain = 0U;
		if (now < s_avoid_tick + AVOID_TIMEOUT_MS)
			remain = (s_avoid_tick + AVOID_TIMEOUT_MS - now) / 1000U;
		LineFollow_Mode m = LineFollow_GetMode();
		snprintf(buf, sizeof(buf), "AVOID %c %lus",
			 (m == MODE_RIGHT) ? 'R' : 'L',
			 (unsigned long)remain);
	} else {
		snprintf(buf, sizeof(buf), "FOLLOW");
	}
	u8g2_DrawStr(&s_u8g2, 0, 62, buf);

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
  F302_Radar_Init(&huart1);

  /* init state machine */
  s_avoid_active  = 0U;
  s_radar_mode    = 0U;
  s_stop_active   = 0U;
  s_stop_has_target = 0U;
  s_pb12_prev     = 0U;

  s_ui_last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IR_Line8_PollRx();
    F302_Radar_Poll();

    uint32_t now = HAL_GetTick();

    /* ---- all_black: 8 sensors on black line ---- */
    uint8_t all_black = 1U;
    for (uint8_t i = 0U; i < 8U; i++) {
      if (IR_Line8_Value[i] != 0U) { all_black = 0U; break; }
    }

    /* ---- PB12: camera T-junction signal ---- */
    uint8_t pb12       = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pb12_rise  = (pb12 == 1U && s_pb12_prev == 0U) ? 1U : 0U;
    s_pb12_prev = pb12;

    /* ---- suppressed: window after any left/right turn ---- */
    uint32_t last_turn = LineFollow_GetLastTurnTick();
    uint8_t suppressed = (last_turn != 0U && (uint32_t)(now - last_turn) < SUPPRESS_MS);

    /* ---- PA6 LED: blink 200ms on/off for first LED_BLINK_MS of RADAR_MODE ---- */
    if (s_radar_mode) {
      uint32_t led_elapsed = (uint32_t)(now - s_radar_mode_tick);
      if (led_elapsed < LED_BLINK_MS) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
            ((led_elapsed / 200U) % 2U == 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      }
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }

    /* ================================================================
     *  STATE MACHINE
     * ================================================================ */
    if (s_stop_active) {
      /* ── STOP_1S: car halted, poll radar (top of loop already polled) ── */

      f302_target_t rd;
      F302_Radar_GetTarget(&rd);
      if (rd.have_target
          && rd.range_0p01m <= (uint16_t)RADAR_DIST_THRESH_CM
          && rd.angle_0p01deg >= -RADAR_ANGLE_LIMIT
          && rd.angle_0p01deg <=  RADAR_ANGLE_LIMIT) {
        s_stop_has_target = 1U;
        s_stop_angle = rd.angle_0p01deg;
      }

      if ((uint32_t)(now - s_stop_tick) >= STOP_DURATION_MS) {
        s_stop_active = 0U;
        if (s_stop_has_target) {
          /* obstacle detected → enter AVOID */
          if (s_stop_angle < 0) {
            LineFollow_SetMode(MODE_RIGHT);   /* obstacle left → go right */
          } else {
            LineFollow_SetMode(MODE_LEFT);    /* obstacle right → go left */
          }
          s_avoid_active = 1U;
          s_avoid_tick   = now;
        }
        /* else: no target → back to NORMAL, keep original mode */
      }

    } else if (s_avoid_active) {
      /* ── AVOID: line-walk with avoid direction ── */
      LineWalking();
      if ((uint32_t)(now - s_avoid_tick) >= AVOID_TIMEOUT_MS) {
        LineFollow_SetMode(s_pre_mode);
        s_avoid_active = 0U;
      }

    } else if (s_radar_mode) {
      /* ── RADAR_MODE: wait for all_black or timeout ── */
      LineWalking();
      if (!pb12) {
        /* camera lost T-junction before all_black → back to NORMAL */
        s_radar_mode = 0U;
      } else if (all_black) {
        /* all_black trigger → stop and read radar */
        s_radar_mode = 0U;
        Motion_Car_Control(0, 0, 0);
        s_stop_active     = 1U;
        s_stop_tick       = now;
        s_stop_has_target = 0U;
      } else if ((uint32_t)(now - s_radar_mode_tick) >= RADAR_MODE_TIMEOUT_MS) {
        /* 5s timeout → back to NORMAL */
        s_radar_mode = 0U;
      }

    } else {
      /* ── NORMAL: line-walk, wait for camera T-junction ── */
      LineWalking();
      if (pb12_rise && !suppressed && now >= STARTUP_IGNORE_MS) {
        s_radar_mode      = 1U;
        s_radar_mode_tick = now;
        s_pre_mode        = LineFollow_GetMode();
      }
    }

    /* ---- UI refresh ---- */
    if ((uint32_t)(now - s_ui_last_tick) >= UI_STEP_MS) {
      s_ui_last_tick = now;
      if (s_avoid_active || s_stop_active) {
        UI_DrawRadarScreen();
      } else {
        UI_DrawSensorScreen();
      }
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

  /* PA6: LED indicator, push-pull output, default low */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

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
