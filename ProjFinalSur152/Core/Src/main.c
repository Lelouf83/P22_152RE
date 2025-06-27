/* USER CODE BEGIN Header */

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include "lis2dw12_reg.h"
#include "knowledge.h"
#include "cycle_dwt.h"
#include "NanoEdgeAI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/************************************************************ NEAI algorithm defines begin ************************************************************/
/************************************************************ Global settings part ************************************************************/
#ifndef AXIS
#define AXIS              3                       /* Axis should be defined between 1 and 3 */
#endif
#ifndef SAMPLES
#define SAMPLES           128                     /* Should be between 16 & 4096 */
#endif
/************************************************************ Sensors configuration part ************************************************************/
#ifndef ACCELEROMETER_ODR
#define ACCELEROMETER_ODR  LIS2DW12_XL_ODR_400Hz  /* Shoud be between LIS2DW12_XL_ODR_12Hz5 and LIS2DW12_XL_ODR_1k6Hz */
#endif
#ifndef ACCELEROMETER_FS
#define ACCELEROMETER_FS   LIS2DW12_4g            /* Should be between LIS2DW12_2g and LIS2DW12_16g */
#endif
/************************************************************ Datalogger / NEAI mode part ************************************************************/
#ifndef NEAI_MODE
#define NEAI_MODE         1                       /* 0: Datalogger mode, 1: NEAI functions mode */
#endif
//#if (NEAI_MODE == 1)
//  #ifndef NEAI_LEARN_NB
//    #define NEAI_LEARN_NB   20                      /* Number of buffers to be learn by the NEAI library */
//  #endif
#if (NEAI_MODE)
#ifndef NB_CLASSES
#define NB_CLASSES  5/* Number of NEAI classes */
#endif
#endif
/************************************************************ NEAI algorithm defines end ************************************************************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t flag_tim3 = 0;
uint8_t motorState = 0;
uint8_t motorOverride = 0;
uint8_t debound_flag = 0;
uint8_t buzzerState = 0;
uint8_t buzzerOverride = 0;
uint32_t digitalValue = 0;
uint8_t valeurLB = 0;

static int16_t data_raw_acceleration[3];
static uint8_t whoamI, rst;
uint8_t neai_similarity = 0, neai_state = 0, first_comm = 1;
uint16_t sample_index = 0, id_class = 0;
volatile uint8_t drdy = 0;
uint16_t neai_cnt = 0, drdy_counter = 0;
float neai_time = 0.0;
float neai_buffer[AXIS * SAMPLES] = { 0 };
stmdev_ctx_t dev_ctx;

#if (NEAI_MODE)
static float class_output_buffer[NB_CLASSES];
const char *id2class[NB_CLASSES + 1] = { "unknown", "vertical", "shake","Immobile", "Horizontale" };
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static void lis2dw12_initialize(void);
static float lis2dw12_convert_data_to_mg(int16_t accel_raw_data);
static void iks01a3_i2c_stuck_quirk(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//RESOLU adc probleme arrete boucle infinie a une certaine value
void adcFunctionXblinking(void) {
	static uint8_t ledState = 0;  // mémorise l'état ON/OFF des LEDs

	HAL_ADC_Start(&hadc);  // Start sans interruption
	HAL_ADC_PollForConversion(&hadc, 100);
	if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC)) {
		digitalValue = HAL_ADC_GetValue(&hadc);
	}
	HAL_ADC_Stop(&hadc);  // Stop propre

	printf("Digital value: %lu\r\n", digitalValue);

	// Alternance ON/OFF
	ledState = !ledState;

	// L0 reste allumée si digitalValue > 0
	HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,
			(digitalValue > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// Clignotement de L1
	if (digitalValue > 2000) {
		HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,
				(ledState) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, GPIO_PIN_RESET);
	}

	// Délai pour rendre le clignotement visible
	HAL_Delay(200);
}

void buzzer(uint8_t buzzerState) {
	if (buzzerState) {
		printf("buzzer on \r\n");
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	} else if (!buzzerState) {
		printf("buzzer OFF \r\n");
		HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_2);
	}
}

void motor(uint8_t motorState) {
	if (motorState) {
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
		printf("Motor ON \r\n");
	} else {
		HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
		printf("Motor OFF \r\n");
	}
}

void TIM3_IRQ(void) {
	HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
	if (flag_tim3) {
		flag_tim3 = 0;
	} else {
		flag_tim3 = 1;
	}

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/* Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	iks01a3_i2c_stuck_quirk();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
	KIN1_InitCycleCounter();
	KIN1_EnableCycleCounter();
	HAL_TIM_Base_Start_IT(&htim3);
	printf("FINAL Timer 3 enabled\r\n");

	lis2dw12_initialize();
	if (NEAI_MODE) {
		neai_state = neai_classification_init(knowledge);
		printf("Initialize NEAI library. NEAI init return: %d.\n", neai_state);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (drdy) {
			/* Reset data ready condition */
			drdy = 0;
			/* Read acceleration data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
			for (uint8_t i = 0; i < AXIS; i++) {
				neai_buffer[(AXIS * drdy_counter) + i] =
						lis2dw12_convert_data_to_mg(data_raw_acceleration[i]);
			}
			drdy_counter++;
			if (drdy_counter >= SAMPLES) {
				/* Set Output Data Rate */
				lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_OFF);
#if NEAI_MODE
				neai_state = neai_classification(neai_buffer,
						class_output_buffer, &id_class);
				printf("Class: %s. NEAI classification return: %d.\r\n",
						id2class[id_class], neai_state);


				if (strcmp(id2class[id_class], "Horizontale") == 0) {

					printf("OK HORIZON.\r\n");

					if (!motorOverride) {
						motorState = 1;
						motor(motorState);
					}

					if (!buzzerOverride) {
						buzzerState = 1;
						buzzer(buzzerState);
					}

				}

				else if (strcmp(id2class[id_class], "Immobile") == 0) {
					printf("OK IMMOBILE.\r\n");

					if (!motorOverride) {
						motorState = 0;
						motor(motorState);
					}

					if (!buzzerOverride) {
						buzzerState = 0;
						buzzer(buzzerState);
					}

					adcFunctionXblinking();
				}

				else if (strcmp(id2class[id_class], "vertical") == 0) {

					printf("OK vertical.\r\n");
					if (!motorOverride) {
						motorState = 1;
						motor(motorState);
					}

					if (!buzzerOverride) {
						buzzerState = 0;
						buzzer(buzzerState);
					}

				}

				else if (strcmp(id2class[id_class], "shake") == 0) {

					printf("OK shake.\r\n");
					if (!motorOverride) {
						motorState = 0;
						motor(motorState);
					}

					if (!buzzerOverride) {
						buzzerState = 1;
						buzzer(buzzerState);
					}
					switchLedAll();

				}

#else
        for (uint16_t i = 0; i < AXIS * SAMPLES; i++) {
          printf("%.3f ", neai_buffer[i]);
        }
        printf("\n");
#endif
				/* Reset drdy_counter in order to get a new buffer */
				drdy_counter = 0;
				/* Clean neai buffer */
				memset(neai_buffer, 0x00, AXIS * SAMPLES * sizeof(float));
				/* Set Output Data Rate */
				lis2dw12_data_rate_set(&dev_ctx, ACCELEROMETER_ODR);
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = DISABLE;
  AnalogWDGConfig.HighThreshold = 3724;
  AnalogWDGConfig.LowThreshold = 0;
  if (HAL_ADC_AnalogWDGConfig(&hadc, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L0_Pin|L1_Pin|L2_Pin|L3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin BP3_OnOffMotor_Pin */
  GPIO_InitStruct.Pin = B1_Pin|BP3_OnOffMotor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIS2DW12_INT_Pin */
  GPIO_InitStruct.Pin = LIS2DW12_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIS2DW12_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L0_Pin L1_Pin L2_Pin L3_Pin */
  GPIO_InitStruct.Pin = L0_Pin|L1_Pin|L2_Pin|L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BP2_onOffBuzzer_Pin */
  GPIO_InitStruct.Pin = BP2_onOffBuzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BP2_onOffBuzzer_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Redirecting stdout to USART2 which is connected on the STLINK port
 * @retval
 * @param
 */
int __io_putchar(int ch) {
	uint8_t c[1];
	c[0] = ch & 0x00FF;
	HAL_UART_Transmit(&huart2, &*c, 1, 10);
	return ch;
}

void switchLedAll(void) {
	HAL_GPIO_TogglePin(L0_GPIO_Port, L0_Pin);
	HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
}

/**
 * @brief  EXTI line rising detection callback.
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BP2_onOffBuzzer_Pin) {
		if (!debound_flag) {
			debound_flag = 1;
			__HAL_TIM_SET_COUNTER(&htim6, 0);
			HAL_TIM_Base_Start_IT(&htim6);

			buzzerOverride = 1;               // activer override
			buzzerState = !buzzerState;       // inverser l'état
			buzzer(buzzerState);
			printf("!!!! Buzzer toggled via button: %d !!!!\r\n", buzzerState);
		}
	}

	if (GPIO_Pin == BP3_OnOffMotor_Pin) {
		if (!debound_flag) {
			debound_flag = 1;
			__HAL_TIM_SET_COUNTER(&htim6, 0);
			HAL_TIM_Base_Start_IT(&htim6);

			motorOverride = 1;
			motorState = !motorState;
			motor(motorState);
			printf("!!!! Motor toggled via button: %d!!!! \r\n", motorState);
		}
	}

	switch (GPIO_Pin) {
	case LIS2DW12_INT_Pin:
		drdy = 1;
		break;
	}
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) bufp, len, 1000);
	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT,
			bufp, len, 1000);
	return 0;
}

/*
 * @brief  Initialize LIS2DW12 sensor interface
 *
 * @param  No
 *
 * @return No
 *
 */
static void lis2dw12_initialize() {
	/* Check device ID */
	whoamI = 0;

	do {
		HAL_Delay(20);
		lis2dw12_device_id_get(&dev_ctx, &whoamI);
	} while (whoamI != LIS2DW12_ID);

	/* Restore default configuration */
	lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lis2dw12_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set full scale */
	lis2dw12_full_scale_set(&dev_ctx, ACCELEROMETER_FS);
	/* Configure power mode */
	lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
	/* Set Output Data Rate */
	lis2dw12_data_rate_set(&dev_ctx, ACCELEROMETER_ODR);
	/* Data-ready routed on INT1 pin */
	lis2dw12_ctrl4_int1_pad_ctrl_t int1_conf = { 0 };
	int1_conf.int1_drdy = PROPERTY_ENABLE;
	lis2dw12_pin_int1_route_set(&dev_ctx, &int1_conf);
	/* Read data to avoid bug at boot/reset du to the interrupt */
	lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
}

/*
 * @brief  Convert accelerometer raw data to milli-G' (mg)
 *
 * @param  accel_raw_data: which is accelerometer raw data
 *                        depending on the full scale selected
 *
 * @return The converted value in milli-G' (mg)
 *
 */
static float lis2dw12_convert_data_to_mg(int16_t accel_raw_data) {
	float accel_data_mg = 0.0;
	switch (ACCELEROMETER_FS) {
	case LIS2DW12_2g:
		accel_data_mg = lis2dw12_from_fs2_to_mg(accel_raw_data);
		break;
	case LIS2DW12_4g:
		accel_data_mg = lis2dw12_from_fs4_to_mg(accel_raw_data);
		break;
	case LIS2DW12_8g:
		accel_data_mg = lis2dw12_from_fs8_to_mg(accel_raw_data);
		break;
	case LIS2DW12_16g:
		accel_data_mg = lis2dw12_from_fs16_to_mg(accel_raw_data);
		break;
	default:
		accel_data_mg = 0.0;
		break;
	}
	return accel_data_mg;
}

/*
 * Pressing the reset button while the sensor is answering to a read request
 * might lead to disaster.
 * In this case the device is stuck, waiting for pulses on SCL to finish the
 * previous transfer.
 * While stuck the sensor keep the SDA low.
 *
 * As a workaround we simply configure the SCL pin as a GPIO and send a burst
 * of pulses to bring the sensor back to an idle state.
 */
static void iks01a3_i2c_stuck_quirk(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure SCL as a GPIO */
	GPIO_InitStruct.Pin = SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);

	/* Send a burst of pulses on SCL */
	int pulses = 20;
	do {
		HAL_Delay(1);
		HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
	} while (pulses--);

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_DISABLE();
}

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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
