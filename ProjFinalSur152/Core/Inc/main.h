/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void switchLedAll(void);
void motor(uint8_t motorState);
void TIM3_IRQ(void);
void adcFunctionXblinking(void);
void buzzer(uint8_t buzzerState);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LIS2DW12_INT_Pin GPIO_PIN_0
#define LIS2DW12_INT_GPIO_Port GPIOB
#define LIS2DW12_INT_EXTI_IRQn EXTI0_IRQn
#define L0_Pin GPIO_PIN_1
#define L0_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_2
#define L1_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_10
#define L2_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_11
#define L3_GPIO_Port GPIOB
#define BP3_OnOffMotor_Pin GPIO_PIN_6
#define BP3_OnOffMotor_GPIO_Port GPIOC
#define BP3_OnOffMotor_EXTI_IRQn EXTI9_5_IRQn
#define BP2_onOffBuzzer_Pin GPIO_PIN_12
#define BP2_onOffBuzzer_GPIO_Port GPIOA
#define BP2_onOffBuzzer_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SCL_PORT GPIOB
#define SCL_PIN (1 << 8)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
