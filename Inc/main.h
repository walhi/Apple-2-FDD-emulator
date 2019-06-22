/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

enum phase {
	PHASE0,
	PHASE1,
	PHASE2,
	PHASE3,
} lastPhase;
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define WRITE_REQUEST_Pin GPIO_PIN_0
#define WRITE_REQUEST_GPIO_Port GPIOA
#define WRITE_SIGNAL_Pin GPIO_PIN_1
#define WRITE_SIGNAL_GPIO_Port GPIOA
#define WRITE_SIGNAL_COPY_Pin GPIO_PIN_2
#define WRITE_SIGNAL_COPY_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOA
#define PHASE0_Pin GPIO_PIN_10
#define PHASE0_GPIO_Port GPIOB
#define PHASE0_EXTI_IRQn EXTI15_10_IRQn
#define PHASE1_Pin GPIO_PIN_11
#define PHASE1_GPIO_Port GPIOB
#define PHASE1_EXTI_IRQn EXTI15_10_IRQn
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB
#define SD_CLK_Pin GPIO_PIN_13
#define SD_CLK_GPIO_Port GPIOB
#define SD_DO_Pin GPIO_PIN_14
#define SD_DO_GPIO_Port GPIOB
#define SD_DI_Pin GPIO_PIN_15
#define SD_DI_GPIO_Port GPIOB
#define PHASE2_Pin GPIO_PIN_8
#define PHASE2_GPIO_Port GPIOA
#define PHASE2_EXTI_IRQn EXTI9_5_IRQn
#define PHASE3_Pin GPIO_PIN_9
#define PHASE3_GPIO_Port GPIOA
#define PHASE3_EXTI_IRQn EXTI9_5_IRQn
#define SD_DETECT_Pin GPIO_PIN_11
#define SD_DETECT_GPIO_Port GPIOA
#define SD_WREN_Pin GPIO_PIN_12
#define SD_WREN_GPIO_Port GPIOA
#define PULSE_TRIG_Pin GPIO_PIN_4
#define PULSE_TRIG_GPIO_Port GPIOB
#define PULSE_Pin GPIO_PIN_5
#define PULSE_GPIO_Port GPIOB
#define DRIVE1_ENABLE_Pin GPIO_PIN_8
#define DRIVE1_ENABLE_GPIO_Port GPIOB
#define DRIVE2_ENABLE_Pin GPIO_PIN_9
#define DRIVE2_ENABLE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
