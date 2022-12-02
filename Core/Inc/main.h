/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN4_Pin GPIO_PIN_1
#define IN4_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOC
#define SPI1_CS2_Pin GPIO_PIN_10
#define SPI1_CS2_GPIO_Port GPIOD
#define SPI1_CS3_Pin GPIO_PIN_11
#define SPI1_CS3_GPIO_Port GPIOD
#define SPI2_CS1_Pin GPIO_PIN_12
#define SPI2_CS1_GPIO_Port GPIOD
#define SPI2_CS3_Pin GPIO_PIN_14
#define SPI2_CS3_GPIO_Port GPIOD
#define SPI1_CS1_Pin GPIO_PIN_7
#define SPI1_CS1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

#define PITCH 0
#define ROLL  1
#define YAW   2

# define LED1_OFF    HAL_GPIO_WritePin(LED1_PORT, LED1_Pin, GPIO_PIN_SET);
# define LED1_TOGGLE    HAL_GPIO_TogglePin(LED1_PORT, LED1_Pin);
# define LED1_ON   HAL_GPIO_WritePin(LED1_PORT, LED1_Pin, GPIO_PIN_RESET);
# define LED2_OFF    HAL_GPIO_WritePin(LED2_PORT, LED2_Pin, GPIO_PIN_SET);
# define LED2_TOGGLE    HAL_GPIO_TogglePin(LED2_PORT, LED2_Pin);
# define LED2_ON   HAL_GPIO_WritePin(LED2_PORT, LED2_Pin, GPIO_PIN_RESET);
# define SD_LED_OFF  HAL_GPIO_WritePin(TF_DET_LED_PORT, TF_DET_LED_PIN, GPIO_PIN_SET);
# define SD_LED_ON  HAL_GPIO_WritePin(TF_DET_LED_PORT, TF_DET_LED_PIN, GPIO_PIN_RESET);

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
