/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "tim.h"
#include "i2c.h"
#include "hardwaresdrv.h"
#include "sht_config.h"
#include "mimiciic.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOA
#define SHT2_SDA_Pin GPIO_PIN_3
#define SHT2_SDA_GPIO_Port GPIOA
#define SHT2_SCL_Pin GPIO_PIN_4
#define SHT2_SCL_GPIO_Port GPIOA
#define SHT3_SCL_Pin GPIO_PIN_10
#define SHT3_SCL_GPIO_Port GPIOB
#define SHT3_SDA_Pin GPIO_PIN_11
#define SHT3_SDA_GPIO_Port GPIOB
#define SHT1_SCL_Pin GPIO_PIN_6
#define SHT1_SCL_GPIO_Port GPIOB
#define SHT1_SDA_Pin GPIO_PIN_7
#define SHT1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
