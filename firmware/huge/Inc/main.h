/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32h7xx_hal.h"

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
#define GPIO0_Pin GPIO_PIN_0
#define GPIO0_GPIO_Port GPIOF
#define GPIO1_Pin GPIO_PIN_1
#define GPIO1_GPIO_Port GPIOF
#define GPIO2_Pin GPIO_PIN_2
#define GPIO2_GPIO_Port GPIOF
#define GPIO3_Pin GPIO_PIN_3
#define GPIO3_GPIO_Port GPIOF
#define GPIO4_Pin GPIO_PIN_4
#define GPIO4_GPIO_Port GPIOF
#define GPIO5_Pin GPIO_PIN_5
#define GPIO5_GPIO_Port GPIOF
#define GPIO6_Pin GPIO_PIN_6
#define GPIO6_GPIO_Port GPIOF
#define GPIO7_Pin GPIO_PIN_7
#define GPIO7_GPIO_Port GPIOF
#define GPIO8_Pin GPIO_PIN_8
#define GPIO8_GPIO_Port GPIOF
#define GPIO9_Pin GPIO_PIN_9
#define GPIO9_GPIO_Port GPIOF
#define GPIO10_Pin GPIO_PIN_10
#define GPIO10_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOF
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOF
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOF
#define LED5_Pin GPIO_PIN_15
#define LED5_GPIO_Port GPIOF
#define LED6_Pin GPIO_PIN_0
#define LED6_GPIO_Port GPIOG
#define LED7_Pin GPIO_PIN_1
#define LED7_GPIO_Port GPIOG
#define EXT_RX_Pin GPIO_PIN_8
#define EXT_RX_GPIO_Port GPIOD
#define EXT_TX_Pin GPIO_PIN_9
#define EXT_TX_GPIO_Port GPIOD
#define CONTROL1_Pin GPIO_PIN_2
#define CONTROL1_GPIO_Port GPIOG
#define CONTROL2_Pin GPIO_PIN_3
#define CONTROL2_GPIO_Port GPIOG
#define CONTROL3_Pin GPIO_PIN_4
#define CONTROL3_GPIO_Port GPIOG
#define CONTROL4_Pin GPIO_PIN_5
#define CONTROL4_GPIO_Port GPIOG
#define CONTROL5_Pin GPIO_PIN_6
#define CONTROL5_GPIO_Port GPIOG
#define CONTROL6_Pin GPIO_PIN_7
#define CONTROL6_GPIO_Port GPIOG
#define CONTROL7_Pin GPIO_PIN_8
#define CONTROL7_GPIO_Port GPIOG
#define CONTROL8_Pin GPIO_PIN_8
#define CONTROL8_GPIO_Port GPIOA
#define HOST_RX_Pin GPIO_PIN_9
#define HOST_RX_GPIO_Port GPIOA
#define HOST_TX_Pin GPIO_PIN_10
#define HOST_TX_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
