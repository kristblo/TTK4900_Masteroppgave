/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_EN_Pin GPIO_PIN_13
#define CAN_EN_GPIO_Port GPIOC
#define RELAY_EN_Pin GPIO_PIN_0
#define RELAY_EN_GPIO_Port GPIOC
#define DRV2_IN1_Pin GPIO_PIN_1
#define DRV2_IN1_GPIO_Port GPIOC
#define DRV2_IN2_Pin GPIO_PIN_2
#define DRV2_IN2_GPIO_Port GPIOC
#define DRV2_VIP_Pin GPIO_PIN_3
#define DRV2_VIP_GPIO_Port GPIOC
#define DRV1_IN1_Pin GPIO_PIN_2
#define DRV1_IN1_GPIO_Port GPIOA
#define DRV1_IN2_Pin GPIO_PIN_3
#define DRV1_IN2_GPIO_Port GPIOA
#define DRV1_VIP_Pin GPIO_PIN_4
#define DRV1_VIP_GPIO_Port GPIOA
#define END_SW_Pin GPIO_PIN_5
#define END_SW_GPIO_Port GPIOA
#define END_SW_EXTI_IRQn EXTI9_5_IRQn
#define IMU_INT2_Pin GPIO_PIN_12
#define IMU_INT2_GPIO_Port GPIOB
#define IMU_INT1_Pin GPIO_PIN_13
#define IMU_INT1_GPIO_Port GPIOB
#define ENC2B_Pin GPIO_PIN_6
#define ENC2B_GPIO_Port GPIOC
#define ENC2A_Pin GPIO_PIN_7
#define ENC2A_GPIO_Port GPIOC
#define USB_VB_Pin GPIO_PIN_9
#define USB_VB_GPIO_Port GPIOA
#define USB_VB_EXTI_IRQn EXTI9_5_IRQn
#define USB_DP_DRV_Pin GPIO_PIN_10
#define USB_DP_DRV_GPIO_Port GPIOA
#define END_SW_BAK_Pin GPIO_PIN_15
#define END_SW_BAK_GPIO_Port GPIOA
#define END_SW_BAK_EXTI_IRQn EXTI15_10_IRQn
#define OPT_SW2_Pin GPIO_PIN_10
#define OPT_SW2_GPIO_Port GPIOC
#define OPT_SW2_EXTI_IRQn EXTI15_10_IRQn
#define OPT_SW1_Pin GPIO_PIN_11
#define OPT_SW1_GPIO_Port GPIOC
#define OPT_SW1_EXTI_IRQn EXTI15_10_IRQn
#define ENC1A_Pin GPIO_PIN_4
#define ENC1A_GPIO_Port GPIOB
#define ENC1B_Pin GPIO_PIN_5
#define ENC1B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
