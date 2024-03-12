/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_can.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h> 
#include "can_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_SENDER_ID 0x10C
#define CAN_RECPNT_ID 0x10A
#define MTR1 TIM15
#define MTR2 TIM1
#define CTR_PRD 7200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void UART_msg_txt(char* txt)
{
  int len = 0;
  char* cp = txt;
  while(*cp)
  {
    len++;
    cp++;
  }

  HAL_UART_Transmit(&huart5, (uint8_t*)txt, len, HAL_MAX_DELAY);
}
void SetPWD_DT(uint32_t* timer_counter, double pct)
{
  int ticks = (pct / 100) * CTR_PRD;
  *timer_counter = ticks;
}

void GoFWD(double pct, TIM_TypeDef* mtr)
{
  SetPWD_DT(&(mtr->CCR1), 100);
  SetPWD_DT(&(mtr->CCR3), 100);
  SetPWD_DT(&(mtr->CCR2), 100-pct);

}

void GoBWD(double pct, TIM_TypeDef* mtr)
{
  SetPWD_DT(&(mtr->CCR1), 100-pct);
  SetPWD_DT(&(mtr->CCR3), 100-pct);
  SetPWD_DT(&(mtr->CCR2), 100);

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CAN_TxHeaderTypeDef TxHeader;
// CAN_RxHeaderTypeDef RxHeader;

// uint32_t TxMailbox[3];

uint8_t TxData[8];
uint8_t RxData[8];

uint8_t count = 0;

void SendCanMsg(uint8_t* data)
{
  CAN_TxHeaderTypeDef TxHeaderInternal;
  uint32_t TxMailboxInternal[3];
  uint8_t TxDataInternal[8];

  TxHeaderInternal.DLC = 2;
  TxHeaderInternal.ExtId = 0;
  TxHeaderInternal.IDE = CAN_ID_STD;
  TxHeaderInternal.RTR = CAN_RTR_DATA;
  TxHeaderInternal.StdId = CAN_SENDER_ID;
  TxHeaderInternal.TransmitGlobalTime = DISABLE;

  TxDataInternal[0] = data[0];
  TxDataInternal[1] = data[1];

  HAL_CAN_AddTxMessage(&hcan, &TxHeaderInternal, TxDataInternal, &TxMailboxInternal[0]);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == END_SW_Pin)
  {
    UART_msg_txt("End switch triggered\n\r");
    TxData[0] = 100;
    TxData[1] = 10;

    //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox[0]);
    SendCanMsg(TxData);
  }
  if(GPIO_Pin == OPT_SW1_Pin)
  {
    UART_msg_txt("Twist switch triggered\n\r");
    TxData[0] = 100;
    TxData[1] = 10;

    //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox[0]);
    SendCanMsg(TxData);
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
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//MTR2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//MTR2
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);//MTR1
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);//MTR1
  
  UART_msg_txt("Hello world\n\r");


  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  // TxHeader.DLC = 2;
  // TxHeader.ExtId = 0;
  // TxHeader.IDE = CAN_ID_STD;
  // TxHeader.RTR = CAN_RTR_DATA;
  // TxHeader.StdId = 0x10A;
  // TxHeader.TransmitGlobalTime = DISABLE;

  TxData[0] = 0xDE;
  TxData[1] = 0xAD;
  TxData[2] = 0xBB;
  HAL_StatusTypeDef ret;
  
  // if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData[0], &TxMailbox[0]) != HAL_OK)
  // {
  //   Error_Handler();
  //   UART_msg_txt("TxMailbox 0 not OK\n\r");
  // }
  // if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData[1], &TxMailbox[1]) != HAL_OK)
  // {
  //   Error_Handler();
  //   UART_msg_txt("TxMailbox 1 not OK\n\r");
  // }
  // if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData[2], &TxMailbox[2]) != HAL_OK)
  // {
  //   Error_Handler();
  //   UART_msg_txt("TxMailbox 2 not OK\n\r");
  // }

  
  HAL_Delay(100);
  char* stringbuf[64];
  sprintf(stringbuf, "CAN counter: %u\n\r", count);
  UART_msg_txt(stringbuf);
  sprintf(stringbuf, "CAN msg: %u\n\r", RxData[0]);
  UART_msg_txt(stringbuf);

  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
