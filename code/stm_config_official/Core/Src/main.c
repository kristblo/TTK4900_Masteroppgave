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
#include "usbd_cdc_if.h"
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CTR_PRD 2880

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

uint8_t Rx_data[10];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart5, Rx_data, 4);
}

void whoami_test(){
  HAL_StatusTypeDef ret;
  uint8_t SLAVE_WRITE = 0xD4;
  uint8_t SLAVE_READ = 0xD5;  
  uint8_t WHO_AM_I = 0x0F;
  uint8_t i2cbuf_tx[1] = {WHO_AM_I};
  uint8_t i2cbuf_rx[2];


  ret = HAL_I2C_Master_Transmit(&hi2c3,(uint16_t)SLAVE_READ,i2cbuf_tx,1,2000); // Tell slave you want to read
  if(ret != HAL_OK)
  {
    UART_msg_txt("transmit failed, check pullup in hal_i2c_mspinit\n\r");
  }
  //HAL_Delay(20);
  ret = HAL_I2C_Master_Receive(&hi2c3, (uint16_t)SLAVE_READ, i2cbuf_rx,1,2000);
  if(ret != HAL_OK)
  {
    UART_msg_txt("receive failed, check pullup in hal_i2c_mspinit\n\r");
  }
  double rxdata = (double)i2cbuf_rx[0];
  if(rxdata == (double)0x6A){
  //Blink LED 3 times
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
  // HAL_Delay(100);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
  // HAL_Delay(500);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
  // HAL_Delay(100);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
  // HAL_Delay(500);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
  // HAL_Delay(100);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
  // HAL_Delay(500);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
    uint8_t result[20];
    UART_msg_txt("Got 0x6A on I2C");

  }

}

int16_t get_acc_z(void){
  uint8_t OUTZ_L_XL = 0x2C;
  uint8_t OUTZ_H_XL = 0x2D;
  uint8_t OUTX_L_XL = 0x28;
  uint8_t OUTX_H_XL = 0x29;  
  uint8_t READ = 0xD5;
  uint8_t WRITE = 0xD4;

  uint8_t txbufh[1] = {OUTX_H_XL};
  uint8_t txbufl[1] = {OUTX_L_XL};
  uint8_t rxbufh[1];
  uint8_t rxbufl[1];

  HAL_StatusTypeDef i2cret;
  //receive high byte
  char stringbuf[20];
  i2cret = HAL_I2C_Master_Transmit(&hi2c3,(uint16_t)READ, txbufh,1,100);
  if(i2cret != HAL_OK)
  {
    UART_msg_txt("high transmit failed\n\r");
  }
  i2cret = HAL_I2C_Master_Receive(&hi2c3, (uint16_t)READ, rxbufh,1,100);
  if(i2cret != HAL_OK)
  {
    UART_msg_txt("high receive failed\n\r");
  }


  //receive low byte
  i2cret = HAL_I2C_Master_Transmit(&hi2c3,(uint16_t)READ, txbufl,1,100);
  if(i2cret != HAL_OK)
  {
    UART_msg_txt("low transmit failed\n\r");
  }
  i2cret = HAL_I2C_Master_Receive(&hi2c3, (uint16_t)READ, rxbufl,1,100);
  if(i2cret != HAL_OK)
  {
    UART_msg_txt("low receive failed\n\r");
  }
  sprintf(stringbuf, "High byte: %X\n\r", rxbufh[0]);
  UART_msg_txt(stringbuf);
  sprintf(stringbuf, "Low byte: %X\n\r", rxbufl[0]);
  UART_msg_txt(stringbuf);  

  
  int16_t accval = (int16_t)(rxbufh[0]<<8|rxbufl[1]);

  return accval;


}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t *data = "Hello world from USB CDC\n";
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
  HAL_UART_Receive_IT(&huart5, Rx_data, 4);
  int num = 0;
  char str[20];

  //Accelerometer setup
  uint8_t CTRL1_XL = 0x10;
  uint8_t initbuf[2] = {CTRL1_XL, 0b01010000};
  uint8_t SLAVE_WRITE = 0xD4;
  uint8_t SLAVE_READ = 0xD5;
  HAL_StatusTypeDef ret;
  ret = HAL_I2C_Master_Transmit(&hi2c3,(uint16_t)SLAVE_WRITE,initbuf,2,100);
  if(ret != HAL_OK)
  {
    UART_msg_txt("Accelerometer setup failed");
  }
  
  HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)SLAVE_READ, initbuf, 1,100);
  uint8_t i2cbuf_rx[1];
  ret = HAL_I2C_Master_Receive(&hi2c3, (uint16_t)SLAVE_READ, i2cbuf_rx,1,100);
  if(ret != HAL_OK)
  {
    UART_msg_txt("Could not read CTRL1_XL");
  }
  int ctrl_readback = (int)i2cbuf_rx[0];
  char stringbuf[20];
  sprintf(stringbuf, "CTRL1_XL: 0x%X\n\r", ctrl_readback);
  UART_msg_txt(stringbuf);

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  UART_msg_txt("Program begin\n\r");
  HAL_GPIO_WritePin(IMU_INT1_GPIO_Port, IMU_INT1_Pin, 0);
  HAL_GPIO_WritePin(IMU_INT2_GPIO_Port, IMU_INT2_Pin, 0);
  HAL_Delay(2000);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    ////Accelerometer
    whoami_test();
    UART_msg_txt("Accelerometer: ");
    int acc = (int)get_acc_z();
    sprintf(str, "%u", acc);
    UART_msg_txt(str);
    UART_msg_txt("\n\r");
    
    
    HAL_Delay(100);
    sprintf(str, "%d", num);
    // HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin,1);
    // HAL_Delay(1000);
    // HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin,0);

    //uint8_t buffer[] = "Hello, world! USB\r\n";
    //CDC_Transmit_FS(data, strlen(data));
    //UART_msg_txt("Hello world\n\r");
    UART_msg_txt(str);
    num++;
    // HAL_UART_Receive(&huart5, Rx_data,4,1000);
    // UART_msg_txt(Rx_data);

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
