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

#include "unit_config.h"
#include "can_driver.h"
#include "gpio_driver.h"
#include "uart_driver.h"
#include "motor_driver.h"
#include "string_cmd_parser.h"
#include "shoulder_controller.h"

#if ACTIVE_UNIT == SHOULDER
#include "accelerometer_driver.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

// void whoami_test(){
//   HAL_StatusTypeDef ret;
//   uint8_t SLAVE_READ = 0xD4;  
//   uint8_t SLAVE_WRITE = 0xD5;
//   uint8_t WHO_AM_I = 0x0F;
//   uint8_t i2cbuf_tx[1] = {WHO_AM_I};
//   uint8_t i2cbuf_rx[2] = {0xDE, 0xAD};

//   // uint8_t CTRL_REG1 = 0x2A;
//   // uint8_t i2cbuf_setup[2] = {CTRL_REG1, 0x3};
//   // ret = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVE_WRITE,i2cbuf_setup, 2, 2000);
//   // if(ret != HAL_OK)
//   // {
//   //   UART_msg_txt("Setup failed\n\r");
//   // }

//   ret = HAL_I2C_Master_Transmit(&hi2c3,(uint16_t)SLAVE_READ,i2cbuf_tx,1,2000); // Tell slave you want to read
//   if(ret != HAL_OK)
//   {
//     uart_send_string("transmit failed\n\r");
//   }
//   //HAL_Delay(20);
//   ret = HAL_I2C_Master_Receive(&hi2c3, (uint16_t)SLAVE_READ, i2cbuf_rx,1,2000);
//   if(ret != HAL_OK)
//   {
//     uart_send_string("receive failed\n\r");
//   }
//   double rxdata = (double)i2cbuf_rx[0];
//   char stringbuf[20];
//   sprintf(stringbuf, "Got %X on address %X\n\r", i2cbuf_rx[0], i2cbuf_tx[0]);
//   uart_send_string(stringbuf);


//   // if(rxdata == (double)0x1A){
//   //   uint8_t result[20];
//   //   UART_msg_txt("Got 0x1A on I2C\n\r");

//   // }

// }

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  uart_send_string("Hello world\n\r");
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//MTR2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//MTR2
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);//MTR1
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);//MTR1

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  

#if (HW_INTERFACE == UART_INTERFACE)  && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_hmi_init();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(2000);
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, 1);

  //motor_interface_controller_init(1);
  //motor_interface_controller_init(0);
  //controller_interface_request_position();

#if ACTIVE_UNIT == SHOULDER
  accl_interface_set_byte(0x10, 0b01110000); //Init acc 2g
  accl_interface_set_byte(0x11, 0b01110000); //Init rotation 250dps
#endif

  uint8_t canData[8] = {0x41, 0x1, 0x2A, 0x0, 0x0, 0x0, 0, 0};
  uint8_t canrx[8];

  while (1)
  {

    // uart_send_string("not dead while\n\r");
    
    // controller_interface_update_position();

    // if(can_interface_get_newrxflag() == 0)
    // {
    //   controller_interface_request_position();
    // }

    // HAL_Delay(200);
    can_rx_executive();
    can_tx_executive();

    //HAL_Delay(200);
    // // controller_interface_update_power();
    // if(can_interface_get_newrxflag() == 1)
    // {
    //   uint8_t mailbox[8];
    //   can_interface_get_acc_rxMailbox(mailbox);
    //   int16_t test = (int16_t)((mailbox[2]<<8)|mailbox[3]);
    //   char* debug[64];
    //   sprintf(debug, "Decoded rx: %i\n\r", test);
    //   uart_send_string(debug);
    //   can_interface_clear_newrx();
    // }


    motor_interface_update_power(0);
    motor_interface_update_tot_cnt(0);    
    motor_interface_update_power(1);
    motor_interface_update_tot_cnt(1);


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
