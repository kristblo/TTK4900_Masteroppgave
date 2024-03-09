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
#define CTR_PRD 7200 //2880
#define MTR2 TIM1
#define MTR1 TIM15
#define ENC1 TIM3
#define ENC2 TIM8

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

void process_uart_input(uint8_t* buffer, uint8_t buffer_length)
{
  char delim[] = " ";
  // char debug[64];
  // sprintf(debug, "length: %u\n\r", buffer_length);
  // UART_msg_txt(debug);
  char cpy[(int)buffer_length];
  strcpy(cpy, buffer);

  char *ptr = strtok(cpy, delim);
  char parser[64][64];
  int num_tokens = 0;
  while(ptr != NULL)
  {
    strcpy(parser[num_tokens], ptr);
    ptr = strtok(NULL, delim);
    ++num_tokens;
  }

  //Individual command processing
  char cmdbuf[64];
  if((int)strcmp(parser[0], "M1") == 0)
  {
    UART_msg_txt("\n\rGot M1\n\r");
    process_cmd_M1(parser[1]);
  }
  if((int)strcmp(parser[0], "M2") == 0)
  {
    UART_msg_txt("\n\rGot M2\n\r");
    process_cmd_M2(parser[1]);
  }


  //Clean the command array
  for(int i = 0; i < 64; i++)
  {
    for(int k = 0; k < 64; k++)
    {
      parser[i][k] = NULL;
    }
  }

}

uint16_t MTR1_setpoint = 0x7FFF;
void process_cmd_M1(char* token)
{
  uint16_t enc_setpoint = (uint16_t)atoi(token);
  char output[64];
  sprintf(output, "Setting M1: %d\n\r", enc_setpoint);
  UART_msg_txt(output);
  MTR1_setpoint = enc_setpoint;



}

uint16_t MTR2_setpoint = 0x7FFF;
void process_cmd_M2(char* token)
{
  uint16_t enc_setpoint = (uint16_t)atoi(token);
  char output[64];
  sprintf(output, "Setting M2: %d\n\r", enc_setpoint);
  UART_msg_txt(output);
  MTR2_setpoint = enc_setpoint;



}

void parse_uart_input(char* input, uint8_t* buffer, uint8_t buffer_length, uint8_t* buffer_pos)
{

  uint8_t found_cr = (input[0] == '\r') ? (uint8_t)1 : (uint8_t)0;
  uint8_t found_bs = (input[0] == '\b') ? (uint8_t)1 : (uint8_t)0;
  if(found_cr)
  {
    
    process_uart_input(buffer, *buffer_pos);
    uint8_t pos = 0;
    while (pos < buffer_length)
    {
      buffer[pos] = (uint8_t)0;
      pos++;
    }
    *buffer_pos = 0;
    found_cr = 0;
  }
  else if(found_bs)
  {
    *buffer_pos = (--(*buffer_pos) < 0) ? (uint8_t)0 : --(*buffer_pos);
    char* testbuf[64];
    sprintf(testbuf, "Buffer pos: %d\n\r", *buffer_pos);
    UART_msg_txt(testbuf);
  }
  else
  {
    buffer[*buffer_pos] = input[0];
    *buffer_pos = (++(*buffer_pos))%buffer_length;
  }
}


uint8_t Rx_receive[1];
uint8_t Rx_holding_buffer[64];
uint8_t buffer_pos = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart5, Rx_receive, 1);
  parse_uart_input(Rx_receive, Rx_holding_buffer, 64, &buffer_pos);
}



void whoami_test(){
  HAL_StatusTypeDef ret;
  uint8_t SLAVE_READ = 0x3B;  
  uint8_t SLAVE_WRITE = 0x3A;
  uint8_t WHO_AM_I = 0x0D;
  uint8_t i2cbuf_tx[1] = {WHO_AM_I};
  uint8_t i2cbuf_rx[2] = {0xDE, 0xAD};

  uint8_t CTRL_REG1 = 0x2A;
  uint8_t i2cbuf_setup[2] = {CTRL_REG1, 0x3};
  // ret = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVE_WRITE,i2cbuf_setup, 2, 2000);
  // if(ret != HAL_OK)
  // {
  //   UART_msg_txt("Setup failed\n\r");
  // }

  ret = HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)SLAVE_READ,i2cbuf_tx,1,2000); // Tell slave you want to read
  if(ret != HAL_OK)
  {
    UART_msg_txt("transmit failed\n\r");
  }
  //HAL_Delay(20);
  ret = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)SLAVE_READ, i2cbuf_rx,1,2000);
  if(ret != HAL_OK)
  {
    UART_msg_txt("receive failed\n\r");
  }
  double rxdata = (double)i2cbuf_rx[0];
  char stringbuf[20];
  sprintf(stringbuf, "Got %X on address %X\n\r", i2cbuf_rx[0], i2cbuf_tx[0]);
  UART_msg_txt(stringbuf);


  // if(rxdata == (double)0x1A){
  //   uint8_t result[20];
  //   UART_msg_txt("Got 0x1A on I2C\n\r");

  // }

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

int16_t get_acc_adafruit()
{
  uint8_t OUT_X = 0x1;
  uint8_t OUT_Y = 0x3;
  uint8_t OUT_Z = 0x5;
  uint16_t READ = 0x3B;
  uint16_t WRITE = 0x3A;

  HAL_StatusTypeDef i2cret;
  char stringbuf[20];
  uint8_t txbuf[1] = {OUT_X};
  uint8_t rxbuf[1];

  i2cret = HAL_I2C_Master_Transmit(&hi2c1, READ, txbuf, 1, 100);
  if(i2cret != HAL_OK)
  {
    UART_msg_txt("Transmit failed\n\r");
  }
  i2cret = HAL_I2C_Master_Receive(&hi2c1, READ, rxbuf, 1, 100);
  if(i2cret != HAL_OK)
  {
    UART_msg_txt("Receive failed\n\r");
  }

  int8_t accval = (int8_t)rxbuf[0];

  return accval;


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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == OPT_SW1_Pin);
  {
    UART_msg_txt("Opt switch triggered\n\r");
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t *data = "Hello world from USB CDC\n";

////CAN setup
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox[3];
uint8_t TxData[8];
uint8_t RxData[8];
uint8_t CAN_count = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  char stringbuf[32];
  sprintf(stringbuf, "RxData: 0x%X\n\r", RxData[0]);
  UART_msg_txt(stringbuf);
  CAN_count++; //Mistenker at dette ikke funker, se HAL UM1786 s.92
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
  UART_msg_txt("Main begin\n\r");
  HAL_Delay(1000);

  
  HAL_UART_Receive_IT(&huart5, Rx_receive, 1);
  int num = 0;
  char str[20];

  //Accelerometer setup
  // uint8_t CTRL1_XL = 0x10;
  // uint8_t initbuf[2] = {CTRL1_XL, 0b01010000};
  // uint8_t SLAVE_WRITE = 0xD4;
  // uint8_t SLAVE_READ = 0xD5;
  // HAL_StatusTypeDef ret;
  // ret = HAL_I2C_Master_Transmit(&hi2c3,(uint16_t)SLAVE_WRITE,initbuf,2,100);
  // if(ret != HAL_OK)
  // {
  //   UART_msg_txt("Accelerometer setup failed\n\r");
  // }
  
  // HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVE_READ, initbuf, 1,100);
  // uint8_t i2cbuf_rx[1];
  // ret = HAL_I2C_Master_Receive(&hi2c3, (uint16_t)SLAVE_READ, i2cbuf_rx,1,100);
  // if(ret != HAL_OK)
  // {
  //   UART_msg_txt("Could not read CTRL1_XL");
  // }
  // int ctrl_readback = (int)i2cbuf_rx[0];
  // char stringbuf[20];
  // sprintf(stringbuf, "CTRL1_XL: 0x%X\n\r", ctrl_readback);
  // UART_msg_txt(stringbuf);
  //End accelerometer setup

  //Start adafruit accelerometer setup
  uint8_t CTRL_REG1 = 0x2A;
  uint8_t i2cbuf_setup[2] = {CTRL_REG1, 0x3};
  uint16_t WRITE = 0x3A;
  uint16_t READ = 0x3B;
  HAL_StatusTypeDef ret;
  ret = HAL_I2C_Master_Transmit(&hi2c1, WRITE,i2cbuf_setup, 2, 2000);
  if(ret != HAL_OK)
  {
    UART_msg_txt("Setup failed: write\n\r");
  }
  ret = HAL_I2C_Master_Transmit(&hi2c1, READ,i2cbuf_setup, 1, 2000);
  if(ret != HAL_OK)
  {
    UART_msg_txt("Setup failed: tx read\n\r");
  }
  ret = HAL_I2C_Master_Receive(&hi2c1, READ,i2cbuf_setup, 1, 2000);
  if(ret != HAL_OK)
  {
    UART_msg_txt("Setup failed: rx read\n\r");
  }


  HAL_Delay(2000);
  //End adafruit accelerometer setup

  //Timer setup
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//MTR2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//MTR2
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);//MTR1
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);//MTR1
  //Encoder startup
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //ENC1
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); //ENC2


  //CAN init
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  TxHeader.DLC = 1;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x107; //Transmit ID
  TxHeader.TransmitGlobalTime = DISABLE;

  TxData[0] = 0xDE;
  TxData[1] = 0xAD;
  TxData[2] = 0xBB;
  //TxData[3] = 0x03;
  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData[0], &TxMailbox[0]) != HAL_OK)
  {
    Error_Handler();
    UART_msg_txt("Mailbox 0 not OK\n\r");
  }
  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData[1], &TxMailbox[1]) != HAL_OK)
  {
    Error_Handler();
    UART_msg_txt("Mailbox 1 not OK\n\r");
  }
  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData[2], &TxMailbox[2]) != HAL_OK)
  {
    Error_Handler();
    UART_msg_txt("Mailbox 2 not OK\n\r");
  }
  // if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData[3], &TxMailbox[3]) != HAL_OK)
  // {
  //   Error_Handler();
  //   UART_msg_txt("Mailbox 3 not OK\n\r");
  // }


  HAL_Delay(10);
  char can_stringbuf[32];
  sprintf(can_stringbuf, "CAN counter: %d\n\r", CAN_count);
  UART_msg_txt(can_stringbuf);

  //Encoder loop setup
  uint16_t MID = 0x7FFF;
  uint16_t STA = MID - 10000;
  uint16_t END = MID + 10000;
  ENC1->CNT = MID;
  int setpoint = MID;
  int fwd = 1;
  ENC2->CNT = MID;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(IMU_INT1_GPIO_Port, IMU_INT1_Pin, 0);
  HAL_GPIO_WritePin(IMU_INT2_GPIO_Port, IMU_INT2_Pin, 0);
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, 1);
  UART_msg_txt("Loop begin\n\r");
  HAL_Delay(1000);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(10);
    
    
    ////Relay test, first contact
    if(0)
    {
      HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin,1);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin,0);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin,1);
    }

    
    ////UART test
    if(1)
    {
      sprintf(str, "%d", num);
      //UART_msg_txt(str);
      //HAL_Delay(200);
      UART_msg_txt("\33[2K\r");
      //num++;
      //HAL_UART_Receive(&huart5, Rx_data,4,1000);
      char* testbuf = "\33[2K\r";
      HAL_UART_Transmit(&huart5, Rx_holding_buffer, 64, HAL_MAX_DELAY);
      
    }
    
    ////Accelerometer test
    if(0)
    {
      whoami_test();
      //UART_msg_txt("Accelerometer: ");
      //int acc = (int)get_acc_z();
      //sprintf(str, "%u", acc);
      //UART_msg_txt(str);
      //UART_msg_txt("\n\r");
    }

    ////Adafruit acc test
    if(0)
    {
      char stringbuf[20];
      int16_t accval = get_acc_adafruit();
      sprintf(stringbuf, "Acceleration: %d\n\r", accval);
      UART_msg_txt(stringbuf);
    }

    ////Motor driver test
    if(0)
    {
      char number[32];
      uint32_t delay = 50;
      double max_pct = 20;
      double pct = 0;
      
      while(pct < max_pct)
      {
        GoFWD(pct, MTR1);
        GoFWD(pct, MTR2);
        ++pct;
        HAL_Delay(delay);        
        int current = (TIM1->CNT);
        sprintf(number, "MTRCCR1: %d\n\r", current);
        UART_msg_txt(number);
        
      }
      while(pct > 0){
        GoFWD(pct, MTR1);
        GoFWD(pct, MTR2);
        --pct;
        HAL_Delay(delay);
      }
      while (pct < max_pct)
      {
        GoBWD(pct, MTR1);
        GoBWD(pct, MTR2);
        ++pct;
        HAL_Delay(delay);
      }
      while (pct > 0)
      {
        GoBWD(pct, MTR1);
        GoBWD(pct, MTR2);
        --pct;
        HAL_Delay(delay);
      }
      
    }

    ///Motor encoder test
    if(0)
    {
      //Waving setpoint generation START
      if(setpoint < END && fwd)
      {
        setpoint += 100;
      }
      else if (setpoint >= END && fwd)
      {
        fwd = 0;
        setpoint -= 100;
      }
      else if (setpoint > STA && !fwd)
      {
        setpoint -= 100;
      }
      else if (setpoint <= STA && !fwd)
      {
        fwd = 1;
        setpoint += 100;
      }
      //Waving setpoint generation END

      //print setpoint
      char enc_stringbuf[16];
      sprintf(enc_stringbuf, "S: %d, P: %u\n\r", setpoint, ENC1->CNT);
      //sprintf(enc_stringbuf, "Setpoint: %d\n\r", MTR1_setpoint);
      UART_msg_txt(enc_stringbuf);
      sprintf(enc_stringbuf, "\33[2K\r");
      UART_msg_txt(enc_stringbuf);


      double pos = (double)(ENC1->CNT);
      double e = (double)MTR1_setpoint - pos;
      double padrag = e*0.05; //p-reg
      int safe_cap_pct = 20;
      if(abs(padrag) > safe_cap_pct) //100% cutoff
      {
        if(padrag > 0)
        {
          padrag = safe_cap_pct;
        }
        else
        {
          padrag = -safe_cap_pct;
        }
      }
      if(padrag > 0)
      {
        GoFWD(abs(padrag), MTR1);
      }
      else if(padrag < 0)
      {
        GoBWD(abs(padrag), MTR1);
      }
    }

    ///Concurrent motor control
    if(1)
    {
      //MTR1
      double mtr1_pos = (double)(ENC1->CNT);
      double mtr1_e = (double)MTR1_setpoint - mtr1_pos;
      double mtr1_pwr = mtr1_e*0.05;
      int mtr1_cap = 20;
      if(abs(mtr1_pwr) > mtr1_cap)
      {
        if(mtr1_pwr > 0)
        {
          mtr1_pwr = mtr1_cap;
        }
        else
        {
          mtr1_pwr = -mtr1_cap;
        }
      }
      if(mtr1_pwr > 0)
      {
        GoFWD(abs(mtr1_pwr), MTR1);
      }
      else if(mtr1_pwr < 0)
      {
        GoBWD(abs(mtr1_pwr), MTR1);
      }
      // char printbuf[32];
      // sprintf(printbuf, "POS1: %u\n\r", ENC1->CNT);
      // UART_msg_txt(printbuf);


      //MTR2
      double mtr2_pos = (double)(ENC2->CNT);
      double mtr2_e = (double)MTR2_setpoint - mtr2_pos;
      double mtr2_pwr = mtr2_e*0.05;
      int mtr2_cap = 10;
      if(abs(mtr2_pwr) > mtr2_cap)
      {
        if(mtr2_pwr > 0)
        {
          mtr2_pwr = mtr2_cap;
        }
        else
        {
          mtr2_pwr = -mtr2_cap;
        }
      }
      if(mtr2_pwr > 0)
      {
        GoBWD(abs(mtr2_pwr), MTR2);
      }
      else if(mtr2_pwr < 0)
      {
        GoFWD(abs(mtr2_pwr), MTR2);
      }
      // char printbuf[32];
      // sprintf(printbuf, "POS2: %u\n\r", ENC2->CNT);
      // UART_msg_txt(printbuf);
    }

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
