/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "unit_config.h"

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef mtrFilterConfig;
  mtrFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  mtrFilterConfig.FilterBank = 0;
  mtrFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  mtrFilterConfig.FilterIdHigh = CAN_FILTER_M << 5;
  mtrFilterConfig.FilterIdLow = 0x0000;
  mtrFilterConfig.FilterMaskIdHigh = CAN_FILTERMASK_M << 5;//0s are DC when incoming IDs are compared
  mtrFilterConfig.FilterMaskIdLow = 0x0000;
  mtrFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  mtrFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  mtrFilterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &mtrFilterConfig);

  CAN_FilterTypeDef accFilterConfig;
  accFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  accFilterConfig.FilterBank = 1;
  accFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  accFilterConfig.FilterIdHigh = CAN_FILTER_A << 5;
  accFilterConfig.FilterIdLow = 0x0000;
  accFilterConfig.FilterMaskIdHigh = CAN_FILTERMASK_A << 5;//0s are DC when incoming IDs are compared
  accFilterConfig.FilterMaskIdLow = 0x0000;
  accFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  accFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  accFilterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &accFilterConfig);

  CAN_FilterTypeDef gblFilterConfig;
  gblFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  gblFilterConfig.FilterBank = 2;
  gblFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  gblFilterConfig.FilterIdHigh = CAN_FILTER_G << 5;
  gblFilterConfig.FilterIdLow = 0x0000;
  gblFilterConfig.FilterMaskIdHigh = CAN_FILTERMASK_G << 5;
  gblFilterConfig.FilterMaskIdLow = 0x0000;
  gblFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  gblFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  gblFilterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &gblFilterConfig);

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN interrupt Deinit */
  /* USER CODE BEGIN CAN:USB_LP_CAN_RX0_IRQn disable */
    /**
    * Uncomment the line below to disable the "USB_LP_CAN_RX0_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USB_LP_CAN_RX0_IRQn); */
  /* USER CODE END CAN:USB_LP_CAN_RX0_IRQn disable */

  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
