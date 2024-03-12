#include "can_driver.h"
#define MTR2 TIM1











void can_rx_handler(uint8_t* inputData)
{
    char* outputbuf[64];
    sprintf(outputbuf, "Data0: %u, Data1: %u\n\r", inputData[0], inputData[1]);
    UART_msg_txt(outputbuf);
    

    if(MTR2->CCR1 == 0)
    {
      GoFWD(20, MTR2);
    }
    else{
      GoFWD(0, MTR2);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeaderInternal;
  uint8_t RxDataInternal[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeaderInternal, RxDataInternal);

  char* debugbuf[64];
  sprintf(debugbuf, "Data length: %u\n\r", RxHeaderInternal.DLC);
  UART_msg_txt(debugbuf);

  can_rx_handler(RxDataInternal);

}
