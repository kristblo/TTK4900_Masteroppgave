#include "can_driver.h"
#if GLOBAL_DEBUG
  #include "uart_driver.h"
#endif

void can_send_msg_base(uint8_t* data, 
                        int stdId,
                        int dlc,
                        uint8_t mailbox)
{
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox[3];
  
  txHeader.DLC = 8;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = stdId;
  txHeader.TransmitGlobalTime = DISABLE;

  if(HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox[mailbox]) != HAL_OK)
  {
    uart_send_string("CAN TX not successful");
  }
#if GLOBAL_DEBUG
  //char* debugbuffer[128];
  //sprintf(debugbuffer, "DLC: %u\n\r TXData: %s\n\r",txHeader.DLC, data);
  // for(int i = 0; i < txHeader.DLC; i++)
  // {
  //   sprintf(debugbuffer, "TXdata %i: %X\n\r", i, data[i]);
  //   uart_send_string(debugbuffer);
  // }
#endif

}

void can_send_msg_wrp(can_send_msg_args* input)
{
  int dlc_out = input->dlc ? input->dlc : 8;
  int stdId_out = input->stdId ? input->stdId : CAN_TXID;
  int mailbox_out = input->mailbox && input->mailbox < 3 ? input->mailbox : 0;

  can_send_msg_base(input->data,
                    CAN_TXID,
                    dlc_out,
                    mailbox_out);
}

void can_rx_handler(uint8_t* data)
{
  uart_send_string("Got a CAN message\n\r");

  if(data[0] == 'M')
  {
    uint8_t motorId = (data[1] & 0xF0) >> 4;
    uint8_t command = data[1] & 0x0F;
    int32_t argument;
    memcpy(&argument, &data[2], 4);
    uint8_t motorSelect = 255;

#if ACTIVE_UNIT == SHOULDER
    if(motorId == 2)
    {
      motorSelect = 1;
    }
    else if(motorId == 3)
    {
      motorSelect = 0;
    }
#else if ACTIVE_UNIT == HAND
    if(motorId == 4)
    {
      motorSelect = 1;
    }
    else if(motorId == 5)
    {
      motorSelect = 0;
    }
#endif

    if(command == 1)
    {
      motor_interface_set_power(motorSelect, 1, (double)argument);
    }
    else if(command == 2)
    {
      motor_interface_set_power(motorSelect, 0, (double)argument);
    }
    else if(command == 3)
    {
      motor_interface_set_setpoint(motorSelect, argument);
    }
  }


#if GLOBAL_DEBUG
  // char* debugbuffer[128];
  // for(int i = 0; i < 8; i++)
  // {
  //   sprintf(debugbuffer, "Data rx %i: %X\n\r", i, data[i]);
  //   uart_send_string(debugbuffer);
  // }
#endif

}

//Declared in stm32fxx_hal_can.h
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t rxData[8];
  CAN_RxHeaderTypeDef rxHeader;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

#if GLOBAL_DEBUG
  char* debugbuffer[128];
  sprintf(debugbuffer, "DLC: %i\n\r RXData: %s\n\r",rxHeader.DLC, rxData);
  uart_send_string(debugbuffer);
#endif

  can_rx_handler(rxData);
}
