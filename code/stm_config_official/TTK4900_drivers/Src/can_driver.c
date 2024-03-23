#include "can_driver.h"
#if GLOBAL_DEBUG
  #include "uart_driver.h"
#endif



const can_message_types numCanTypes = num_types;
#define NUM_CAN_TYPES num_types //Compiler gets sad if numCanTypes is used directly

void (*canRxFunctions[NUM_CAN_TYPES])() =
{
  can_driver_cmd_rx0,
  can_driver_cmd_rx1,
  can_driver_cmd_rx2,
  can_driver_cmd_rx3,
  can_driver_cmd_rx4,
  can_driver_cmd_rx5,
  can_driver_cmd_rx6,
  can_driver_cmd_rx7,
  can_driver_cmd_rx8,
};

static can_mailbox rxMailboxes[NUM_CAN_TYPES]; 
static can_mailbox txMailboxes[NUM_CAN_TYPES];

void can_interface_queue_tx(uint8_t mailbox, uint8_t* outData, uint32_t id)
{
  can_driver_queue_tx(&txMailboxes[mailbox], outData, id);
}

void can_rx_executive()
{
  for(int i = 0; i < numCanTypes; i++)
  {
    if(can_mailbox_get_flag(&rxMailboxes[i]))
    {      
      uint8_t data[8];
      can_mailbox_get_data(&rxMailboxes[i], data);
      
      char* debug[64];
      for(int i = 0; i < 8; i++)
      {
        sprintf(debug, "rx exec data %i: %X\n\r",i, data[i]);
        uart_send_string(debug);
      }



      uint16_t id = can_mailbox_get_id(&rxMailboxes[i]);
      canRxFunctions[i](id, data);
      can_mailbox_clear_flag(&rxMailboxes[i]);
    }
  }
}

void can_tx_executive()
{
  for(int i = 0; i < numCanTypes; i++)
  {  
    if(can_mailbox_get_flag(&txMailboxes[i]) == 1)
    {     
      can_driver_send_msg(&txMailboxes[i].data,
                              can_mailbox_get_id(&txMailboxes[i]),
                              8,
                              0);
      can_mailbox_clear_flag(&txMailboxes[i]);
    }
  }  
}

void can_mailbox_set_data(can_mailbox* mailbox, uint8_t* inData)
{
  memcpy(mailbox->data, inData, 8);
}

void can_mailbox_get_data(can_mailbox* mailbox, uint8_t* target)
{
  memcpy(target, mailbox->data, 8);
}

void can_mailbox_set_flag(can_mailbox* mailbox)
{
  mailbox->newMsg = 1;
}

uint8_t can_mailbox_get_flag(can_mailbox* mailbox)
{
  return (mailbox->newMsg);
}

void can_mailbox_clear_flag(can_mailbox* mailbox)
{
  (mailbox->newMsg) = 0;
}

void can_mailbox_set_id(can_mailbox* mailbox, uint32_t id)
{
  (mailbox->msgId) = id;
}

uint32_t can_mailbox_get_id(can_mailbox* mailbox)
{
  return (mailbox->msgId);
}


void can_driver_send_msg(uint8_t* data, 
                        uint32_t stdId,
                        int dlc,
                        uint8_t hwMailbox)
{
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox[3];
  
  txHeader.DLC = dlc;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = stdId;
  txHeader.TransmitGlobalTime = DISABLE;

  //TXID DEBUG
  // char* debug[64];
  // sprintf(debug, "tx ID: %X\n\r", stdId);
  // uart_send_string(debug);
  // for(int i = 0; i < 8; i++)
  // {
  //   sprintf(debug, "tx send data %i: %X\n\r",i, data[i]);
  //   uart_send_string(debug);
  // }

  if(HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox[hwMailbox]) != HAL_OK)
  {
    uart_send_string("CAN TX not successful");
  }

#if GLOBAL_DEBUG
  // char* debugbuffer[128];
  // sprintf(debugbuffer, "DLC: %u\n\r TXData: %s\n\r",txHeader.DLC, data);
  // for(int i = 0; i < txHeader.DLC; i++)
  // {
  //   sprintf(debugbuffer, "TXdata %i: %X\n\r", i, data[i]);
  //   uart_send_string(debugbuffer);
  // }

#endif
}


void can_cmd_handle_regVal(uint32_t id, uint8_t* inData)
{
  uint8_t accId = (id >> CAN_ACC_CMD_OFFSET) & 3;
  uint8_t regId = inData[0];
  uint16_t regVal;
  memcpy(&regVal, &inData[1], 2);

  char* debug[64];
  sprintf(debug, "Regval: %i\n\r", regVal);
  uart_send_string(debug);  
}


void can_cmd_handle_regReq(uint32_t id, uint8_t* inData)
{
  uint8_t accSelect = (uint8_t)(id >> CAN_ACC_CMD_OFFSET) & 1; //Relevant if more accs are ever implemented
  uint8_t regSelect = inData[0];
  uint16_t regVal = accl_interface_read_register(regSelect);
  
  //TODO: imeplement logic to check if the requested register
  //is one of the x/y/z registers, as well as embedding the
  //correct accelerometer ID into the message
  accSelect = 0;
  uint32_t outId = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_REG_RX;
  uint8_t outData[8];
  outData[0] = regSelect;
  memcpy(&outData[1], &regVal, 2);

  can_interface_queue_tx(ACC_REG_RX, outData, outId);
}


void can_cmd_handle_motorSp(uint32_t id, uint8_t* inData)
{
  uint8_t motorSelect = (uint8_t)(id >> CAN_MOTOR_CMD_OFFSET) & 1;
  if(inData[0] == 'e')
  {
    int32_t setpoint;
    memcpy(&setpoint, &inData[1], 4);
    motor_interface_set_setpoint(motorSelect, setpoint);
    int32_t resolution = motor_interface_get_resolution(motorSelect);
    float setpointRads = (float)(setpoint/resolution);
    controller_interface_set_setpoint(motorSelect, setpointRads);

  }
  else if(inData[0] == 'r')
  {
    float setpoint;
    memcpy(&setpoint, &inData[1], 4);
    controller_interface_set_setpoint(motorSelect, setpoint);
  }

}



void can_driver_cmd_rxTEMPLATE(uint32_t id, uint8_t* inData)
{
  //Template for rx handlers
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == num_types)
  {

  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED COMMAND FOR THIS HANDLER: RXn\n\r");
#endif
  }  
}


void can_driver_cmd_rx0(uint32_t id, uint8_t* inData)
{
  //Incoming accelerometer x data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == num_types)
  {

  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED COMMAND FOR THIS HANDLER: RXn\n\r");
#endif
  }    
  
}
void can_driver_cmd_rx1(uint32_t id, uint8_t* inData)
{
  //Incoming accelerometer y data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == num_types)
  {

  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED COMMAND FOR THIS HANDLER: RXn\n\r");
#endif
  }    
}
void can_driver_cmd_rx2(uint32_t id, uint8_t* inData)
{
  //Incoming accelerometer z data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == num_types)
  {

  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED COMMAND FOR THIS HANDLER: RXn\n\r");
#endif
  }    
}
void can_driver_cmd_rx3(uint32_t id, uint8_t* inData)
{
  //Incoming IMU register data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == ACC_REG_RX)
  {
    can_cmd_handle_regVal(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX3\n\r");
#endif
  }    
}
void can_driver_cmd_rx4(uint32_t id, uint8_t* inData)
{
  //Incoming IMU register readout request
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == ACC_REG_REQ)
  {
    can_cmd_handle_regReq(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX4\n\r");
#endif
  }    
}
void can_driver_cmd_rx5(uint32_t id, uint8_t* inData)
{
  //Incoming motor position setpoint
  //Arg 1 is clicks vs rads
  uint8_t cmd = (uint8_t)(id & 0x1F);

  // char* debug[64];
  // for(int i = 0; i < 8; i++)
  // {
  //   sprintf(debug, "rx handler data %i: %X\n\r",i, inData[i]);
  //   uart_send_string(debug);
  // }

  if(cmd == MOTOR_POS_SP)
  {
    can_cmd_handle_motorSp(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX5\n\r");
#endif
  }
}


void can_driver_cmd_rx6(uint32_t id, uint8_t* inData)
{
  //Incoming motor voltage setpoint
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == MOTOR_VLT_SP)
  {

  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX6\n\r");
#endif
  }
}

void can_driver_cmd_rx7(uint32_t id, uint8_t* inData)
{
  //Incoming motor position request
  //Arg 1 is clicks vs rads
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == MOTOR_POS_REQ)
  {

  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX7\n\r");
#endif
  }  
}

void can_driver_cmd_rx8(uint32_t id, uint8_t* inData)
{
  //Incoming motor position data
  //Arg 1 is clicks vs rads
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == num_types)
  {

  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED COMMAND FOR THIS HANDLER: RXn\n\r");
#endif
  }    
}

void can_driver_queue_tx(can_mailbox* mailbox, uint8_t* outData, uint32_t id)
{
  can_mailbox_set_data(mailbox, outData);
  can_mailbox_set_id(mailbox, id);
  can_mailbox_set_flag(mailbox);

}

// void can_driver_rx_accelerometer_cmd(uint8_t* data)
// {
//   uint8_t accelerometerId = (data[1] & 0xF0) >> 4;
//   uint8_t command = data[1] & 0x0F;
//   int32_t argument;
  
//   if(command == 1) //Sender requests 2-byte register value
//   {

//     uint16_t registerData = accl_interface_read_register(data[2]);
//     uint8_t txBuffer[4] = { 'A',
//                             2,
//                             (uint8_t)(registerData >> 8), 
//                             (uint8_t)(registerData & 0x00FF)};    
    
    
//     memcpy(accTxMailbox.data, txBuffer, 4);
//     accTxMailbox.newMsg = 1;
//     //can_interface_send_msg(txBuffer);
//     // int16_t test = (int16_t)((accTxMailbox.txData[2]<<8)|accTxMailbox.txData[3]);
//     // char* debug[64];
//     // sprintf(debug, "Decoded tx: %i\n\r", test);
//     // uart_send_string(debug);
//   }
//   if(command == 2) //Sender sends register value
//   {
//     //mailbox->rxData = &data;
//     if(accRxMailbox.newMsg == 0)
//     {
//       memcpy(accRxMailbox.data, data, 8);
//       accRxMailbox.newMsg = 1;
//     }
//     // int16_t test = (int16_t)((mailbox->rxData[2]<<8)|(mailbox->rxData[3]));
//     // char* debug[64];
//     // sprintf(debug, "Decoded rx: %i\n\r", test);
//     // uart_send_string(debug);

// #if GLOBAL_DEBUG
//     //char* debug[64];
//     // sprintf(debug, "Data: %i\n\r", test);
//     // uart_send_string(debug);

// #endif
//   }
//   //CMD3 not relevant for receive

// }


void can_rxfifo0_IRQHandler()
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  HAL_StatusTypeDef retVal;
  retVal = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData);

  //RX DEBUG
  char* debug[64];
  sprintf(debug, "rx ID: %X\n\r", rxHeader.StdId);
  uart_send_string(debug);
  for(int i = 0; i < 8; i++)
  {
    sprintf(debug, "Rx data %i: %X\n\r",i, rxData[i]);
    uart_send_string(debug);
  }



  if(retVal != HAL_OK)
  {
    uart_send_string("HAL_CAN_GetRxMessage ERROR");
  }
  else
  {
    uint8_t messageType = (rxHeader.StdId) & 0x1F;
    if(can_mailbox_get_flag(&rxMailboxes[messageType]) == 0)
    {
      can_mailbox_set_data(&rxMailboxes[messageType], rxData);
      can_mailbox_set_flag(&rxMailboxes[messageType]);
      can_mailbox_set_id(&rxMailboxes[messageType], rxHeader.StdId);
    }
  }

}

//Declared in stm32fxx_hal_can.h
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  can_rxfifo0_IRQHandler();

}