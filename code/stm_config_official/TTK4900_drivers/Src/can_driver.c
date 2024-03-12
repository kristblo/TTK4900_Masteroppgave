#include "can_driver.h"
#if GLOBAL_DEBUG
  #include "uart_driver.h"
#endif

void can_send_msg_base(uint8_t* data, 
                        int dlc, 
                        int extId, 
                        int ide, 
                        int rtr,
                        int stdId, 
                        int transmitGlobalTime)
{
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;
  
  txHeader.DLC = dlc;
  txHeader.ExtId = extId;
  txHeader.IDE = ide;
  txHeader.RTR = rtr;
  txHeader.StdId = stdId;
  txHeader.TransmitGlobalTime = transmitGlobalTime;

#if GLOBAL_DEBUG
  char* debugbuffer[128];
  sprintf(debugbuffer, "DLC: %i\n\r ID: %i\n\r, Data: %s\n\r",
          txHeader.DLC, txHeader.StdId, data);
  uart_send_string(debugbuffer);
#endif

  HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox);
}

void can_send_msg_wrp(can_send_msg_args input)
{
  int dlc_out = input.dlc ? input.dlc : 8;
  int extId_out = input.extId ? input.extId : 0;
  int ide_out = input.ide ? input.ide : CAN_ID_STD;
  int rtr_out = input.rtr ? input.rtr : CAN_RTR_DATA;
  int stdId_out = input.stdId ? input.stdId : CAN_TXID;
  int transmitGlobalTime_out = input.transmitGlobalTime ? input.transmitGlobalTime : DISABLE;    

  can_send_msg_base(input.data,
                    dlc_out,
                    extId_out,
                    ide_out,
                    rtr_out,
                    stdId_out,
                    transmitGlobalTime_out);
}

void can_rx_handler(uint8_t* data)
{
  uart_send_string("Got a CAN message\n\r");
}

//Declared in stm32fxx_hal_can.h
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

#if GLOBAL_DEBUG
  char* debugbuffer[128];
  sprintf(debugbuffer, "DLC: %i\n\r ID: %i\n\r, Data: %s\n\r",
          rxHeader.DLC, rxHeader.StdId, rxData);
  uart_send_string(debugbuffer);
#endif

  can_rx_handler(rxData);
}