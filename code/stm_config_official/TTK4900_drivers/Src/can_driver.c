#include "can_driver.h"
#if GLOBAL_DEBUG
  #include "uart_driver.h"
#endif



const can_message_type numCanTypes = num_types;
#define NUM_CAN_TYPES num_types //Compiler gets angry if numCanTypes is used directly

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
  can_driver_cmd_rx9,
  can_driver_cmd_rxA,
  can_driver_cmd_rxB,
  can_driver_cmd_rxC,
  can_driver_cmd_rxD,
  can_driver_cmd_rxE,
};

static can_mailbox rxMailboxes[NUM_CAN_TYPES]; 
static can_mailbox txMailboxes[NUM_CAN_TYPES];

void can_interface_queue_tx(uint8_t mailbox, uint8_t* outData, uint32_t id)
{
  can_driver_queue_tx(&txMailboxes[mailbox], outData, id);
}


void can_interface_send_msg(uint8_t* data, uint32_t id, int dlc, uint8_t hwMailbox)
{
  can_driver_send_msg(data, id, dlc, hwMailbox);
}

void can_rx_executive()
{
  for(int i = 0; i < numCanTypes; i++)
  {
    if(can_mailbox_get_flag(&rxMailboxes[i]))
    {      
      uint8_t data[8];
      can_mailbox_get_data(&rxMailboxes[i], data);

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

  if(HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox[hwMailbox]) != HAL_OK)
  {

#if GLOBAL_DEBUG //&& (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("CAN TX not successful\n\r");
    char* debug[64];
    sprintf(debug, "ID: %X\n\r", stdId);
    uart_send_string(debug);
#endif    
  }
}

void can_cmd_handle_yAcc(uint32_t id, uint8_t* inData)
{

}

void can_cmd_handle_regVal(uint32_t id, uint8_t* inData)
{
  uint8_t accId = (id >> CAN_ACC_CMD_OFFSET) & 3;
  uint8_t regId = inData[0];
  uint16_t regVal;
  memcpy(&regVal, &inData[1], 2);

#if GLOBAL_DEBUG && (HW_INTERFACE == UART_INTERFACE)
  char* debug[64];
  sprintf(debug, "Regval: %i\n\r", regVal);
  uart_send_string(debug);
#endif  
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
  if(0 /*regSelect == 0x2A*/)
  {
    uint32_t outId = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_Y_TX;
    uint8_t outData[8];
    memcpy(&outData, &regVal, 2);
    can_interface_queue_tx(ACC_REG_RX, outData, outId);
  }
  else
  {
    uint32_t outId = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_REG_RX;
    uint8_t outData[8];
    outData[0] = regSelect;
    memcpy(&outData[1], &regVal, 2);

    can_interface_queue_tx(ACC_REG_RX, outData, outId);    
  }
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

void can_cmd_handle_axisReq(uint32_t id, uint8_t* inData)
{
  uint8_t accSelect = (uint8_t)(id >> CAN_ACC_CMD_OFFSET) & 1; //Relevant if more accs are ever implemented
  can_message_type cmd = (can_message_type)(id & 0x1F);
  
  //TODO: imeplement logic to embed the
  //correct accelerometer ID into the message
  if(cmd == ACC_X_REQ)
  {
    int16_t acceleration = accl_interface_get_x_acc(); //Hardcoded for shoulder
    int16_t rotation = accl_interface_get_x_rot();
    uint32_t outId = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_X_TX;
    uint8_t outData[8];
    memcpy(&outData, &acceleration, 2);
    memcpy(&outData[2], &rotation, 2);
    can_interface_queue_tx(ACC_Y_TX, outData, outId);
  }
  else if(cmd == ACC_Y_REQ)
  {
    int16_t acceleration = accl_interface_get_y_acc(); //Hardcoded for shoulder
    int16_t rotation = accl_interface_get_y_rot();
    uint32_t outId = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_Y_TX;
    uint8_t outData[8];
    memcpy(&outData, &acceleration, 2);
    memcpy(&outData[2], &rotation, 2);
    can_interface_queue_tx(ACC_Y_TX, outData, outId);
  }
  else if(cmd == ACC_Z_REQ)
  {
    int16_t acceleration = accl_interface_get_z_acc(); //Hardcoded for shoulder
    int16_t rotation = accl_interface_get_z_rot();
    uint32_t outId = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_Z_TX;
    uint8_t outData[8];
    memcpy(&outData, &acceleration, 2);
    memcpy(&outData[2], &rotation, 2);
    can_interface_queue_tx(ACC_Z_TX, outData, outId);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("ERROR AXIS REQUEST CMD\n\r");
#endif
  }
}

void can_cmd_handle_axisData(uint32_t id, uint8_t* inData)
{
  uint8_t accSelect = (uint8_t)(id >> CAN_ACC_CMD_OFFSET) & 1; //Relevant if more accs are ever implemented
  can_message_type cmd = (can_message_type)(id & 0x1F);

  if(cmd == ACC_X_TX)
  {
    int16_t acceleration;
    int16_t rotation;
    memcpy(&acceleration, inData, 2);
    memcpy(&rotation, inData[2], 2);

    controller_interface_acc_setX(accSelect, acceleration);
    controller_interface_rot_setX(accSelect, rotation);
    controller_interface_acc_set_newX(accSelect);
    controller_interface_rot_set_newX(accSelect);
  }
  else if(cmd == ACC_Y_TX)
  {
    
    int16_t acceleration;
    int16_t rotation;
    memcpy(&acceleration, inData, 2);
    memcpy(&rotation, inData[2], 2);

    controller_interface_acc_setY(accSelect, acceleration);
    controller_interface_rot_setY(accSelect, rotation);
    controller_interface_acc_set_newY(accSelect);
    controller_interface_rot_set_newY(accSelect);

  }
  else if(cmd == ACC_Z_TX)
  {
    int16_t acceleration;
    int16_t rotation;
    memcpy(&acceleration, inData, 2);
    memcpy(&rotation, inData[2], 2);
    controller_interface_acc_setZ(accSelect, acceleration);
    controller_interface_rot_setZ(accSelect, rotation);
    controller_interface_acc_set_newZ(accSelect);
    controller_interface_rot_set_newZ(accSelect);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("ERROR INCOMING AXIS CMD\n\r");
#endif
  }

}


void can_cmd_handle_inState(uint32_t id, uint8_t* inData)
{
  uint8_t globalState = inData[0];
  uint8_t calibrationState = inData[1];
  state_interface_set_global_state(globalState);
  state_interface_set_calibration_state(calibrationState);
}

void can_cmd_handle_dualJointSp(uint32_t id, uint8_t* inData)
{
  float setpoint_joint0;
  float setpoint_joint1;

  memcpy(&setpoint_joint0, &inData[0], 4);
  memcpy(&setpoint_joint1, &inData[4], 4);

  controller_interface_set_setpoint(0, setpoint_joint0);
  controller_interface_set_setpoint(1, setpoint_joint1);
}


void can_cmd_handle_wristElbowSp(uint32_t id, uint8_t* inData)
{
  float setpoint_wrist;
  float setpoint_elbow;

  memcpy(&setpoint_wrist, &inData[0], 4);
  memcpy(&setpoint_elbow, &inData[4], 4);

  controller_interface_set_setpoint(0, setpoint_wrist);
  controller_interface_set_setpoint(1, setpoint_elbow);

}

void can_cmd_handle_pinchTwistSp(uint32_t id, uint8_t* inData)
{
  float setpoint_pinch;
  float setpoint_twist;

  memcpy(&setpoint_pinch, &inData[0], 4);
  memcpy(&setpoint_twist, &inData[4], 4);

  controller_interface_set_setpoint(0, setpoint_pinch);
  controller_interface_set_setpoint(1, setpoint_twist);

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
  if(cmd == ACC_X_TX)
  {
    can_cmd_handle_axisData(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX0");
#endif
  }    
  
}
void can_driver_cmd_rx1(uint32_t id, uint8_t* inData)
{
  //Incoming accelerometer y data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == ACC_Y_TX)
  {
    can_cmd_handle_axisData(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX1\n\r");
#endif
  }    
}
void can_driver_cmd_rx2(uint32_t id, uint8_t* inData)
{
  //Incoming accelerometer z data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == ACC_Z_TX)
  {
    can_cmd_handle_axisData(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX2\n\r");
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

  if(cmd == JOINT_POS_SP)
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
  //Arg 0 is clicks vs rads
  //TODO: Move to dedicated handler functions
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == JOINT_POS_REQ)
  {
    uint8_t motorNum = (uint8_t)(id >> CAN_MOTOR_CMD_OFFSET) & 3;
    uint8_t motorSelect = (uint8_t)(id >> CAN_MOTOR_CMD_OFFSET) & 1;
        
    uint8_t outData[8];
    memcpy(&outData[0], &inData[0], 1);

    uint8_t outId = (motorNum << CAN_MOTOR_CMD_OFFSET) | JOINT_POS_TX;
    
    if(inData[0] == 'e')
    {
      int32_t clicks = motor_interface_get_total_count(motorSelect);
      memcpy(&outData[1], &clicks, 4);
    }
    if(inData[0] == 'r')
    {
      float rads = controller_interface_get_position(motorSelect);
      memcpy(&outData[1], &rads, 4);
    }

    can_interface_queue_tx(JOINT_POS_TX, outData, outId);
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
  //Arg 0 is clicks vs rads
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == JOINT_POS_TX)
  {
    //This may be wrong
    uart_send_string(inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX8\n\r");
#endif
  }    
}


void can_driver_cmd_rx9(uint32_t id, uint8_t* inData)
{
  //Incoming request for accelerometer X axis data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == ACC_X_REQ)
  {
    void can_cmd_handle_axisReq(uint32_t id, uint8_t* inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RX9\n\r");
#endif
  }    
}

void can_driver_cmd_rxA(uint32_t id, uint8_t* inData)
{
  //Incoming request for accelerometer Y axis data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == ACC_Y_REQ)
  {
    can_cmd_handle_axisReq(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RXA\n\r");
#endif
  }    
}

void can_driver_cmd_rxB(uint32_t id, uint8_t* inData)
{
  //Incoming request for accelerometer Z axis data
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == ACC_Z_REQ)
  {
    can_cmd_handle_axisReq(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RXB\n\r");
#endif
  }    
}

void can_driver_cmd_rxC(uint32_t id, uint8_t* inData)
{
  //Incoming order to set the global state
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == GBL_ST_SET)
  {
    can_cmd_handle_inState(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RXC\n\r");
#endif
  }  
}

void can_driver_cmd_rxD(uint32_t id, uint8_t* inData)
{
  //Incoming setpoints for elbow and wrist joints
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == WRIST_ELBOW_SP)
  {
    can_cmd_handle_wristElbowSp(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RXD\n\r");
#endif
  }  
}


void can_driver_cmd_rxE(uint32_t id, uint8_t* inData)
{
  //Incoming setpoints for twist and pinch joints
  uint8_t cmd = (uint8_t)(id & 0x1F);
  if(cmd == PINCH_TWIST_SP)
  {
    can_cmd_handle_pinchTwistSp(id, inData);
  }
  else
  {
#if GLOBAL_DEBUG
    uart_send_string("RECEIVED WRONG COMMAND FOR THIS HANDLER: RXE\n\r");
#endif
  }  
}


void can_driver_queue_tx(can_mailbox* mailbox, uint8_t* outData, uint32_t id)
{
  can_mailbox_set_data(mailbox, outData);
  can_mailbox_set_id(mailbox, id);
  can_mailbox_set_flag(mailbox);

}

void can_rxfifo0_IRQHandler()
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  HAL_StatusTypeDef retVal;
  retVal = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData);
  
  if(retVal != HAL_OK)
  {

#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("HAL_CAN_GetRxMessage ERROR");
#endif    
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