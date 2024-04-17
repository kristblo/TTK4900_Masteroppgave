#include "ros_uart_parser.h"

static uint8_t uartRosRxBuffer[32];
static uint8_t newRosMsgFlag = 0;
float pos_pinch = 0; //TODO: Fix MoveIt so that it can send pinch references!

void ros_interface_set_rxBuffer(uint8_t* inData)
{
  memcpy(uartRosRxBuffer, inData, 32);
}

void ros_interface_get_rxBuffer(uint8_t* target)
{
  memcpy(target, &uartRosRxBuffer, 32);
}

void ros_interface_clear_rxBuffer()
{
  //TODO:Implement
}

void ros_interface_set_newMsgFlag()
{
  newRosMsgFlag = 1;
}

uint8_t ros_interface_get_newMsgFlag()
{
  return newRosMsgFlag;
}

void ros_interface_clear_newMsgFlag()
{
  newRosMsgFlag = 0;
}

void ros_interface_parse_input()
{
  
  char* debug[64];

  const int HEADEROFFSET = 3;
  char* header[HEADEROFFSET+1];//strcmp requires string+eof
  memcpy(header, uartRosRxBuffer, 3);
  

  if((int)strcmp(header, "num") == 0)
  {
    float pos_rail    ;// = &uartRosRxBuffer[HEADEROFFSET + 0]; //TODO: find out why this don't work
    float pos_shoulder;// = &uartRosRxBuffer[HEADEROFFSET + 4];
    float pos_elbow   ;// = &uartRosRxBuffer[HEADEROFFSET + 8];
    float pos_wrist   ;// = &uartRosRxBuffer[HEADEROFFSET + 12];
    float pos_twist   ;// = &uartRosRxBuffer[HEADEROFFSET + 16];

    memcpy(&pos_rail, &uartRosRxBuffer[HEADEROFFSET], 4);
    controller_interface_set_setpoint(0, (pos_rail)*1000); //ROS outputs mm

    memcpy(&pos_shoulder, &uartRosRxBuffer[HEADEROFFSET + 4], 4);
    controller_interface_set_setpoint(1, pos_shoulder);

    memcpy(&pos_elbow, &uartRosRxBuffer[HEADEROFFSET + 8], 4);
    memcpy(&pos_wrist, &uartRosRxBuffer[HEADEROFFSET + 12], 4);
    memcpy(&pos_twist, &uartRosRxBuffer[HEADEROFFSET + 16 ], 4);
    
    uint8_t shoulderData[8];
    memcpy(&shoulderData[0], &pos_wrist, 4);
    memcpy(&shoulderData[4], &pos_elbow, 4);
    uint32_t id = (2 << CAN_MOTOR_CMD_OFFSET) | WRIST_ELBOW_SP;
    can_interface_queue_tx(WRIST_ELBOW_SP, shoulderData, id);

    uint8_t handData[8];
    memcpy(&handData[0], &pos_pinch, 4);
    memcpy(&handData[4], &pos_twist, 4);

    // char* debug[64];
    // sprintf(debug, "Reading pinchpos: %i\n\r", (int)(pos_pinch));
    // uart_send_string(debug);

    id = (4 << CAN_MOTOR_CMD_OFFSET) | PINCH_TWIST_SP;
    can_interface_queue_tx(PINCH_TWIST_SP, handData, id);

  }
  else if((int)strcmp(header, "str") == 0)
  {
    uart_send_string("entered str\n\r");
    // state_interface_set_global_state(GS_CALIBRATING);
    // state_interface_set_calibration_state(CS_RAIL);
    // state_interface_set_global_state(GS_OPERATING);
    // state_interface_broadcast_global_state();
    char* headerLessMsg[64]; //must be 64 byte
    memcpy(headerLessMsg, &uartRosRxBuffer[3], 29);
    uart_send_string(headerLessMsg);
    string_cmd_processor(headerLessMsg);
  }
}

void ros_interface_set_pinchPos(float pos)
{
  char* debug[64];
  sprintf(debug, "Actually setting pinchpos: %i\n\r", (int)(pos));
  uart_send_string(debug);

  pos_pinch = pos;
}

void ros_interface_queue_setpoints();

void ros_interface_dequeue_setpoints();

void ros_interface_send_setpoints();
