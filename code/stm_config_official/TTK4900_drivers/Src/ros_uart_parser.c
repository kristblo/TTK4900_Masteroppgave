#include "ros_uart_parser.h"

static uint8_t uartRosRxBuffer[32];
static uint8_t newRosMsgFlag = 0;

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
  
  //Breaks the strcmp if not present...
  // for(int i = 0; i < 32; i++)
  // {
  //   sprintf(debug, "buffer %i: %X\n\r", i, uartRosRxBuffer[i]);
  //   uart_send_string(debug);
  // }

  const int HEADEROFFSET = 3;
  char* header[HEADEROFFSET+1];//strcmp requires string+eof
  memcpy(header, uartRosRxBuffer, 3);

  //uart_send_string(header);
  

  if((int)strcmp(header, "num") == 0)
  {
    float pos_rail    ;// = &uartRosRxBuffer[HEADEROFFSET + 0];
    float pos_shoulder;// = &uartRosRxBuffer[HEADEROFFSET + 4];
    float pos_elbow   ;// = &uartRosRxBuffer[HEADEROFFSET + 8];
    float* pos_wrist   ;// = &uartRosRxBuffer[HEADEROFFSET + 12];
    float* pos_twist   ;// = &uartRosRxBuffer[HEADEROFFSET + 16];

    memcpy(&pos_rail, &uartRosRxBuffer[HEADEROFFSET], 4);
    controller_interface_set_setpoint(0, (pos_rail)*1000);

    memcpy(&pos_shoulder, &uartRosRxBuffer[HEADEROFFSET + 4], 4);
    controller_interface_set_setpoint(1, pos_shoulder);

    memcpy(&pos_elbow, &uartRosRxBuffer[HEADEROFFSET + 8], 4);
    
    uint8_t data[8];
    uint8_t mode = 'r';
    uint32_t id = (3 << CAN_MOTOR_CMD_OFFSET) | JOINT_POS_SP;
    memcpy(data, &mode, 1);
    memcpy(&data[1], &pos_elbow, 4);
    can_interface_queue_tx(JOINT_POS_SP, data, id);

  }
  else if((int)strcmp(header, "hom") == 0)
  {
    uart_send_string("home\n\r");
    // state_interface_set_global_state(GS_CALIBRATING);
    // state_interface_set_calibration_state(CS_RAIL);
    state_interface_set_global_state(GS_OPERATING);
    state_interface_broadcast_global_state();

  }
}


void ros_interface_queue_setpoints();

void ros_interface_dequeue_setpoints();

void ros_interface_send_setpoints();
