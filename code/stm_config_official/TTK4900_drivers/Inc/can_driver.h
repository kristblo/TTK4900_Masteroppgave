//External library includes
#include "stdint.h"
//CubeMX generated includes
#include "can.h"

//TTK4900 library includes
#include "unit_config.h"


//------FILE BEGIN------

//Generate default values for CAN messages
typedef struct 
{
  uint8_t* data[8]; //Max data size for CAN message is 8
  int dlc;
  int extId;
  int ide;
  int rtr;
  int stdId;
  int transmitGlobalTime;

} can_send_msg_args;
#define can_send_msg(...) can_send_msg_wrp((can_send_msg_args){__VA_ARGS__});
void can_send_msg_base(uint8_t* data, int dlc, int extId, int ide, int rtr, int stdId, int transmitGlobalTime);
void can_send_msg_wrp(can_send_msg_args input);

void can_rx_handler(uint8_t* data);
