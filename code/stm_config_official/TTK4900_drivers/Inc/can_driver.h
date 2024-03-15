#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H
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
  int dlc;  //Bytes of data to transfer
  int stdId; //Transmission ID
  uint8_t mailbox; //Between 0 and 2


} can_send_msg_args;
void can_send_msg_base(uint8_t* data, int stdId, int dlc, uint8_t mailbox);
void can_send_msg_wrp(can_send_msg_args* input);
#define can_send_msg(...) can_send_msg_wrp((can_send_msg_args*){__VA_ARGS__});

void can_rx_handler(uint8_t* data);

#endif //CAN_DRIVER_H