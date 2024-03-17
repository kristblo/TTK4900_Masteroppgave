#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H
//External library includes
#include "stdint.h"
//CubeMX generated includes
#include "can.h"

//TTK4900 library includes
#include "unit_config.h"
#include "motor_driver.h"

//------FILE BEGIN------

//Generate default values for CAN messages
typedef struct 
{
  uint8_t* data[8]; //Max data size for CAN message is 8
  int dlc;  //Bytes of data to transfer
  int stdId; //Transmission ID
  uint8_t mailbox; //Between 0 and 2

} can_send_msg_args;

//User level function, allows variable arguments 
#define can_interface_send_msg(...) can_driver_send_msg_wrp((can_send_msg_args*){__VA_ARGS__});


/// @brief Sends a CAN message, is called by can_interface_send_msg
/// @param data Data to send, max 8 bytes
/// @param stdId Message ID, defaults to CAN_TXID
/// @param dlc Number of bytes to send, defaults to 8
/// @param mailbox Transmit mailbox to send from, defaults to 0
void can_driver_send_msg_base(uint8_t* data, int stdId, int dlc, uint8_t mailbox);


/// @brief Wrapper function to allow default args
/// @param input CAN argument struct, essentially the message header
void can_driver_send_msg_wrp(can_send_msg_args* input);


/// @brief CAN receive interrupt handler
/// @param data CAN message data
void can_driver_rx_handler(uint8_t* data);


/// @brief CAN motor control message handler
/// @param data CAN message data
void can_driver_rx_motor_cmd(uint8_t* data);


/// @brief CAN accelerometer message handler
/// @param data CAN message data
void can_driver_rx_accelerometer_cmd(uint8_t* data);

#endif //CAN_DRIVER_H