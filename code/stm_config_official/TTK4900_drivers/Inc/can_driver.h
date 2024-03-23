#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H
//External library includes
#include "stdint.h"
//CubeMX generated includes
#include "can.h"

//TTK4900 library includes
#include "unit_config.h"
#include "motor_driver.h"
#include "accelerometer_driver.h"
#include "joint_controller.h"

//------FILE BEGIN------
#define CAN_MOTOR_CMD_OFFSET 5
#define CAN_ACC_CMD_OFFSET 8


typedef struct
{
  uint8_t newMsg;
  uint32_t msgId;
  uint8_t data[8];
} can_mailbox;

/// @brief CAN message types, also dictate CAN priority
typedef enum 
{
  ACC_X_RX,
  ACC_Y_RX,
  ACC_Z_RX,
  ACC_REG_RX,
  ACC_REG_REQ,
  MOTOR_POS_SP,
  MOTOR_VLT_SP,
  MOTOR_POS_REQ,
  MOTOR_POS_RX,
  num_types, //This must always be last
} can_message_types;


/// @brief Queues a CAN message for transmit, for external use
/// @param mailbox Mailbox number, must correspond to the correct message type
/// @param outData Data to send
/// @param id CAN transmit ID
void can_interface_queue_tx(uint8_t mailbox, uint8_t* outData, uint32_t id);


/// @brief Looks for new incoming CAN messages and handles them. MAIN LOOP ONLY
void can_rx_executive();


/// @brief Looks for new CAN messages to send, sends. MAIN LOOP ONLY
void can_tx_executive();


/// @brief Sets the data field of the given mailbox
/// @param mailbox Mailbox to insert data into
/// @param inData Data to insert
void can_mailbox_set_data(can_mailbox* mailbox, uint8_t* inData);


/// @brief Inserts the data field of the given mailbox to target. CAUTION: memcpy 8 bytes
/// @param mailbox Mailbox to retrieve data from
/// @param target Pointer to data target
void can_mailbox_get_data(can_mailbox* mailbox, uint8_t* target);


/// @brief Sets the newmsg flag of the given mailbox
/// @param mailbox Mailbox to set flag
void can_mailbox_set_flag(can_mailbox* mailbox);


/// @brief Gets the newmsg flag of the given mailbox
/// @param mailbox Mailbox to get
/// @return Mailbox' newmsg flag
uint8_t can_mailbox_get_flag(can_mailbox* mailbox);


/// @brief Clears the newmsg flag of the given mailbox
/// @param mailbox Mailbox to clear flag from
void can_mailbox_clear_flag(can_mailbox* mailbox);


/// @brief Sets the CAN message ID field of the given mailbox
/// @param mailbox Mailbox to insert ID
/// @param id CAN message ID to insert
void can_mailbox_set_id(can_mailbox* mailbox, uint32_t id);


/// @brief Gets the CAN message ID of the given mailbox
/// @param mailbox Mailbox to get ID from
/// @return CAN message ID
uint32_t can_mailbox_get_id(can_mailbox* mailbox);


/// @brief Queues a can message for transmit, driver internal
/// @param mailbox Mailbox struct in which data will be placed
/// @param outData Data to send (8 bytes)
/// @param id CAN transmit ID
void can_driver_queue_tx(can_mailbox* mailbox, uint8_t* outData, uint32_t id);

/// @brief Sends a CAN message, is called by can_interface_send_msg
/// @param data Data to send, max 8 bytes
/// @param stdId Message ID required for rx handling
/// @param dlc Number of bytes to send
/// @param hwMailbox Hardware(!) mailbox to queue to
void can_driver_send_msg(uint8_t* data, uint32_t stdId, int dlc, uint8_t hwMailbox);


/// @brief CAN accelerometer message handler. DEPRECATED
/// @param data CAN message data
void can_driver_rx_accelerometer_cmd(uint8_t* data);

void can_cmd_handle_yAcc(uint32_t id, uint8_t* inData);

/// @brief Handles an incoming accelerometer register message
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_regVal(uint32_t id, uint8_t* inData);


/// @brief Handles an incoming accelerometer read request
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_regReq(uint32_t id, uint8_t* inData);


/// @brief Handles an incoming motor setpoint
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_motorSp(uint32_t id, uint8_t* inData);



//The following rxn functions MUST match with the number of
//available can_message_types, and MUST be added to the
//canRxFunctions list in the .c file

/// @brief "Generic" function to handle CAN message type ACC_X_RX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx0(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_Y_RX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx1(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_Z_RX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx2(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_REG_RX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx3(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_REG_REQ
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx4(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type MOTOR_POS_SP
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx5(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type MOTOR_VLT_SP
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx6(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_X_RX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx7(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type MOTOR_POS_RX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx8(uint32_t id, uint8_t* inData);



#endif //CAN_DRIVER_H