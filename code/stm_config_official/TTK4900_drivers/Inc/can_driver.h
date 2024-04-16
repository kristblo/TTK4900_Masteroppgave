#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H
/**
  ******************************************************************************
  * @file    can_driver.h
  * @brief   This file contains all the function prototypes and struct
  *           definitions for the can_driver.c file
  *        
  ******************************************************************************
  * @attention
  *
  * CAN driver for the TTK4900 Master project of Kristian Blom, spring
  * semester of 2024. The driver specifies CAN message types relevant to
  * the project, as well as relevant functions for the handling of transmission
  * and reception of messages.
  *
  ******************************************************************************
  */


//External library includes
#include "stdint.h"
//CubeMX generated includes
#include "can.h"

//TTK4900 library includes
#include "unit_config.h"
#include "motor_driver.h"
#include "accelerometer_driver.h"
#include "joint_controller.h"
#include "state_machine.h"

//------FILE BEGIN------
#define CAN_MOTOR_CMD_OFFSET 5
#define CAN_ACC_CMD_OFFSET 8
#define CAN_GBL_CMD_OFFSET 10

/// @brief A virtual CAN mailbox for outgoing and incoming messages
typedef struct
{
  /// @brief Flag signifying that the mailbox contains an unhandled message
  uint8_t newMsg;
  
  /// @brief CAN message ID, 11 bits (standard ID)
  uint32_t msgId;
  
  /// @brief CAN message data
  uint8_t data[8];
} can_mailbox;

/// @brief CAN message types
typedef enum 
{
  /// @brief A message containting acc/rot X axis
  ACC_X_TX,
  
  /// @brief A message containting acc/rot Y axis
  ACC_Y_TX,
  
  /// @brief A message containting acc/rot Z axis
  ACC_Z_TX,
  
  /// @brief A message containting an arbitrary accelerometer register value
  ACC_REG_RX,
  
  /// @brief A message requesting an arbitrary accelerometer register value
  ACC_REG_REQ,

  /// @brief A message containing a joint position setpoint
  JOINT_POS_SP,
  
  /// @brief A message containting a motor voltage percent setpoint
  MOTOR_VLT_SP,

  /// @brief A message requesting the position of a joint
  JOINT_POS_REQ,
  
  /// @brief A message containing the position of a joint
  JOINT_POS_TX,

  /// @brief A message requesting acc/rot x axis
  ACC_X_REQ,

  /// @brief A message requesting acc/rot y axis
  ACC_Y_REQ,

  /// @brief A message requesting acc/rot z axis
  ACC_Z_REQ,

  /// @brief Message contains a global state to set
  GBL_ST_SET,

  /// @brief Message contains setpoints for the elbow and wrist joint controllers
  WRIST_ELBOW_SP,

  /// @brief Message contains setpoints for the twist and pinch joints controllers
  PINCH_TWIST_SP,

  /// @brief Dummy type for counting the number of message types, must always be last
  num_types,
} can_message_type;



//////////////////
//Public functions
//////////////////


/// @brief Module external function for queueing a CAN message for transmit
/// @param mailbox Mailbox number, must correspond to the correct message type
/// @param outData Data to send
/// @param id CAN transmit ID
void can_interface_queue_tx(uint8_t mailbox, uint8_t* outData, uint32_t id);


/// @brief Public function to send CAN message immediately
/// @param data Data to send, max 8 byte
/// @param id CAN message ID
/// @param dlc CAN message data length
/// @param hwMailbox Hardware transmit mailbox to send from
void can_interface_send_msg(uint8_t* data, uint32_t id, int dlc, uint8_t hwMailbox);


///////////////////
//Private functions
///////////////////


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

/// @brief Sends a CAN message
/// @param data Data to send, max 8 bytes
/// @param stdId Message ID required for rx handling
/// @param dlc Number of bytes to send
/// @param hwMailbox Hardware(!) mailbox to queue to
void can_driver_send_msg(uint8_t* data, uint32_t stdId, int dlc, uint8_t hwMailbox);



//The following functions handle reception of certain CAN message types,
//and are wrapped by the cmd_rxn functions

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


/// @brief Handles an incoming request for accelerometer axis data
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_axisReq(uint32_t id, uint8_t* inData);

/// @brief Handles incoming accelerometer axis data
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_axisData(uint32_t id, uint8_t* inData);


/// @brief Handles incoming state information
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_inState(uint32_t id, uint8_t* inData);


/// @brief Handles incoming setpoints for both joints
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_dualJointSp(uint32_t id, uint8_t* inData);


/// @brief Handles incoming setpoints for elbow and wrist joints
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_wristElbowSp(uint32_t id, uint8_t* inData);


/// @brief Handles incoming setpoints for twist and pinch joints
/// @param id Incoming CAN ID
/// @param inData Incoming CAN data
void can_cmd_handle_pinchTwistSp(uint32_t id, uint8_t* inData);

//The following cmd_rxn functions MUST match with the number of
//available can_message_type, and MUST be added to the
//canRxFunctions list in the can_driver.c file

/// @brief "Generic" function to handle CAN message type ACC_X_TX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx0(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_Y_TX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx1(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_Z_TX
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

/// @brief "Generic" function to handle CAN message type JOINT_POS_SP
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx5(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type MOTOR_VLT_SP
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx6(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_X_TX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx7(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type JOINT_POS_TX
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx8(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_X_REQ
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rx9(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_Y_REQ
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rxA(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type ACC_Z_REQ
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rxB(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type GBL_ST_SET
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rxC(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type WRIST_ELBOW_SP
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rxD(uint32_t id, uint8_t* inData);

/// @brief "Generic" function to handle CAN message type PINCH_TWIST_SP
/// @param id CAN message ID
/// @param inData CAN message data field
void can_driver_cmd_rxE(uint32_t id, uint8_t* inData);


#endif //CAN_DRIVER_H