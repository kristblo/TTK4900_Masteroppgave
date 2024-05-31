#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

/**
  ******************************************************************************
  * @file    joint_controller.h
  * @brief   This file contains all the function prototypes and struct
  *           definitions for the joint_controller.c file
  *        
  ******************************************************************************
  * @attention
  *
  * Joint controller for the TTK4900 Master project of Kristian Blom, spring
  * semester of 2024. The controller makes use of the motor driver
  * to implement PID positional control of the two joints for which
  * the relevant MCU is responsible. The accelerometer struct holds data
  * from the joint's accelerometer, where applicable, and is always received
  * via the CAN bus. This makes accelerometer data inherent to joint control,
  * not the accelerometer driver itself.
  * 
  * Private functions are not described individually, but correspond to their
  * public counterparts. They take a joint controller descriptor struct as 
  * argument, and are called by the public functions.
  *
  ******************************************************************************
  */


//External library includes
#include "stdint.h"
#include "math.h"

//CubeMX generated includes
#include "tim.h"

//TTK4900 library includes
#include "unit_config.h"
#include "uart_driver.h"
#include "motor_driver.h"
#include "can_driver.h"
#include "accelerometer_driver.h"

/// @brief Joint controller information database
typedef struct
{
  /// @brief Whether the joint has an accelerometer for position control
  uint8_t hasAccelerometer;

  /// @brief The joint's setpoint in radians/mm relative to its zero position
  float posSetpoint;
  
  /// @brief The joint's current position in radians/mm relative to its zero position
  float posCurrent;

  /// @brief The joint's position in the previous timestep
  float prevPos;
  
  /// @brief The joint's position error in radians/mm, relative to its setpoint
  float posError;

  /// @brief The joint's positional error in the previous timestep
  float prevError;

  /// @brief Current power setting of the joint
  float power;
  
  /// @brief The joint's power setting in the previous timestep
  float prevPower;
  
  /// @brief Whether the joint is in a "moving" state
  uint8_t isMoving;

  /// @brief Link to the corresponding motor_descriptor
  uint8_t motorNum;

  /// @brief Whether or not the joint should use sigmoid integral gain
  uint8_t sigmoidIntGain;

  /// @brief PID controller Kp
  float Kp;

  /// @brief PID controller Kp/Ti
  float KpTi;

  /// @brief PID controller Kd
  float Kd;

  /// @brief Positional integral error
  float intError;
  
  /// @brief The joint's human readable name. This field must be last
  uint8_t* jointName;
} joint_controller_descriptor;


/// @brief Holds INCOMING accelerometer data, NOT part of the accelerometer driver
typedef struct
{
  /// @brief X axis acceleration
  int16_t xAcc;
  
  /// @brief X axis rotation
  int16_t xRot;
  
  /// @brief Y axis acceleration
  int16_t yAcc;
  
  /// @brief Y axis rotation
  int16_t yRot;
  
  /// @brief Z axis acceleration
  int16_t zAcc;
  
  /// @brief Z axis rotation
  int16_t zRot;

  /// @brief Flags new X acceleration data on arrival
  uint8_t newXAcc;
  
  /// @brief Flags new Y acceleration data on arrival
  uint8_t newYAcc;
  
  /// @brief Flags new Z acceleration data on arrival
  uint8_t newZAcc;

  /// @brief Flags new X rotation data on arrival
  uint8_t newXRot;
  
  /// @brief Flags new Y rotation data on arrival
  uint8_t newYRot;
  
  /// @brief Flags new Z rotation data on arrival
  uint8_t newZRot;
} accelerometer_inData;

//////////////////
//Public functions
//////////////////


//----------------------
//Joint control handlers
//----------------------

/// @brief Main function to run the controller on both joints
void controller_interface_update_controller();

/// @brief Public function to get the current positional setpoint of a joint
/// @param controllerSelect One of two joints available to the MCU
/// @return Joint positional setpoints in radians relative to its zero position
float controller_interface_get_setpoint(uint8_t controllerSelect);


/// @brief Public function to set the positional setpoint of a joint
/// @param controllerSelect One of two joints available to the MCU
/// @param setPoint Joint positional setpoint in rads relative to its zero position
void controller_interface_set_setpoint(uint8_t controllerSelect, float setPoint);


/// @brief Public function to get the position of a joint
/// @param controllerSelect One of two joints available to the MCU
/// @return Joint position in radians relative to its zero position
float controller_interface_get_position(uint8_t controllerSelect);


/// @brief Public function to trigger an update of the joint's position
/// @param controllerSelect One of two joints available to the MCU
void controller_interface_update_position(uint8_t controllerSelect);


/// @brief Public function to get the current position of a joint
/// @param controllerSelect One of two joints available to the MCU
/// @param position Joint position in rads relative to its zero position
void controller_interface_set_position(uint8_t controllerSelect, float position);


/// @brief Public function to get the current positional error of a joint
/// @param controllerSelect One of two joints available to the MCU
/// @return Joint positional error in rads relative to its setpoint
float controller_interface_get_error(uint8_t controllerSelect);


/// @brief Public function to trigger a calculation of the joint's positional error
/// @param controllerSelect One of two joints available to the MCU
void controller_interface_update_error(uint8_t controllerSelect);


/// @brief Public function to set the positional error of a joint
/// @param controllerSelect One of two joints available to the MCU
/// @param error Joint positional error in rads
void controller_interface_set_error(uint8_t controllerSelect, float error);


/// @brief Public function to get the moving state value of the joint
/// @param controllerSelect One of two joints available to the MCU
/// @return isMoving flag
uint8_t controller_interface_get_moving(uint8_t controllerSelect);


/// @brief Public function to set the moving state flag of the joint
/// @param controllerSelect One of two joints available to the MCU
void controller_interface_set_moving(uint8_t controllerSelect);


/// @brief Public function to clear the moving state flag of the joint
/// @param controllerSelect One of two joints available to the MCU
void controller_interface_clear_moving(uint8_t controllerSelect);


/// @brief Public function to set the power of the joint
/// @param controllerSelect One of two joints available to the MCU
/// @param power Percentage of maximum power
void controller_interface_set_power(uint8_t controllerSelect, float power);


/// @brief Public function to trigger an update of the joint's power by PID
/// @param controllerSelect One of two joints available to the MCU
void controller_interface_update_power(uint8_t controllerSelect);

/// @brief Public function to convert the joint's associated encoder count to joint position
/// @param controllerSelect One of two joints available to the MCU
/// @return Joint position in rads
float controller_interface_clicks_to_pos(uint8_t controllerSelect);

/// @brief Public function to request an update from the joint's accelerometer via CAN bus; acc and rot
/// @param controllerSelect One of two joints available to the MCU
/// @param accSelect (One of) the joint's accelerometer(s)
/// @param axis The axis for which information is requested
void controller_interface_request_acc_axis(uint8_t controllerSelect, uint8_t accSelect, char axis);


//----------------------
//Accelerometer data handlers
//----------------------

/// @brief Public function to get the accelerometer X axis acceleration
/// @param accSelect The relevant accelerometer inData struct
/// @return X axis acceleration raw value
int16_t controller_interface_acc_getX(uint8_t accSelect);


/// @brief Public function to get the accelerometer Y axis acceleration
/// @param accSelect The relevant accelerometer inData struct
/// @return Y axis acceleration raw value
int16_t controller_interface_acc_getY(uint8_t accSelect);


/// @brief Public function to get the accelerometer Z axis acceleration
/// @param accSelect The relevant accelerometer inData struct
/// @return Z axis acceleration raw value
int16_t controller_interface_acc_getZ(uint8_t accSelect);


/// @brief Public function to set the accelerometer X axis acceleration
/// @param accSelect The relevant accelerometer inData struct
/// @param accVal X axis acceleration raw value
void controller_interface_acc_setX(uint8_t accSelect, int16_t accVal);

/// @brief Public function to set the accelerometer Y axis acceleration
/// @param accSelect The relevant accelerometer inData struct
/// @param accVal Y axis acceleration raw value
void controller_interface_acc_setY(uint8_t accSelect, int16_t accVal);


/// @brief Public function to set the accelerometer Z axis acceleration
/// @param accSelect The relevant accelerometer inData struct
/// @param accVal Z axis acceleration raw value
void controller_interface_acc_setZ(uint8_t accSelect, int16_t accVal);


/// @brief Public function to get the accelerometer X axis rotation rate
/// @param accSelect The relevant accelerometer inData struct
/// @return X axis rotation rate raw value
int16_t controller_interface_rot_getX(uint8_t accSelect);


/// @brief Public function to get the accelerometer Y axis rotation rate
/// @param accSelect The relevant accelerometer inData struct
/// @return Y axis rotation rate raw value
int16_t controller_interface_rot_getY(uint8_t accSelect);


/// @brief Public function to get the accelerometer Z axis rotation rate
/// @param accSelect The relevant accelerometer inData struct
/// @return Z axis rotation rate raw value
int16_t controller_interface_rot_getZ(uint8_t accSelect);


/// @brief Public function to set the accelerometer X axis rotation rate
/// @param accSelect The relevant accelerometer inData struct
/// @param rotVal X axis rotation rate raw value
void controller_interface_rot_setX(uint8_t accSelect, int16_t rotVal);


/// @brief Public function to set the accelerometer Y axis rotation rate
/// @param accSelect The relevant accelerometer inData struct
/// @param rotVal Y axis rotation rate raw value
void controller_interface_rot_setY(uint8_t accSelect, int16_t rotVal);


/// @brief Public function to set the accelerometer Z axis rotation rate
/// @param accSelect The relevant accelerometer inData struct
/// @param rotVal Z axis rotation rate raw value
void controller_interface_rot_setZ(uint8_t accSelect, int16_t rotVal);


/// @brief Public function to get the new X acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
/// @return X axis acceleration new data flag
uint8_t controller_interface_acc_get_newX(uint8_t accSelect);


/// @brief Public function to get the new Y acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
/// @return Y axis acceleration new data flag
uint8_t controller_interface_acc_get_newY(uint8_t accSelect);


/// @brief Public function to get the new Z acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
/// @return Z axis acceleration new data flag
uint8_t controller_interface_acc_get_newZ(uint8_t accSelect);


/// @brief Public function to set the new X acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_acc_set_newX(uint8_t accSelect);


/// @brief Public function to set the new Y acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_acc_set_newY(uint8_t accSelect);


/// @brief Public function to set the new Z acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_acc_set_newZ(uint8_t accSelect);


/// @brief Public function to clear the new X acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_acc_clear_newX(uint8_t accSelect);


/// @brief Public function to clear the new Y acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_acc_clear_newY(uint8_t accSelect);


/// @brief Public function to clear the new Z acceleration data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_acc_clear_newZ(uint8_t accSelect);


/// @brief Public function to get the new X rotation data flag
/// @param accSelect The relevant accelerometer inData struct
/// @return Z axis rotation new data flag
uint8_t controller_interface_rot_get_newX(uint8_t accSelect);


/// @brief Public function to get the new Y rotation data flag
/// @param accSelect The relevant accelerometer inData struct
/// @return Y axis rotation new data flag
uint8_t controller_interface_rot_get_newY(uint8_t accSelect);


/// @brief Public function to get the new Z rotation data flag
/// @param accSelect The relevant accelerometer inData struct
/// @return Z axis rotation new data flag
uint8_t controller_interface_rot_get_newZ(uint8_t accSelect);


/// @brief Public function to set the new X rotation data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_rot_set_newX(uint8_t accSelect);


/// @brief Public function to set the new Y rotation data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_rot_set_newY(uint8_t accSelect);


/// @brief Public function to set the new Z rotation data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_rot_set_newZ(uint8_t accSelect);


/// @brief Public function to clear the new X rotation data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_rot_clear_newX(uint8_t accSelect);


/// @brief Public function to clear the new Y rotation data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_rot_clear_newY(uint8_t accSelect);


/// @brief Public function to clear the new Z rotation data flag
/// @param accSelect The relevant accelerometer inData struct
void controller_interface_rot_clear_newZ(uint8_t accSelect);


//---------------------
//Polling flag handlers
//---------------------

/// @brief Public function to poll the timer driven accelerometer poll flag
/// @return Status of the accelerometer poll flag
uint8_t controller_interface_get_acc_poll();


/// @brief Public function to read the timer driven motor poll flag
/// @return Status of the motor poll flag
uint8_t controller_interface_get_mtr_poll();


/// @brief Puclic function to set the accelerometer poll flag
void controller_interface_set_acc_poll();


/// @brief Puclic function to set the motor poll flag
void controller_interface_set_mtr_poll();

/// @brief Public function to clear the accelerometer poll flag
void controller_interface_clear_acc_poll();

/// @brief Public function to clear the motor poll flag
void controller_interface_clear_mtr_poll();

uint8_t controller_interface_get_upd_ctrl();

void controller_interface_set_upd_ctrl();

void controller_interface_clear_upd_ctrl();

uint8_t controller_interface_get_upd_telemetry();

void controller_interface_set_upd_telemetry();

void controller_interface_clear_upd_telemetry();

///////////////////
//Private functions
///////////////////

/**
 * Private functions are not described individually, but serve the same purpose as their
 * public counterparts. The joint controller descriptors are static structs defined
 * in the .c file and correspond to the unique joints of the robotic arm.
 */



//----------------------
//Joint control handlers
//----------------------
float joint_controller_get_setpoint(joint_controller_descriptor* joint);
void joint_controller_set_setpoint(joint_controller_descriptor* joint, float setpoint);

float joint_controller_get_position(joint_controller_descriptor* joint);
void joint_controller_update_position(joint_controller_descriptor* joint);
void joint_controller_set_position(joint_controller_descriptor* joint, float position);

float joint_controller_get_error(joint_controller_descriptor* joint);
void joint_controller_update_error(joint_controller_descriptor* joint);
void joint_controller_set_error(joint_controller_descriptor* joint, float error);

uint8_t joint_controller_get_moving(joint_controller_descriptor* joint);
void joint_controller_set_moving(joint_controller_descriptor* joint);
void joint_controller_clear_moving(joint_controller_descriptor* joint);

void joint_controller_set_power(joint_controller_descriptor* joint, float power);
void joint_controller_update_power(joint_controller_descriptor* joint);

float joint_controller_clicks_to_pos(joint_controller_descriptor* joint);
void joint_controller_adjust_enc_sp(joint_controller_descriptor* joint);

void joint_controller_request_acc_axis(joint_controller_descriptor* joint, uint8_t accSelect, char axis);
float joint_controller_acceleration_to_angle(joint_controller_descriptor* joint);



//----------------------
//Accelerometer data handlers
//----------------------
int16_t controller_acc_getX(accelerometer_inData* accSelect);
int16_t controller_acc_getY(accelerometer_inData* accSelect);
int16_t controller_acc_getZ(accelerometer_inData* accSelect);

void controller_acc_setX(accelerometer_inData* accSelect, int16_t accVal);
void controller_acc_setY(accelerometer_inData* accSelect, int16_t accVal);
void controller_acc_setZ(accelerometer_inData* accSelect, int16_t accVal);

int16_t controller_rot_getX(accelerometer_inData* accSelect);
int16_t controller_rot_getY(accelerometer_inData* accSelect);
int16_t controller_rot_getZ(accelerometer_inData* accSelect);

void controller_rot_setX(accelerometer_inData* accSelect, int16_t rotVal);
void controller_rot_setY(accelerometer_inData* accSelect, int16_t rotVal);
void controller_rot_setZ(accelerometer_inData* accSelect, int16_t rotVal);

uint8_t controller_acc_get_newX(accelerometer_inData* accSelect);
uint8_t controller_acc_get_newY(accelerometer_inData* accSelect);
uint8_t controller_acc_get_newZ(accelerometer_inData* accSelect);

void controller_acc_set_newX(accelerometer_inData* accSelect);
void controller_acc_set_newY(accelerometer_inData* accSelect);
void controller_acc_set_newZ(accelerometer_inData* accSelect);

void controller_acc_clear_newX(accelerometer_inData* accSelect);
void controller_acc_clear_newY(accelerometer_inData* accSelect);
void controller_acc_clear_newZ(accelerometer_inData* accSelect);

uint8_t controller_rot_get_newX(accelerometer_inData* accSelect);
uint8_t controller_rot_get_newY(accelerometer_inData* accSelect);
uint8_t controller_rot_get_newZ(accelerometer_inData* accSelect);

void controller_rot_set_newX(accelerometer_inData* accSelect);
void controller_rot_set_newY(accelerometer_inData* accSelect);
void controller_rot_set_newZ(accelerometer_inData* accSelect);

void controller_rot_clear_newX(accelerometer_inData* accSelect);
void controller_rot_clear_newY(accelerometer_inData* accSelect);
void controller_rot_clear_newZ(accelerometer_inData* accSelect);



#endif //JOINT_CONTROLLER_H