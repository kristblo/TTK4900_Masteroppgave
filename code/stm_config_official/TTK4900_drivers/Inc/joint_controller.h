#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

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

  /// @brief The joint's position error in radians/mm, relative to its setpoint
  float posError;
  
  /// @brief Whether the joint is in a "moving" state
  uint8_t isMoving;

  /// @brief Link to the corresponding motor_control_descriptor
  uint8_t motorNum;

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


/// @brief Holds INCOMING accelerometer data
typedef struct
{
  int16_t xAcc;
  int16_t yAcc;
  int16_t zAcc;
  uint8_t newXAcc;
  uint8_t newYAcc;
  uint8_t newZAcc;
} accelerometer_inData;

//Public functions

//----------------------
//Joint control handlers
//----------------------
float controller_interface_get_setpoint(uint8_t controllerSelect);
void controller_interface_set_setpoint(uint8_t controllerSelect, float setPoint);

float controller_interface_get_position(uint8_t controllerSelect);
void controller_interface_set_position(uint8_t controllerSelect, float position);

float controller_interface_get_error(uint8_t controllerSelect);
void controller_interface_update_error(uint8_t controllerSelect);
void controller_interface_set_error(uint8_t controllerSelect, float error);

uint8_t controller_interface_get_moving(uint8_t controllerSelect);
void controller_interface_set_moving(uint8_t controllerSelect);
void controller_interface_clear_moving(uint8_t controllerSelect);

void controller_interface_set_power(uint8_t controllerSelect, float power);
void controller_interface_update_power(uint8_t controllerSelect);

float controller_interface_clicks_to_pos(uint8_t controllerSelect);
void controller_interface_adjust_enc_sp(uint8_t controllerSelect);


//----------------------
//Accelerometer data handlers
//----------------------

int16_t controller_interface_acc_getX(uint8_t accSelect);
int16_t controller_interface_acc_getY(uint8_t accSelect);
int16_t controller_interface_acc_getZ(uint8_t accSelect);
void controller_interface_acc_setX(uint8_t accSelect, int16_t accVal);
void controller_interface_acc_setY(uint8_t accSelect, int16_t accVal);
void controller_interface_acc_setZ(uint8_t accSelect, int16_t accVal);

uint8_t controller_interface_acc_get_newX(uint8_t accSelect);
uint8_t controller_interface_acc_get_newY(uint8_t accSelect);
uint8_t controller_interface_acc_get_newZ(uint8_t accSelect);

void controller_interface_acc_set_newX(uint8_t accSelect);
void controller_interface_acc_set_newY(uint8_t accSelect);
void controller_interface_acc_set_newZ(uint8_t accSelect);

void controller_interface_acc_clear_newX(uint8_t accSelect);
void controller_interface_acc_clear_newY(uint8_t accSelect);
void controller_interface_acc_clear_newZ(uint8_t accSelect);

//Private functions

//----------------------
//Joint control handlers
//----------------------
float joint_controller_get_setpoint(joint_controller_descriptor* joint);
void joint_controller_set_setpoint(joint_controller_descriptor* joint, float setpoint);

float joint_controller_get_position(joint_controller_descriptor* joint);
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

void joint_controller_request_acceleration(joint_controller_descriptor* joint);
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

uint8_t controller_acc_get_newX(accelerometer_inData* accSelect);
uint8_t controller_acc_get_newY(accelerometer_inData* accSelect);
uint8_t controller_acc_get_newZ(accelerometer_inData* accSelect);

void controller_acc_set_newX(accelerometer_inData* accSelect);
void controller_acc_set_newY(accelerometer_inData* accSelect);
void controller_acc_set_newZ(accelerometer_inData* accSelect);

void controller_acc_clear_newX(accelerometer_inData* accSelect);
void controller_acc_clear_newY(accelerometer_inData* accSelect);
void controller_acc_clear_newZ(accelerometer_inData* accSelect);

#endif //JOINT_CONTROLLER_H