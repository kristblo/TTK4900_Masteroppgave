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


//Public functions
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

//Private functions

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

#endif //JOINT_CONTROLLER_H