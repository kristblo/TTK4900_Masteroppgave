#ifndef SHOULDER_CONTROLLER_H
#define SHOULDER_CONTROLLER_H


//External library includes
#include "string.h"
#include <stdio.h>
#include "math.h"

//CubeMX generated includes


//TTK4900 library includes
#include "uart_driver.h"
#include "accelerometer_driver.h"
#include "motor_driver.h"
#include "can_driver.h"

typedef struct
{
  uint8_t inputSource;
  float posSetpoint;
  float posCurrent;
  float posError;
  uint8_t isMoving;
} controller_descriptor;


// float controller_interface_get_setpoint();
// void controller_interface_update_power();
// float controller_interface_get_position();
// void controller_interface_update_position();
// void controller_interface_request_position();
// void controller_interface_set_setpoint(float radSetpoint);
// void controller_interface_update_pos_error();
// int32_t controller_interface_error_as_clicks();
// void controller_interface_adjust_enc_sp();
// uint8_t controller_interface_get_moving();
// void controller_interface_set_moving();
// void controller_interface_clear_moving();


// float controller_get_setpoint(controller_descriptor* controller);
// void controller_set_setpoint(controller_descriptor* controller, float radSetpoint);
// void controller_update_power(controller_descriptor* controller);
// float controller_get_position(controller_descriptor* controller);
// void controller_request_position(controller_descriptor* controller);
// void controller_update_position(controller_descriptor* controller);
// float controller_acc_to_radians(controller_descriptor* controller, int16_t rawAcc);
// void controller_update_pos_error(controller_descriptor* controller);
// int32_t controller_error_as_clicks(controller_descriptor* controller);
// void controller_adjust_enc_sp(controller_descriptor* controller);
// uint8_t controller_get_moving(controller_descriptor* controller);
// void controller_set_moving(controller_descriptor* controller);
// void controller_clear_moving(controller_descriptor* controller);

#endif //SHOULDER_CONTROLLER_H