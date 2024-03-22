#include "shoulder_controller.h"

controller_descriptor shoulder =
{
  .inputSource = 'A',
  .posSetpoint = 0,
  .posCurrent = 0,
  .posError = 0,
  .isMoving = 0,
};

void controller_interface_set_setpoint(float radSetpoint)
{
  controller_set_setpoint(&shoulder, radSetpoint);
}

float controller_interface_get_setpoint()
{
  return controller_get_setpoint(&shoulder);
}

void controller_interface_update_power()
{
  controller_update_power(&shoulder);
}

void controller_interface_update_position()
{
  controller_update_position(&shoulder);
}

void controller_interface_request_position()
{
  controller_request_position(&shoulder);
}

float controller_interface_get_position()
{
  return controller_get_position(&shoulder);
}

void controller_interface_update_pos_error()
{
  controller_update_pos_error(&shoulder);
}

int32_t controller_interface_error_as_clicks()
{
  return controller_error_as_clicks(&shoulder);
}

void controller_interface_adjust_enc_sp()
{
  return controller_adjust_enc_sp(&shoulder);
}

uint8_t controller_interface_get_moving()
{
  return controller_get_moving(&shoulder);
}

void controller_interface_set_moving()
{
  controller_set_moving(&shoulder);
}

void controller_interface_clear_moving()
{
  controller_clear_moving(&shoulder);
}


float controller_get_setpoint(controller_descriptor* controller)
{
  return (controller->posSetpoint);
}

void controller_update_power(controller_descriptor* controller)
{
  controller->posError = controller->posSetpoint - controller->posCurrent;
  double power = (controller->posError)*0.002;

  if(power < 0)
  {
    motor_interface_set_power(1, 1, fabs(power));
  }
  else
  {
    motor_interface_set_power(1, 0, power);
  }  
}

float controller_get_position(controller_descriptor* controller)
{
  return (controller->posCurrent);
}
void controller_request_position(controller_descriptor* controller)
{
  uint8_t request[8] = {'A', 1, 0x2A, 0,0,0,0,0};
  can_interface_send_msg(request, 0, 0, 0);
}

void controller_update_position(controller_descriptor* controller)
{
  //TODO: more interfaces, or just rename this whole file to shoulder control
  //Also, raw data should be converted to degrees
  // if(can_interface_get_newrxflag() == 1)
  // {
  //   uint8_t accelerationData[8];
  //   can_interface_get_acc_rxMailbox(accelerationData);
  //   int16_t acceleration = (int16_t)((accelerationData[2] << 8) | accelerationData[3]);
  //   controller->posCurrent = controller_acc_to_radians(controller, acceleration);
  //   can_interface_clear_newrx();
  //   //This might be dangerous
  //   //controller_request_position(controller);

  // }
}

void controller_set_setpoint(controller_descriptor* controller, float setpoint)
{
  controller->posSetpoint = setpoint;
}

float controller_acc_to_radians(controller_descriptor* controller, int16_t rawAcc)
{
  double middle = ((double)(rawAcc-2500)/16384);
  double radians = asin(middle);

  int32_t output = (int32_t)(radians*100);
  char* debug[64];
  sprintf(debug, "debugout: %i\n\r", rawAcc);
  uart_send_string(debug);

  return (float)radians;
}

//TODO: function to get error from encoder position

void controller_update_pos_error(controller_descriptor* controller)
{
  controller->posError = controller->posSetpoint - controller->posCurrent;
}

int32_t controller_error_as_clicks(controller_descriptor* controller)
{
  int32_t resolution = motor_interface_get_resolution(1); //shoulder motor res
  int32_t errorAsClicks = (int32_t)(controller->posError)*resolution;
  
  return errorAsClicks;
}

void controller_adjust_enc_sp(controller_descriptor* controller)
{
  int32_t error = controller_error_as_clicks(controller);
  motor_interface_delta_setpoint(1, error);
}

uint8_t controller_get_moving(controller_descriptor* controller)
{
  return controller->isMoving;
}

void controller_set_moving(controller_descriptor* controller)
{
  controller->isMoving = 1;
}

void controller_clear_moving(controller_descriptor* controller)
{
  controller->isMoving = 0;
}