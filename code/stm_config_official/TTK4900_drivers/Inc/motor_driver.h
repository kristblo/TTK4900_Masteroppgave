#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

//External library includes
#include "stdint.h"
#include "math.h"

//CubeMX generated includes
#include "tim.h"

//TTK4900 library includes
#include "unit_config.h"
#include "uart_driver.h"
//------FILE BEGIN--------
typedef struct
{
  uint8_t motorId;
  uint8_t voltageLimit;
  uint8_t voltagePctCap;
  TIM_TypeDef* motorTimer;
  TIM_TypeDef* encoderTimer;
  uint16_t encoderInitCount;
  int32_t encoderTotalInit;
  int32_t encoderTotalSetpoint;
  int32_t encoderTotalCount;
  char* motorName; //flexible array must be at the end of a struct
  //TODO: Add regulator parameters
} motor_control_descriptor;



void motor_interface_update_power(uint8_t motorSelect);
void motor_interface_set_power(uint8_t motorSelect, uint8_t direction, double power);
int32_t motor_interface_get_setpoint(uint8_t motorSelect);
void motor_interface_set_setpoint(uint8_t motorSelect, int32_t setpoint);


void motor_driver_update_power(motor_control_descriptor* motor);
void motor_driver_set_pwm_dc(uint32_t* timerCounter, double pct);
//Forward is the direction of increasing encoder count
void motor_driver_go_forward(double pct, TIM_TypeDef* mtr);
//Backward is the direction of decreasing encoder count
void motor_driver_go_backward(double pct, TIM_TypeDef* mtr);

//Use the struct to set a safe voltage
void motor_driver_set_power(motor_control_descriptor* motor, uint8_t direction, double power);
void motor_driver_set_setpoint(motor_control_descriptor* motor, int32_t setpoint);
int32_t motor_driver_get_setpoint(motor_control_descriptor* motor);
// int32_t motor_driver_get_total(uint8_t motorSelect);
// uint16_t motor_driver_get_encoder_cnt(uint8_t motorSelect);
// //Initialise motor controller
// void motor_driver_controller_init(motor_control_descriptor motor);

#endif //MOTOR_DRIVER_H