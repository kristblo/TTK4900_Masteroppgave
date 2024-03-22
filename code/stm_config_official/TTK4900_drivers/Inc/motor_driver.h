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

//-----Structures-----


/// @brief Contains static and state information relevant to the operation of a motor driver.
typedef struct
{
  uint8_t motorId;
  uint8_t voltageLimit;
  uint8_t voltagePctCap;
  int8_t motorPolarity;
  TIM_TypeDef* motorTimer;
  TIM_TypeDef* encoderTimer;
  int32_t resolution; //encoder clicks per rad or mm
  uint16_t encoderInitCount;
  int32_t encoderTotalInit;
  int32_t encoderTotalSetpoint;
  int32_t encoderTotalCount;
  int32_t encoderPreviousCount;
  uint8_t isMoving;
  char* motorName; //flexible array must be at the end of a struct
  //TODO: Add regulator parameters
} motor_control_descriptor;



//-----Interface functions (public)-----

/// @brief Initialize the controller interface
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively
void motor_interface_controller_init(uint8_t motorSelect);


/// @brief Update the power setting of the selected motor based on the relevant controller (P/I/D) 
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively 
void motor_interface_update_power(uint8_t motorSelect);


/// @brief Update the total number of registered encoder counts since init
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively
void motor_interface_update_tot_cnt(uint8_t motorSelect);


/// @brief Returns the setpoint of the selected motors
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively
/// @return encoderTotalSetpoint 
int32_t motor_interface_get_setpoint(uint8_t motorSelect);


/// @brief Returns the number of total registered encoder counts since init
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively
/// @return encoderTotalCount 
int32_t motor_interface_get_total_count(uint8_t motorSelect);


/// @brief Returns the current encoder hardware count
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively
/// @return TIMx->CNT
uint16_t motor_interface_get_encoder_count(uint8_t motorSelect);

/// @brief 
/// @param motorSelect 
/// @return 
uint8_t motor_interface_get_id(uint8_t motorSelect);


/// @brief 
/// @param motorSelect 
/// @return 
int32_t motor_interface_get_resolution(uint8_t motorSelect);


uint8_t motor_interface_get_moving(uint8_t motorSelect);


/// @brief Lets the user set the motor power setting directly
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively
/// @param direction 0 or 1 for forwards or backwards, respectively
/// @param power percentage of input voltage
void motor_interface_set_power(uint8_t motorSelect, uint8_t direction, double power);


/// @brief Lets the user set the motor setpoint directly
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively
/// @param setpoint int32_t
void motor_interface_set_setpoint(uint8_t motorSelect, int32_t setpoint);


/// @brief Lets the user increment or decrement the motor encoder setpoint
/// @param motorSelect 0 or 1 for motor0 or motor1 respectively
/// @param delta int32_t, number of encoder clicks by which the setpoint is changed
void motor_interface_delta_setpoint(uint8_t motorSelect, int32_t delta);



//-----Driver functions (private)-----


/// @brief Initialise motor controller descriptor
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
void motor_driver_init(motor_control_descriptor* motor);


/// @brief Sets the power of the selected motor based on a controller heuristic
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
void motor_driver_update_power(motor_control_descriptor* motor);


/// @brief Updates the encoder total count based on the relevant heuristic
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
void motor_driver_update_tot_cnt(motor_control_descriptor* motor);


/// @brief Gets the total encoder count setpoint of the selected motor
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
/// @return encoderTotalSetpoint
int32_t motor_driver_get_setpoint(motor_control_descriptor* motor);


/// @brief Gets the total encoder count since init
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
/// @return encoderTotalCount
int32_t motor_driver_get_total_cnt(motor_control_descriptor* motor);


/// @brief Gets the current hardware encoder count
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
/// @return TIMx->CNT
uint16_t motor_driver_get_encoder_cnt(motor_control_descriptor* motor);


/// @brief 
/// @param motorSelect 
/// @return 
uint8_t motor_driver_get_id(motor_control_descriptor* motor);


/// @brief 
/// @param motor 
/// @return 
int32_t motor_driver_get_resolution(motor_control_descriptor* motor);


/// @brief 
/// @param motor 
/// @return 
uint8_t motor_driver_get_moving(motor_control_descriptor* motor);

/// @brief Set the power of a motor, limited by the motor's safety cap
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
/// @param direction 0 or 1 for forwards or backwards, respectively
/// @param power percentage of input voltage
void motor_driver_set_power(motor_control_descriptor* motor, uint8_t direction, double power);


/// @brief Sets the total encoder count setpoint of the selected motor
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
/// @param setpoint Total encoder count
void motor_driver_set_setpoint(motor_control_descriptor* motor, int32_t setpoint);


/// @brief Changes the encoder setpoint of the relevant motor
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
/// @param delta Change to encoder count setpoint
void motor_driver_delta_setpoint(motor_control_descriptor* motor, int32_t delta);


/// @brief Sets the duty cycle of the selected PWM timer as a percentage of max
/// @param timerCounter Pointer to the relevant TIMx->CNT register
/// @param pct Percentage of maximum duty cycle
void motor_driver_set_pwm_dc(uint32_t* timerCounter, double pct);


/// @brief Calculates the safe power percentage limit based on the descriptor struct
/// @param motor Pointer to the relevant motor struct, motor0 or motor1
void motor_driver_calc_safe_vlt(motor_control_descriptor* motor);


/// @brief Forward is the direction of increasing encoder count
/// @param pct Percentage of input voltage
/// @param mtr Pointer to timer
void motor_driver_go_forward(double pct, TIM_TypeDef* mtr, int8_t polarity);


/// @brief Backward is the direction of decreasing encoder count
/// @param pct Percentage of input voltage
/// @param mtr Pointer to timer
void motor_driver_go_backward(double pct, TIM_TypeDef* mtr, int8_t polarity);


#endif //MOTOR_DRIVER_H