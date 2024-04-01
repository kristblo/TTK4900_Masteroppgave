#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H


/**
  ******************************************************************************
  * @file    motor_driver.h
  * @brief   This file contains all the function prototypes and struct
  *           definitions for the motor_driver.c file
  *        
  ******************************************************************************
  * @attention
  *
  * Motor driver for the TTK4900 Master project of Kristian Blom, spring
  * semester of 2024. The driver makes use of the STM32's timer peripheral to 
  * generate PWM signals driving the DRV8251A H-bridge motor drivers. The motor
  * descriptor structs hold information relevant to the control of each motor,
  * most importantly the safe voltage limit for each motor embedded in the 
  * robotic arm. Additionally, "trip" information such as the total number
  * of encoder counts registered since startup, critical to the state estimation
  * of the arm.
  * 
  * As all STM32s are responsible for the driving of two motors each,
  * the .c file defines two instances of the descriptor struct, motor1 and 
  * motor2.
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

//-----Structures-----


/// @brief Contains static and state information relevant to the operation of a motor driver.
typedef struct
{
  /// @brief The motor's unique ID, essential for CAN messaging
  uint8_t motorId;
  
  /// @brief Safe voltage limit, as stated in the motor's datasheet
  uint8_t voltageLimit;

  /// @brief Safe voltage percentage cap, given by input voltage and safe limit
  uint8_t voltagePctCap;

  /// @brief The motor's polarity, which pole is connected to +/- on the driver
  int8_t motorPolarity;

  /// @brief The MCU timer peripheral which drives the motor
  TIM_TypeDef* motorTimer;

  /// @brief The MCU timer peripheral which registers the motor's encoder
  TIM_TypeDef* encoderTimer;

  /// @brief Relation between number of encoder click per mm or rad of movement
  int32_t resolution;

  /// @brief Number of encoder clicks counted on startup (nominally 0)
  uint16_t encoderInitCount;

  /// @brief Number of total encoder clicks counted on startup (nominally 0)
  int32_t encoderTotalInit;

  /// @brief Setpoint for motor encoder count, relevant if circumventing joint controller
  int32_t encoderTotalSetpoint;

  /// @brief Total number of encoder clicks registered since startup
  int32_t encoderTotalCount;

  /// @brief The previous total encoder count, used for updating total
  int32_t encoderPreviousCount;
  
  /// @brief The most recently registered increment/decrement in encoder count, essentially movement rate
  int32_t mostRecentDelta;

  /// @brief Whether the motor is moving, assumed true if mostRecentDelta>50
  uint8_t isMoving;

  /// @brief Human readable name of the motor
  char* motorName; //flexible array must be at the end of a struct
  //TODO: Add regulator parameters
} motor_descriptor;

//////////////////
//Public functions
//////////////////

/// @brief Set to zero the motor encoder counters
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
void motor_interface_zero(uint8_t motorSelect);


/// @brief Update the power setting of the selected motor based on the relevant controller (P/I/D) 
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively 
void motor_interface_update_power(uint8_t motorSelect);


/// @brief Update the total number of registered encoder counts since init
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
void motor_interface_update_tot_cnt(uint8_t motorSelect);


/// @brief Returns the setpoint of the selected motors
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @return encoderTotalSetpoint 
int32_t motor_interface_get_setpoint(uint8_t motorSelect);


/// @brief Returns the number of total registered encoder counts since init
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @return encoderTotalCount 
int32_t motor_interface_get_total_count(uint8_t motorSelect);


/// @brief Returns the current encoder hardware count
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @return TIMx->CNT
uint16_t motor_interface_get_encoder_count(uint8_t motorSelect);


/// @brief Returns the numerical ID of the motor
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @return motorID
uint8_t motor_interface_get_id(uint8_t motorSelect);


/// @brief Returns the resolution of the motor in encoder clicks per rad or mm
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @return resolution
int32_t motor_interface_get_resolution(uint8_t motorSelect);


/// @brief Returns the most recently registered change in encoder increment
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @return mostRecentDelta
int32_t motor_interface_get_delta(uint8_t motorSelect);

/// @brief Returns the isMoving flag
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @return isMoving
uint8_t motor_interface_get_moving(uint8_t motorSelect);


/// @brief Lets the user set the motor power setting directly
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @param direction 0 or 1 for forwards or backwards, respectively
/// @param power percentage of input voltage
void motor_interface_set_power(uint8_t motorSelect, uint8_t direction, double power);


/// @brief Lets the user set the motor setpoint directly
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @param setpoint int32_t
void motor_interface_set_setpoint(uint8_t motorSelect, int32_t setpoint);


/// @brief Lets the user override the registered totalCount
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @param count int32_t
void motor_interface_set_total_count(uint8_t motorSelect, int32_t count);


/// @brief Lets the user increment or decrement the motor encoder setpoint
/// @param motorSelect 0 or 1 for motor1 or motor2 respectively
/// @param delta int32_t, number of encoder clicks by which the setpoint is changed
void motor_interface_delta_setpoint(uint8_t motorSelect, int32_t delta);



///////////////////
//Private functions
///////////////////


/// @brief Zero counters
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
void motor_driver_zero(motor_descriptor* motor);


/// @brief Sets the power of the selected motor based on a controller heuristic
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
void motor_driver_update_power(motor_descriptor* motor);


/// @brief Updates the encoder total count based on the relevant heuristic
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
void motor_driver_update_tot_cnt(motor_descriptor* motor);


/// @brief Gets the total encoder count setpoint of the selected motor
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
/// @return encoderTotalSetpoint
int32_t motor_driver_get_setpoint(motor_descriptor* motor);


/// @brief Gets the total encoder count since init
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
/// @return encoderTotalCount
int32_t motor_driver_get_total_cnt(motor_descriptor* motor);


/// @brief Gets the current hardware encoder count
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
/// @return TIMx->CNT
uint16_t motor_driver_get_encoder_cnt(motor_descriptor* motor);


/// @brief 
/// @param motorSelect 
/// @return 
uint8_t motor_driver_get_id(motor_descriptor* motor);


/// @brief 
/// @param motor 
/// @return 
int32_t motor_driver_get_resolution(motor_descriptor* motor);


/// @brief 
/// @param motor 
/// @return 
uint8_t motor_driver_get_moving(motor_descriptor* motor);


/// @brief 
/// @param motor 
/// @return 
int32_t motor_driver_get_delta(motor_descriptor* motor);

/// @brief Set the power of a motor, limited by the motor's safety cap
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
/// @param direction 0 or 1 for forwards or backwards, respectively
/// @param power percentage of input voltage
void motor_driver_set_power(motor_descriptor* motor, uint8_t direction, double power);


/// @brief Sets the total encoder count setpoint of the selected motor
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
/// @param setpoint Total encoder count
void motor_driver_set_setpoint(motor_descriptor* motor, int32_t setpoint);


/// @brief Sets the total encoder count of the selected motor
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
/// @param count Count to set
void motor_driver_set_total_count(motor_descriptor* motor, int32_t count);


/// @brief Changes the encoder setpoint of the relevant motor
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
/// @param delta Change to encoder count setpoint
void motor_driver_delta_setpoint(motor_descriptor* motor, int32_t delta);


/// @brief Sets the duty cycle of the selected PWM timer as a percentage of max
/// @param timerCounter Pointer to the relevant TIMx->CNT register
/// @param pct Percentage of maximum duty cycle
void motor_driver_set_pwm_dc(uint32_t* timerCounter, double pct);


/// @brief Calculates the safe power percentage limit based on the descriptor struct
/// @param motor Pointer to the relevant motor struct, motor1 or motor2
void motor_driver_calc_safe_vlt(motor_descriptor* motor);


/// @brief Forward is the direction of increasing encoder count
/// @param pct Percentage of input voltage
/// @param mtr Pointer to timer
void motor_driver_go_forward(double pct, TIM_TypeDef* mtr, int8_t polarity);


/// @brief Backward is the direction of decreasing encoder count
/// @param pct Percentage of input voltage
/// @param mtr Pointer to timer
void motor_driver_go_backward(double pct, TIM_TypeDef* mtr, int8_t polarity);


#endif //MOTOR_DRIVER_H