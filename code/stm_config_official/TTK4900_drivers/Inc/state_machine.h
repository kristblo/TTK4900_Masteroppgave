#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/**
  ******************************************************************************
  * @file    state_machine.h
  * @brief   This file contains all the function prototypes and struct
  *           definitions for the state_machine.c file
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
  ******************************************************************************
  */


//External library includes
#include "stdint.h"

//CubeMX generated includes
#include "tim.h"

//TTK4900 library includes
#include "unit_config.h"
#include "uart_driver.h"
#include "joint_controller.h"
#include "can_driver.h"


//-----Structures-----
typedef enum
{
  GS_ERROR,
  GS_IDLE,
  GS_CALIBRATING,
  GS_OPERATING,
  num_gStates,
} global_state;

typedef enum
{
  CS_ERROR,
  CS_RAIL,
  CS_SHOULDER,
  CS_ELBOW,
  CS_WRIST,
  CS_TWIST,
  CS_PINCH,
  num_cStates
} calibration_state;


//////////////////
//Public functions
//////////////////
uint8_t state_interface_get_global_state();
void state_interface_set_global_state(uint8_t inState);
void state_interface_broadcast_global_state();

uint8_t state_interface_get_calibration_state();
void state_interface_set_calibration_state(uint8_t inState);
void state_interface_broadcast_calibration_state();


void state_interface_set_es_flag();
uint8_t state_interface_get_es_flag();
void state_interface_clear_es_flag();

void state_interface_set_tw_flag();
uint8_t state_interface_get_tw_flag();
void state_interface_clear_tw_flag();



///////////////////
//Private functions
///////////////////

void state_calibrate_rail();
void state_calibrate_shoulder();
void state_calibrate_elbow();
void state_calibrate_wrist();
void state_calibrate_twist();
void state_calibrate_pinch();

#endif //STATE_MACHINE_H