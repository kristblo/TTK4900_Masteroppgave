#ifndef ROS_UART_PARSER_H
#define ROS_UART_PARSER_H

/**
  ******************************************************************************
  * @file    ros_uart_parser.h
  * @brief   This file contains all the function prototypes for the 
  *           ros_uart_parser.c file
  *        
  ******************************************************************************
  * @attention
  *
  * ROS/UART serial interface driver for the TTK4900 Master project of Kristian Blom, 
  * spring semester of 2024. The driver makes use of the STM32's UART peripheral,
  * as well as the string command parser module and C string libraries to do
  * basic processing of incoming and outgoing UART data.
  * 
  * The parser/handler functions are triggered by the HAL_UART_RxCpltCallback
  *
  ******************************************************************************
  */

//External library includes
#include "string.h"
#include <stdio.h>
#include "stdint.h"

//CubeMX generated includes

//TTK4900 library includes
#include "unit_config.h"
#include "joint_controller.h"
#include "state_machine.h"
#include "can_driver.h"


void ros_interface_set_rxBuffer(uint8_t* input);

void ros_interface_get_rxBuffer(uint8_t* target);

void ros_interface_clear_rxBuffer();

void ros_interface_set_newMsgFlag();

uint8_t ros_interface_get_newMsgFlag();

void ros_interface_clear_newMsgFlag();

void ros_interface_parse_input();

void ros_interface_queue_setpoints();

void ros_interface_dequeue_setpoints();

void ros_interface_send_setpoints();


#endif //ROS_UART_PARSER_H