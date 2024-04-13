#ifndef UART_DRIVER_H
#define UART_DRIVER_H

/**
  ******************************************************************************
  * @file    uart_driver.h
  * @brief   This file contains all the function prototypes for the 
  *           string_cmd_parser.c file
  *        
  ******************************************************************************
  * @attention
  *
  * UART serial interface driver for the TTK4900 Master project of Kristian Blom, 
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

//CubeMX generated includes
#include "usart.h"

//TTK4900 library includes
#include "unit_config.h"
#include "string_cmd_parser.h"
#include "ros_uart_parser.h"
#include "joint_controller.h"
#include "state_machine.h"

//------FILE BEGIN

/// @brief Sends a string over the UART peripheral interface
/// @param str string to send
void uart_send_string(char* str);


/// @brief Reads incoming UART data and enables a keyboard based HMI to the STM32
/// @param input Last incoming byte
/// @param buffer String buffer holding the currently relevant strings
/// @param bufferLength Length of buffer
/// @param bufferPos Keyboard cursor position in buffer
void uart_parse_hmi_input(char* input, 
                      uint8_t* buffer, 
                      uint8_t bufferLength, 
                      uint8_t* bufferPos);


void uart_parse_ros_input(char* input);

/// @brief Handles incoming UART data when the peripheral is used as HMI
void uart_hmi_rx_handler();

/// @brief Initializes the UART peripheral as HMI
void uart_hmi_init();

/// @brief Handles incoming UART data when the peripheral is used for ROS
void uart_ros_rx_handler();

/// @brief Initializes the UART peripheral as ROS interface
void uart_ros_init();

#endif //UART_DRIVER_H