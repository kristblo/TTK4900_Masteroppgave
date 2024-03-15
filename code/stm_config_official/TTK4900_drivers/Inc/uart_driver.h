#ifndef UART_DRIVER_H
#define UART_DRIVER_H
//External library includes
#include "string.h"
#include <stdio.h>

//CubeMX generated includes
#include "usart.h"

//TTK4900 library includes
#include "unit_config.h"

//------FILE BEGIN
void uart_send_string(char* str);

void uart_parse_input(char* input, 
                      uint8_t* buffer, 
                      uint8_t bufferLength, 
                      uint8_t* bufferPos);

void uart_hmi_rx_handler();

void uart_hmi_init();

void uart_ros_rx_handler();
void uart_ros_init();

#endif //UART_DRIVER_H