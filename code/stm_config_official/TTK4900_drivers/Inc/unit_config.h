#ifndef UNIT_CONFIG_H
#define UNIT_CONFIG_H

//Global defines
#define MTR1 TIM15
#define MTR2 TIM1
#define ENC1 TIM3
#define ENC2 TIM8

#define PWM_CTR_PRD 7200
#define GLOBAL_DEBUG 1
#define VOLTAGE_IN 20


//Variables specific to each of the three control units
#define TORSO 0
#define SHOULDER 1
#define HAND 2
#define ACTIVE_UNIT TORSO //Change before flashing another unit

//Choose between UART and USB for communication
#define UART_INTERFACE 0
#define USB_INTERFACE 1
#define HW_INTERFACE UART_INTERFACE

//Choose between terminal text commands and ROS
#define CMD_MODE_TERMINAL 0
#define CMD_MODE_ROS 1
#define SW_INTERFACE CMD_MODE_TERMINAL

#if ACTIVE_UNIT == TORSO
  #define CAN_FILTER_M 0x000 //Motor message filter
  #define CAN_FILTERMASK_M 0x300 //Motor message filtermask
  #define CAN_FILTER_A 0x000 //Accelerometer message filter
  #define CAN_FILTERMASK_A 0x0E0 //Accelerometer message filtermask
  #define MTR_POL -1 //Motor polarity, i.e. whether "forward" increases or decreases encoder count
  #define UART_INPUT 1 //Include UART input parsing in build


#elif ACTIVE_UNIT == SHOULDER
  #define CAN_FILTER_M 0x040 //Motor message filter
  #define CAN_FILTERMASK_M 0x3C0 //Motor message filtermask
  #define CAN_FILTER_A 0x000 //Accelerometer message filter
  #define CAN_FILTERMASK_A 0x2E0 //Accelerometer message filtermask
  #define MTR_POL 1 //Motor polarity, i.e. whether "forward" increases or decreases encoder count
  #define UART_INPUT 1 //Include UART input parsing in build



#elif ACTIVE_UNIT == HAND
  #define CAN_FILTER_M 0x080 //Motor message filter
  #define CAN_FILTERMASK_M 0x3C0 //Motor message filtermask
  #define CAN_FILTER_A 0x200 //Accelerometer message filter
  #define CAN_FILTERMASK_A 0x2E0 //Accelerometer message filtermask
  #define MTR_POL -1 //Motor polarity, i.e. whether "forward" increases or decreases encoder count
  #define UART_INPUT 1 //Include UART input parsing in build
#else
  #error NO VALID CONTROL UNIT SELECTED
#endif

//Peripheral activation
void activate_peripherals();


#endif //UNIT_CONFIG_H