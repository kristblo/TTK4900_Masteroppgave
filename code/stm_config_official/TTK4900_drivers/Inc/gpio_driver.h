#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H
/**
  ******************************************************************************
  * @file    gpio_driver.h
  * @brief   This file contains all the function prototypes
  *           for the gpio_driver.c file
  *        
  ******************************************************************************
  * @attention
  *
  * GPIO driver for the TTK4900 Master project of Kristian Blom, spring
  * semester of 2024. The driver handles GPIO interrupts from the end
  * switch and twist joint optical sensors. The functions are called
  * from the HAL_GPIO_EXTI_Callback function declared in stm32fxx_hal_gpio.h
  *
  ******************************************************************************
  */


//External library includes
#include "stdint.h"
//CubeMX generated includes
#include "gpio.h"
#include "can_driver.h"
#include "joint_controller.h"
#include "state_machine.h"

//TTK4900 library includes
#include "unit_config.h"


/// @brief Handler function for the rail end switch
void gpio_end_switch_handler();

/// @brief Handler function for the twist joint end switch
void gpio_twist_switch_handler();

#endif //GPIO_DRIVER_H