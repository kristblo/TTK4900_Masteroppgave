#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H
//External library includes
#include "stdint.h"
//CubeMX generated includes
#include "gpio.h"
#include "can_driver.h"

//TTK4900 library includes
#include "unit_config.h"

//------FILE BEGIN------
void gpio_end_switch_handler();
void gpio_twist_switch_handler();

#endif //GPIO_DRIVER_H