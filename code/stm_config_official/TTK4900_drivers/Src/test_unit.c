#include "test_unit.h"

void updategbvar()
{
  HAL_Delay(1000);
  motor_interface_set_setpoint(1, 23456);
}