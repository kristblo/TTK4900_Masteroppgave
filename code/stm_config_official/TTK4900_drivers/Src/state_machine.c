#include "state_machine.h"


static global_state globalState = idle;
static calibration_state calibrationState = num_cStates;

static uint8_t end_switch_flag = 0;

void state_interface_set_es_flag()
{
  end_switch_flag = 1;
}
uint8_t state_interface_get_es_flag()
{
  return end_switch_flag;
}

void state_interface_clear_es_flag()
{
  end_switch_flag = 0;
}

uint8_t state_interface_get_global_state()
{
  return (uint8_t)globalState;
}

void state_interface_set_global_state(uint8_t inState)
{
  globalState = (global_state)inState;
}

void state_calibrate_rail()
{
  controller_interface_set_power(0, 25);
  while(state_interface_get_es_flag() == 0)
  {

  }
  controller_interface_set_power(0, 0);
  controller_interface_set_position(0, 0);
  motor_interface_set_total_count(0, 0);
  end_switch_flag = 0;
}
void state_calibrate_shoulder();
void state_calibrate_elbow();
void state_calibrate_wrist();
void state_calibrate_twist();
void state_calibrate_pinch();

