#include "state_machine.h"


static global_state globalState = idle;
static calibration_state calibrationState = num_cStates;

static uint8_t end_switch_flag = 0;
static uint8_t twist_switch_flag = 0;

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


void state_interface_set_tw_flag()
{
  twist_switch_flag = 1;
}
uint8_t state_interface_get_tw_flag()
{
  return twist_switch_flag;
}
void state_interface_clear_tw_flag()
{
  twist_switch_flag = 0;
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
  motor_interface_zero(0);
  end_switch_flag = 0;
}
void state_calibrate_shoulder();


void state_calibrate_elbow()
{
  controller_interface_set_power(1, -10);

  motor_interface_update_tot_cnt(1);
  HAL_Delay(20);
  motor_interface_update_tot_cnt(1);

  // char* debug[64];
  // int32_t delta = motor_interface_get_delta(1);
  // sprintf(debug, "delta ex: %i\n\r", delta);
  // uart_send_string(debug);    


  while(motor_interface_get_delta(1) < 0)
  {
    HAL_Delay(5);//MCU is just too fast
    motor_interface_update_tot_cnt(1);

    char* debug[64];
    int32_t delta = motor_interface_get_delta(1);
    sprintf(debug, "delta: %i\n\r", delta);
    uart_send_string(debug);    
  }
  //motor_interface_zero(1);
  motor_interface_set_total_count(1, 0);
  motor_interface_update_tot_cnt(1);
  controller_interface_set_power(1, 0);
  controller_interface_set_position(1, 0);
  
  controller_interface_set_power(1, 15);
  //HAL_Delay(2);

  //Stops at 25 deg -> vertical at +165deg -> ~104kClicks
  while(motor_interface_get_total_count(1) < 103952)
  {
    motor_interface_update_tot_cnt(1);
  }
  controller_interface_set_power(1, 0);
  motor_interface_zero(1);
  controller_interface_set_position(1, 0);

}


void state_calibrate_wrist()
{
  //motor_interface_zero(0);
  motor_interface_set_total_count(0,0);
  HAL_Delay(20);
  controller_interface_set_power(0, 15);
  motor_interface_update_tot_cnt(0);

  HAL_Delay(20);
  motor_interface_update_tot_cnt(0);

  while(motor_interface_get_delta(0) > 0)
  {
    HAL_Delay(50);//MCU is just too fast
    motor_interface_update_tot_cnt(0);
  }
  controller_interface_set_power(0, 0);
  motor_interface_set_total_count(0,0);
  controller_interface_set_position(0, 0);
  controller_interface_set_power(0, -15);
  HAL_Delay(200);
  motor_interface_update_tot_cnt(0);
  
  //Stops at 45 deg -> horizontal at +135deg -> ~42kClicks
  while(motor_interface_get_total_count(0) > -42300)
  {
    motor_interface_update_tot_cnt(0);
  }
  controller_interface_set_power(0, 0);
  motor_interface_set_total_count(0,0);
  controller_interface_set_position(0, 0);

}

void state_calibrate_twist()
{
  controller_interface_set_power(1, 10);
  while(state_interface_get_tw_flag() == 0)
  {

  }
  twist_switch_flag = 0;
  
  motor_interface_zero(1);
  controller_interface_set_power(1, -10);

  //-10900 encoder counts corresponds to the angular placement
  //of the optical sensor s.t. the twist joint stops when the
  //pincher is horisontal
  while(motor_interface_get_total_count(1) > -10900)
  {
    //Total count is nominally handled by the main while loop,
    //but the calibration phase takes control of that.
    motor_interface_update_tot_cnt(1);
  }
  controller_interface_set_power(1, 0);
  motor_interface_zero(1);
  controller_interface_set_position(1, 0);
}

void state_calibrate_pinch()
{
  controller_interface_set_power(0, -10);
  motor_interface_update_tot_cnt(0);
  HAL_Delay(200);
  motor_interface_update_tot_cnt(0);

  
  //At -2V, the encoder gets about
  //-5 clicks per ms. The pinch motor moves until
  //the joint physically stops.
  while(motor_interface_get_delta(0) < 0)
  {
    HAL_Delay(1);//MCU is just too fast
    motor_interface_update_tot_cnt(0);
  }
  controller_interface_set_power(0, 0);
  motor_interface_zero(0);
  controller_interface_set_position(0, 0);

}

