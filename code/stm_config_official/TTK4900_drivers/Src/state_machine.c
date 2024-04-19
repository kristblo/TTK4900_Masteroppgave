#include "state_machine.h"


static global_state globalState = GS_IDLE;
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
  //Set locally
  globalState = (global_state)inState;
  
  //Transmit on CAN
  // uint32_t outId = (1 << CAN_GBL_CMD_OFFSET) | GBL_ST_SET;
  // uint8_t outData[8];
  // memcpy(&outData, &inState, 1);
  // can_interface_send_msg(outData, outId, 8, 0);
}

void state_interface_broadcast_global_state()
{
  uint8_t state = (uint8_t)globalState;
  uint32_t outId = (1 << CAN_GBL_CMD_OFFSET) | GBL_ST_SET;
  uint8_t outData[8];
  memcpy(&outData, &state, 1);
  can_interface_send_msg(outData, outId, 8, 0);  

}

uint8_t state_interface_get_calibration_state()
{
  return (uint8_t)calibrationState;
}

void state_interface_set_calibration_state(uint8_t inState)
{
  //Set locally
  calibrationState = (calibration_state)inState;

  //Transmit on CAN
  // uint32_t outId = (1 << CAN_GBL_CMD_OFFSET) | GBL_ST_SET;
  // uint8_t outData[8];
  // uint8_t outGlobal = (uint8_t)GS_CALIBRATING;
  // memcpy(&outData[0], &outGlobal, 1);
  // memcpy(&outData[1], &inState, 1);
  // can_interface_send_msg(outData, outId, 8, 0);
}

void state_interface_broadcast_calibration_state()
{
  uint8_t cState = (uint8_t)calibrationState;
  uint32_t outId = (1 << CAN_GBL_CMD_OFFSET) | GBL_ST_SET;
  uint8_t outData[8];
  uint8_t gState = (uint8_t)GS_CALIBRATING;
  memcpy(&outData[0], &gState, 1);
  memcpy(&outData[1], &cState, 1);
  can_interface_send_msg(outData, outId, 8, 0);  

}



void state_calibrate_rail()
{
  // motor_interface_zero(0);
  // motor_interface_set_total_count(0, 0);

  controller_interface_set_power(0, 20);
  while(state_interface_get_es_flag() == 0)
  {

  }
  controller_interface_set_power(0, 0);
  controller_interface_set_position(0, 0);
  motor_interface_zero(0);
  motor_interface_set_total_count(0, 0);

  //Move the rail safely away from the switch
  controller_interface_set_power(0, -10);
  HAL_Delay(1000);
  motor_interface_update_tot_cnt(0);
  controller_interface_update_position(0);
  float pos = controller_interface_get_position(0);
  controller_interface_set_setpoint(0, pos);
  end_switch_flag = 0;
}
void state_calibrate_shoulder()
{
  if((controller_interface_get_upd_ctrl() == 1))
  {
    //controller_interface_update_position(1);

    if(controller_interface_acc_get_newY(0) == 1)
    {
      int16_t rawAcc = controller_interface_acc_getY(0);
      float middle = ((float)(rawAcc-2300)/16384); //Acc descriptor should have these values
      float radians = asinf(middle);

      float position = radians;
      controller_interface_set_position(1, position);
      controller_interface_acc_clear_newY(0);
      controller_interface_update_error(1);
      controller_interface_update_power(1);
    }
    controller_interface_clear_upd_ctrl();
  }

}


void state_calibrate_elbow()
{
  controller_interface_set_power(1, -10);

  motor_interface_update_tot_cnt(1);
  HAL_Delay(20);
  motor_interface_update_tot_cnt(1);

  while(motor_interface_get_delta(1) < 0)
  {
    HAL_Delay(5);//MCU is just too fast
    motor_interface_update_tot_cnt(1);

    //Elbow calibration also affects  wrist pos
    motor_interface_update_tot_cnt(0);
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
  //motor_interface_zero(1);
  motor_interface_set_total_count(1, 0);
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
  //Stops at -5 deg -> horizontal at +185deg -> ~57kClicks
  while(motor_interface_get_total_count(0) > -55600)
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
  state_interface_clear_tw_flag();
  
  motor_interface_zero(1);
  motor_interface_set_total_count(1, 0);
  motor_interface_update_tot_cnt(1);
  
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
  motor_interface_set_total_count(1, 0);
  controller_interface_set_setpoint(1, 1.5708);
}

void state_calibrate_pinch()
{
  controller_interface_set_power(0, -10);
  motor_interface_update_tot_cnt(0);
  HAL_Delay(200);
  motor_interface_update_tot_cnt(0);

  // motor_interface_set_total_count(0,0);
  // motor_interface_zero(0);
  
  //At -2V, the encoder gets about
  //-5 clicks per ms. The pinch motor moves until
  //the joint physically stops.
  while(motor_interface_get_delta(0) < 0)
  {
    HAL_Delay(1);//MCU is just too fast
    motor_interface_update_tot_cnt(0);
  }
  controller_interface_set_power(0, 0);
  motor_interface_set_total_count(0, 0);

}

