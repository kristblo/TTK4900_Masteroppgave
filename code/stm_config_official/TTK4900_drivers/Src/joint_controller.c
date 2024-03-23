#include "joint_controller.h"

#if ACTIVE_UNIT == TORSO
static joint_controller_descriptor joint0 =
{
  .hasAccelerometer = 0,
  .posSetpoint = 0,
  .posCurrent = 0,
  .posError = 0,
  .isMoving = 0,
  .motorNum = 0,
  .Kp = 0.7,
  .KpTi = 0,//0.0001,
  .Kd = 0,
  .intError = 0,
  .jointName = "rail"
};

static joint_controller_descriptor joint1 =
{
  .hasAccelerometer = 1,
  .posSetpoint = 0,
  .posCurrent = 0,
  .posError = 0,
  .isMoving = 0,
  .motorNum = 1,
  .Kp = 40,
  .KpTi = 0, //0.001,
  .Kd = 0,
  .intError = 0,  
  .jointName = "shoulder"
};

#elif ACTIVE_UNIT == SHOULDER
static joint_controller_descriptor joint0 =
{
  .hasAccelerometer = 0,
  .posSetpoint = 0,
  .posCurrent = 0,
  .posError = 0,
  .isMoving = 0,
  .motorNum = 0,
  .Kp = 70,
  .KpTi = 0, //0.0001,
  .Kd = 0,
  .intError = 0,
  .jointName = "wrist"
};

static joint_controller_descriptor joint1 =
{
  .hasAccelerometer = 0,
  .posSetpoint = 0,
  .posCurrent = 0,
  .posError = 0,
  .isMoving = 0,
  .motorNum = 1,
  .Kp = 40,
  .KpTi = 0,// 0.0003,
  .Kd = 0,
  .intError = 0,  
  .jointName = "elbow"
};

#elif ACTIVE_UNIT == HAND

static joint_controller_descriptor joint0 =
{
  .hasAccelerometer = 0,
  .posSetpoint = 0,
  .posCurrent = 0,
  .posError = 0,
  .isMoving = 0,
  .motorNum = 0,
  .Kp = 20,
  .KpTi = 0,//0.0001,
  .Kd = 0,
  .intError = 0,  
  .jointName = "pinch"
};

static joint_controller_descriptor joint1 =
{
  .hasAccelerometer = 0,
  .posSetpoint = 0,
  .posCurrent = 0,
  .posError = 0,
  .isMoving = 0,
  .motorNum = 1,
  .Kp = 20,
  .KpTi = 0,//0.0003,
  .Kd = 0,
  .intError = 0,  
  .jointName = "twist"
};
#endif

joint_controller_descriptor* joints[2] =
{
  &joint0,
  &joint1
};


float controller_interface_get_setpoint(uint8_t controllerSelect)
{
  return joint_controller_get_setpoint(joints[controllerSelect]);
}


void controller_interface_set_setpoint(uint8_t controllerSelect, float setPoint)
{
  joint_controller_set_setpoint(joints[controllerSelect], setPoint);
}


float controller_interface_get_position(uint8_t controllerSelect)
{
  return joint_controller_get_position(joints[controllerSelect]);
}


void controller_interface_set_position(uint8_t controllerSelect, float position)

{
  joint_controller_set_setpoint(joints[controllerSelect], position);
}


float controller_interface_get_error(uint8_t controllerSelect)
{
  return joint_controller_get_error(joints[controllerSelect]);
}

void controller_interface_update_error(uint8_t controllerSelect)
{
  joint_controller_update_error(joints[controllerSelect]);
}

void controller_interface_set_error(uint8_t controllerSelect, float error)
{
  joint_controller_set_error(joints[controllerSelect], error);
}


uint8_t controller_interface_get_moving(uint8_t controllerSelect)
{
  return joint_controller_get_moving(joints[controllerSelect]);
}


void controller_interface_set_moving(uint8_t controllerSelect)
{
  joint_controller_set_moving(joints[controllerSelect]);
}


void controller_interface_clear_moving(uint8_t controllerSelect)
{
  joint_controller_clear_moving(joints[controllerSelect]);
}

void controller_interface_set_power(uint8_t controllerSelect, float power)
{
  joint_controller_set_power(joints[controllerSelect], power);
}

void controller_interface_update_power(uint8_t controllerSelect)
{
  joint_controller_update_power(joints[controllerSelect]);
}


float controller_interface_clicks_to_pos(uint8_t controllerSelect)
{
  return joint_controller_clicks_to_pos(joints[controllerSelect]);
}


void controller_interface_adjust_enc_sp(uint8_t controllerSelect)
{
  joint_controller_adjust_enc_sp(joints[controllerSelect]);
}

float joint_controller_get_setpoint(joint_controller_descriptor* joint)
{
  return (joint->posSetpoint);
}


void joint_controller_set_setpoint(joint_controller_descriptor* joint, float setpoint)
{
  (joint->posSetpoint) = setpoint;
}


float joint_controller_get_position(joint_controller_descriptor* joint)
{
  return (joint->posCurrent);
}


void joint_controller_set_position(joint_controller_descriptor* joint, float position)
{
  joint->posCurrent = position;
}

float joint_controller_get_error(joint_controller_descriptor* joint)
{
  return joint->posError;
}

void joint_controller_update_error(joint_controller_descriptor* joint)
{
  float setpoint = joint_controller_get_setpoint(joint);
  int32_t totalClicks = motor_interface_get_total_count(joint->motorNum);
  int32_t resolution = motor_interface_get_resolution(joint->motorNum);
  float error = setpoint - ((float)totalClicks/(float)resolution);
  joint_controller_set_error(joint, error);

}

void joint_controller_set_error(joint_controller_descriptor* joint, float error)
{
  joint->posError = error;
}


uint8_t joint_controller_get_moving(joint_controller_descriptor* joint)
{
  return joint->isMoving;
}


void joint_controller_set_moving(joint_controller_descriptor* joint)
{
  joint->isMoving = 1;
}


void joint_controller_clear_moving(joint_controller_descriptor* joint)
{
  joint->isMoving = 0;
}

void joint_controller_set_power(joint_controller_descriptor* joint, float power)
{
  if(power < 0)
  {
    motor_interface_set_power(joint->motorNum, 0, fabs(power));
  }
  else
  {
    motor_interface_set_power(joint->motorNum, 1, power);
  }
}


void joint_controller_update_power(joint_controller_descriptor* joint)
{
  float error = joint_controller_get_error(joint);

  joint->intError += ((joint->KpTi)*error); //Bit afraid of overflow, therefore using KpTi here

  float power = (joint->Kp)*error + (joint->intError);
  if(power < 0)
  {
    motor_interface_set_power(joint->motorNum, 0, fabs(power));
  }
  else
  {
    motor_interface_set_power(joint->motorNum, 1, power);
  }
}


float joint_controller_clicks_to_pos(joint_controller_descriptor* joint)
{
  int32_t resolution = motor_interface_get_resolution(joint->motorNum);
  int32_t errorAsClicks = (int32_t)((joint->posError)*(float)resolution);
  return errorAsClicks;
}





void joint_controller_request_acceleration(joint_controller_descriptor* joint);


float joint_controller_acceleration_to_angle(joint_controller_descriptor* joint);
