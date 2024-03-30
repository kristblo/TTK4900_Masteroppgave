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
  .Kp = 30,
  .KpTi = 0.050,
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

/// @brief Ensures polling of accelerometers is kept below a safe maximum frequency
static uint8_t accelerometer_poll_safe = 0;

/// @brief Ensures polling of motors is kept below a safe maximum frequency
static uint8_t motor_poll_safe = 0;

#if ACTIVE_UNIT == TORSO
//Torso uses one accelerometer: Shoulder control
accelerometer_inData accelerometers[1];
#elif ACTIVE_UNIT == SHOULDER
//Shoulder uses two accelerometers: Elbow and wrist control
accelerometer_inData* accelerometers[2];
#elif ACTIVE_UNIT == HAND
//Hand uses no accelerometers, but adding this is easier than
//selectively compiling the dependent functions
accelerometer_inData* accelerometers[0];

#endif


//Public functions

//----------------------
//Joint control handlers
//----------------------
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

void controller_interface_request_acc_axis(uint8_t controllerSelect, uint8_t accSelect, char axis)
{
  joint_controller_request_acc_axis(joints[controllerSelect], accSelect, axis);
}

//----------------------
//Accelerometer data handlers
//----------------------

int16_t controller_interface_acc_getX(uint8_t accSelect)
{
  return controller_acc_getX(&accelerometers[accSelect]);
}
int16_t controller_interface_acc_getY(uint8_t accSelect)
{
  return controller_acc_getY(&accelerometers[accSelect]);
}
int16_t controller_interface_acc_getZ(uint8_t accSelect)
{
  return controller_acc_getX(&accelerometers[accSelect]);
}


void controller_interface_acc_setX(uint8_t accSelect, int16_t accVal)
{
  controller_acc_setX(&accelerometers[accSelect], accVal);
}
void controller_interface_acc_setY(uint8_t accSelect, int16_t accVal)
{
  controller_acc_setY(&accelerometers[accSelect], accVal);
}
void controller_interface_acc_setZ(uint8_t accSelect, int16_t accVal)
{
  controller_acc_setZ(&accelerometers[accSelect], accVal);
}

int16_t controller_interface_rot_getX(uint8_t accSelect)
{
  return controller_rot_getX(&accelerometers[accSelect]);
}
int16_t controller_interface_rot_getY(uint8_t accSelect)
{
  return controller_rot_getY(&accelerometers[accSelect]);
}
int16_t controller_interface_rot_getZ(uint8_t accSelect)
{
  return controller_rot_getZ(&accelerometers[accSelect]);
}

void controller_interface_rot_setX(uint8_t accSelect, int16_t rotVal)
{
  controller_rot_setX(&accelerometers[accSelect], rotVal);
}
void controller_interface_rot_setY(uint8_t accSelect, int16_t rotVal)
{
  controller_rot_setY(&accelerometers[accSelect], rotVal);
}
void controller_interface_rot_setZ(uint8_t accSelect, int16_t rotVal)
{
  controller_rot_setZ(&accelerometers[accSelect], rotVal);
}


uint8_t controller_interface_acc_get_newX(uint8_t accSelect)
{
  return controller_acc_get_newX(&accelerometers[accSelect]);
}
uint8_t controller_interface_acc_get_newY(uint8_t accSelect)
{
  return controller_acc_get_newY(&accelerometers[accSelect]);
}
uint8_t controller_interface_acc_get_newZ(uint8_t accSelect)
{
  return controller_acc_get_newZ(&accelerometers[accSelect]);
}

void controller_interface_acc_set_newX(uint8_t accSelect)
{
  controller_acc_set_newX(&accelerometers[accSelect]);
}
void controller_interface_acc_set_newY(uint8_t accSelect)
{
  controller_acc_set_newY(&accelerometers[accSelect]);
}
void controller_interface_acc_set_newZ(uint8_t accSelect)
{
  controller_acc_set_newZ(&accelerometers[accSelect]);
}

void controller_interface_acc_clear_newX(uint8_t accSelect)
{
  controller_acc_clear_newX(&accelerometers[accSelect]);
}
void controller_interface_acc_clear_newY(uint8_t accSelect)
{
  controller_acc_clear_newY(&accelerometers[accSelect]);
}
void controller_interface_acc_clear_newZ(uint8_t accSelect)
{
  controller_acc_clear_newZ(&accelerometers[accSelect]);
}

uint8_t controller_interface_rot_get_newX(uint8_t accSelect)
{
  return controller_rot_get_newX(&accelerometers[accSelect]);
}
uint8_t controller_interface_rot_get_newY(uint8_t accSelect)
{
  return controller_rot_get_newY(&accelerometers[accSelect]);
}
uint8_t controller_interface_rot_get_newZ(uint8_t accSelect)
{
  return controller_rot_get_newZ(&accelerometers[accSelect]);
}

void controller_interface_rot_set_newX(uint8_t accSelect)
{
  controller_rot_set_newX(&accelerometers[accSelect]);
}
void controller_interface_rot_set_newY(uint8_t accSelect)
{
  controller_rot_set_newY(&accelerometers[accSelect]);
}
void controller_interface_rot_set_newZ(uint8_t accSelect)
{
  controller_rot_set_newZ(&accelerometers[accSelect]);
}

void controller_interface_rot_clear_newX(uint8_t accSelect)
{
  controller_rot_clear_newX(&accelerometers[accSelect]);
}
void controller_interface_rot_clear_newY(uint8_t accSelect)
{
  controller_rot_clear_newY(&accelerometers[accSelect]);
}
void controller_interface_rot_clear_newZ(uint8_t accSelect)
{
  controller_rot_clear_newZ(&accelerometers[accSelect]);
}



uint8_t controller_interface_get_acc_poll()
{
  return accelerometer_poll_safe;
}
uint8_t controller_interface_get_mtr_poll()
{
  return motor_poll_safe;
}
void controller_interface_set_acc_poll()
{
  accelerometer_poll_safe = 1;
}
void controller_interface_set_mtr_poll()
{
  motor_poll_safe = 1;
}
void controller_interface_clear_acc_poll()
{
  accelerometer_poll_safe = 0;
}
void controller_interface_clear_mtr_poll()
{
  motor_poll_safe = 0;
}


//Private functions

//----------------------
//Joint control handlers
//----------------------
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
  
  float error;
  if(joint->hasAccelerometer) //TODO: should check if data is stale; incorporate newX/Y/X flag
  {
    //Only works for shoulder joint obvs
    int16_t rawAcc = controller_acc_getY(&accelerometers[0]);
    float middle = ((float)(rawAcc-2300)/16384); //Acc descriptor should have these values
    float radians = asinf(middle);    

    //error = setpoint - ((float)totalClicks/(float)resolution)
    error = setpoint - (-1*radians); //Shoulder acc has "wrong" polarity

    // int32_t output = (int32_t)(error*100);
    // char* debug[64];
    // sprintf(debug, "rawacc: %i\n\r", output);
    // uart_send_string(debug);

  }
  else
  {
    error = setpoint - ((float)totalClicks/(float)resolution);
  }
  
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



void joint_controller_request_acc_axis(joint_controller_descriptor* joint, uint8_t accSelect, char axis)
{
  uint8_t data[8];
  if(axis == 'x')
  {
    int32_t id = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_X_REQ;
    can_interface_queue_tx(ACC_X_REQ, data, id);
  }
  if(axis == 'y')
  {
    int32_t id = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_Y_REQ;
    can_interface_queue_tx(ACC_Y_REQ, data, id);
  }
  if(axis == 'z')
  {
    int32_t id = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_Z_REQ;
    can_interface_queue_tx(ACC_Z_REQ, data, id);
  }
}


float joint_controller_acceleration_to_angle(joint_controller_descriptor* joint)
{
  //TODO: Implement. Currently handled by update_error. Should account for
  //individual offsets in each accelerometer.
}


//----------------------
//Accelerometer data handlers
//----------------------
int16_t controller_acc_getX(accelerometer_inData* accSelect)
{
  return accSelect->xAcc;
}
int16_t controller_acc_getY(accelerometer_inData* accSelect)
{
  return accSelect->yAcc;
}
int16_t controller_acc_getZ(accelerometer_inData* accSelect)
{
  return accSelect->zAcc;
}

void controller_acc_setX(accelerometer_inData* accSelect, int16_t accVal)
{
  accSelect->xAcc = accVal;
}
void controller_acc_setY(accelerometer_inData* accSelect, int16_t accVal)
{
  accSelect->yAcc = accVal;
}
void controller_acc_setZ(accelerometer_inData* accSelect, int16_t accVal)
{
  accSelect->zAcc = accVal;
}

int16_t controller_rot_getX(accelerometer_inData* accSelect)
{
  return accSelect->xRot;
}
int16_t controller_rot_getY(accelerometer_inData* accSelect)
{
  return accSelect->yRot;
}
int16_t controller_rot_getZ(accelerometer_inData* accSelect)
{
  return accSelect->zRot;
}

void controller_rot_setX(accelerometer_inData* accSelect, int16_t rotVal)
{
  accSelect->xRot = rotVal;
}
void controller_rot_setY(accelerometer_inData* accSelect, int16_t rotVal)
{
  accSelect->yRot = rotVal;
}
void controller_rot_setZ(accelerometer_inData* accSelect, int16_t rotVal)
{
  accSelect->zRot = rotVal;
}


uint8_t controller_acc_get_newX(accelerometer_inData* accSelect)
{
  return accSelect->newXAcc;
}
uint8_t controller_acc_get_newY(accelerometer_inData* accSelect)
{
  return accSelect->newYAcc;
}
uint8_t controller_acc_get_newZ(accelerometer_inData* accSelect)
{
  return accSelect->newZAcc;
}

void controller_acc_set_newX(accelerometer_inData* accSelect)
{
  accSelect->newXAcc = 1;
}
void controller_acc_set_newY(accelerometer_inData* accSelect)
{
  accSelect->newYAcc = 1;
}
void controller_acc_set_newZ(accelerometer_inData* accSelect)
{
  accSelect->newZAcc = 1;
}

void controller_acc_clear_newX(accelerometer_inData* accSelect)
{
  accSelect->newXAcc = 0;
}
void controller_acc_clear_newY(accelerometer_inData* accSelect)
{
  accSelect->newYAcc = 0;
}
void controller_acc_clear_newZ(accelerometer_inData* accSelect)
{
  accSelect->newZAcc = 0;
}

uint8_t controller_rot_get_newX(accelerometer_inData* accSelect)
{
  return accSelect->newXRot;
}
uint8_t controller_rot_get_newY(accelerometer_inData* accSelect)
{
  return accSelect->newYRot;
}
uint8_t controller_rot_get_newZ(accelerometer_inData* accSelect)
{
  return accSelect->newZRot;
}

void controller_rot_set_newX(accelerometer_inData* accSelect)
{
  accSelect->newXRot = 1;
}
void controller_rot_set_newY(accelerometer_inData* accSelect)
{
  accSelect->newYRot = 1;
}
void controller_rot_set_newZ(accelerometer_inData* accSelect)
{
  accSelect->newZRot = 1;
}

void controller_rot_clear_newX(accelerometer_inData* accSelect)
{
  accSelect->newXRot = 0;
}
void controller_rot_clear_newY(accelerometer_inData* accSelect)
{
  accSelect->newYRot = 0;
}
void controller_rot_clear_newZ(accelerometer_inData* accSelect)
{
  accSelect->newZRot = 0;
}


HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  controller_interface_set_acc_poll();
  controller_interface_set_mtr_poll();
}