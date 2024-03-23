#include "motor_driver.h"


#if ACTIVE_UNIT == TORSO
static motor_control_descriptor motor0 =
  {
    .motorId = 0,
    .voltageLimit = 35,
    .voltagePctCap = 70,
    .motorPolarity = -1,
    .motorTimer = TIM15,
    .encoderTimer = TIM3,
    .resolution = 99,
    .encoderInitCount = 0,
    .encoderTotalInit = 0,
    .encoderTotalSetpoint = 0,
    .encoderTotalCount = 0,
    .encoderPreviousCount = 0,
    .isMoving = 0,
    .motorName = "rail"
  };
static motor_control_descriptor motor1 =
  {
    .motorId = 1,
    .voltageLimit = 50,
    .voltagePctCap = 50,
    .motorPolarity = -1,
    .motorTimer = TIM1,
    .encoderTimer = TIM8,
    .resolution = 19022,
    .encoderInitCount = 0,
    .encoderTotalInit = 0,
    .encoderTotalSetpoint = 0,
    .encoderTotalCount = 0,
    .encoderPreviousCount = 0,
    .isMoving = 0,
    .motorName = "shoulder"
  };
#endif
#if ACTIVE_UNIT == SHOULDER
static motor_control_descriptor motor0 =
  {
    .motorId = 2,
    .voltageLimit = 20,
    .voltagePctCap = 20,
    .motorPolarity = 1,
    .motorTimer = TIM15,
    .encoderTimer = TIM3,
    .resolution = 17700,
    .encoderInitCount = 0,
    .encoderTotalInit = 0,
    .encoderTotalSetpoint = 0,
    .encoderTotalCount = 0,
    .encoderPreviousCount = 0,
    .isMoving = 0,
    .motorName = "wrist"
  };
static motor_control_descriptor motor1 =
  {
    .motorId = 3,
    .voltageLimit = 20,
    .voltagePctCap = 20,
    .motorPolarity = 1,
    .motorTimer = TIM1,
    .encoderTimer = TIM8,
    .resolution = 36097,
    .encoderInitCount = 0,
    .encoderTotalInit = 0,
    .encoderTotalSetpoint = 0,
    .encoderTotalCount = 0,
    .encoderPreviousCount = 0,
    .isMoving = 0,
    .motorName = "elbow"
  };
#endif
#if ACTIVE_UNIT == HAND
static motor_control_descriptor motor0 =
  {
    .motorId = 4,
    .voltageLimit = 12,
    .voltagePctCap = 20,
    .motorPolarity = 1,
    .motorTimer = TIM15,
    .encoderTimer = TIM3,
    .resolution = 460,
    .encoderInitCount = 0,
    .encoderTotalInit = 0,
    .encoderTotalSetpoint = 0,
    .encoderTotalCount = 0,
    .encoderPreviousCount = 0,
    .isMoving = 0,
    .motorName = "pinch"
  };
static motor_control_descriptor motor1 =
  {
    .motorId = 5,
    .voltageLimit = 12,
    .voltagePctCap = 20,
    .motorPolarity = -1,
    .motorTimer = TIM1,
    .encoderTimer = TIM8,
    .resolution = 8881,
    .encoderInitCount = 0,
    .encoderTotalInit = 0,
    .encoderTotalSetpoint = 0,
    .encoderTotalCount = 0,
    .encoderPreviousCount = 0,
    .isMoving = 0,
    .motorName = "twist"
  };
#endif

void motor_interface_controller_init(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    motor_driver_init(&motor0);
  }
  else if(motorSelect == 1)
  {
    motor_driver_init(&motor1);
  }
}

void motor_interface_update_power(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    motor_driver_update_power(&motor0);
  }
  else if(motorSelect == 1)
  {
    motor_driver_update_power(&motor1);
  }
}
void motor_interface_set_power(uint8_t motorSelect, uint8_t direction, double power)
{
  if(motorSelect == 0)
  {
    motor_driver_set_power(&motor0, direction, power);
  }
  else if(motorSelect == 1)
  {
    motor_driver_set_power(&motor1, direction, power);
  }
}
int32_t motor_interface_get_setpoint(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    return (motor_driver_get_setpoint(&motor0));
  }
  else if(motorSelect == 1)
  {
    return (motor_driver_get_setpoint(&motor1));
  }
}

void motor_interface_set_setpoint(uint8_t motorSelect, int32_t setpoint)
{
  if(motorSelect == 0)
  {
    motor_driver_set_setpoint(&motor0, setpoint);
  }
  else if(motorSelect == 1)
  {
    motor_driver_set_setpoint(&motor1, setpoint);

#if HW_INTERFACE == UART_INTERFACE && GLOBAL_DEBUG    
    char* debug[64];
    sprintf(debug, "Setpoint interface: %i\n\r", setpoint);
    uart_send_string(debug);
#endif    

  }
}

int32_t motor_interface_get_total_count(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    return motor_driver_get_total_cnt(&motor0);
  }
  else if(motorSelect == 1)
  {
    return motor_driver_get_total_cnt(&motor1);

// #if HW_INTERFACE == UART_INTERFACE && GLOBAL_DEBUG    
//     char* debug[64];
//     sprintf(debug, "Tot cnt interface: %i\n\r", setpoint);
//     uart_send_string(debug);
// #endif    

  }
}

void motor_interface_update_tot_cnt(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    motor_driver_update_tot_cnt(&motor0);
  }
  else if(motorSelect == 1)
  {
    motor_driver_update_tot_cnt(&motor1);

// #if HW_INTERFACE == UART_INTERFACE && GLOBAL_DEBUG    
//     char* debug[64];
//     sprintf(debug, "Tot cnt interface: %i\n\r", setpoint);
//     uart_send_string(debug);
// #endif    

  }  
}


uint16_t motor_interface_get_encoder_count(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    return motor_driver_get_encoder_cnt(&motor0);
  }
  else if(motorSelect == 1)
  {
    return motor_driver_get_encoder_cnt(&motor1);

// #if HW_INTERFACE == UART_INTERFACE && GLOBAL_DEBUG    
//     char* debug[64];
//     sprintf(debug, "Setpoint interface: %i\n\r", setpoint);
//     uart_send_string(debug);
// #endif    

  }
}

uint8_t motor_interface_get_id(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    return motor_driver_get_id(&motor0);
  }
  else if(motorSelect == 1)
  {
    return motor_driver_get_id(&motor1);
  }
}

int32_t motor_interface_get_resolution(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    return motor_driver_get_resolution(&motor0);
  }
  else if(motorSelect == 1)
  {
    return motor_driver_get_resolution(&motor1);
  }
}

void motor_interface_delta_setpoint(uint8_t motorSelect, int32_t delta)
{
  if(motorSelect == 0)
  {
    motor_driver_delta_setpoint(&motor0, delta);
  }
  else if(motorSelect == 1)
  {
    motor_driver_delta_setpoint(&motor1, delta);
  }  
}

uint8_t motor_interface_get_moving(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    return motor_driver_get_moving(&motor0);
  }
  else if(motorSelect == 1)
  {
    return motor_driver_get_moving(&motor1);
  }
}


void motor_driver_update_power(motor_control_descriptor* motor)
{
  //int32_t error = motor->encoderTotalSetpoint - (int32_t)(motor->encoderTimer->CNT);
  int32_t error = motor->encoderTotalSetpoint - motor_driver_get_total_cnt(motor);
  double power = (double)error*0.01;
  if(power < 0)
  {
    motor_driver_set_power(motor, 0, fabs(power));
  }
  else
  {
    motor_driver_set_power(motor, 1, power);
  }
}

void motor_driver_set_power(motor_control_descriptor* motor, uint8_t direction, double power)
{

  double mPower = power > (double)(motor->voltagePctCap) ? (double)(motor1.voltagePctCap) : power;
  if(direction)
  {
    motor_driver_go_forward(mPower, motor->motorTimer, motor->motorPolarity);
  }
  else
  {
    motor_driver_go_backward(mPower, motor->motorTimer, motor->motorPolarity);
  }
  
  if(power < 0.5)
  {
    motor->isMoving = 0;
  }
  else
  {
    motor->isMoving = 1;
  }

}

void motor_driver_set_setpoint(motor_control_descriptor* motor, int32_t setpoint)
{
  motor->encoderTotalSetpoint = setpoint;

#if HW_INTERFACE == UART_INTERFACE && GLOBAL_DEBUG
  char* debug[64];
  sprintf(debug, "Setpoint driver: %i\n\r", motor->encoderTotalSetpoint);
  uart_send_string(debug);
#endif
}

void motor_driver_delta_setpoint(motor_control_descriptor* motor, int32_t delta)
{
  motor->encoderTotalSetpoint += delta;
}

int32_t motor_driver_get_setpoint(motor_control_descriptor* motor)
{
  return (motor->encoderTotalSetpoint);
}

int32_t motor_driver_get_total_cnt(motor_control_descriptor* motor)
{
  return motor->encoderTotalCount;
}

uint16_t motor_driver_get_encoder_cnt(motor_control_descriptor* motor)
{
  return (motor->encoderTimer)->CNT;
}

uint8_t motor_driver_get_id(motor_control_descriptor* motor)
{
  return motor->motorId;
}

int32_t motor_driver_get_resolution(motor_control_descriptor* motor)
{
  return motor->resolution;
}

uint8_t motor_driver_get_moving(motor_control_descriptor* motor)
{
  return motor->isMoving;
}

void motor_driver_init(motor_control_descriptor* motor)
{
  (motor->encoderTimer)->CNT = motor->encoderInitCount;
  //motor_driver_calc_safe_vlt(motor);
}

void motor_driver_set_pwm_dc(uint32_t* timerCounter, double pct)
{
  int timerTicks = (pct / 100) * PWM_CTR_PRD;
  *timerCounter = timerTicks;
}

void motor_driver_update_tot_cnt(motor_control_descriptor* motor)
{
  //static int32_t previousCount;
  uint16_t currentEncCount = motor_driver_get_encoder_cnt(motor);
  int32_t difference = (int32_t)currentEncCount - motor->encoderPreviousCount;
  if(difference < -1000)
  {
    difference = currentEncCount;
  }
  if(difference > 1000)
  {
    difference = 65535-currentEncCount;
  }
  motor->encoderTotalCount += difference;
  motor->encoderPreviousCount = (int32_t)currentEncCount;

  if(abs(difference) > 100)
  {
    motor->isMoving = 1;
  }
  else
  {
    motor->isMoving = 0;
  }

}

void motor_driver_go_forward(double pct, TIM_TypeDef* mtr, int8_t polarity)
{
  // char* debugbuf[200];
  // sprintf(debugbuf, "double: %i\n\r", (int)polarity);
  // uart_send_string(debugbuf);
  if(polarity == 1)
  {
    if(mtr == MTR1)
    {
      motor_driver_set_pwm_dc(&(mtr->CCR1), 100);
      motor_driver_set_pwm_dc(&(mtr->CCR2), 100-pct);
    }
    else if(mtr == MTR2)
    {
      motor_driver_set_pwm_dc(&(mtr->CCR3), 100);
      motor_driver_set_pwm_dc(&(mtr->CCR2), 100-pct);
    }
  }
  else if(polarity == -1)
  {
    if(mtr == MTR1)
    {
      motor_driver_set_pwm_dc(&(mtr->CCR1), 100-pct);
      motor_driver_set_pwm_dc(&(mtr->CCR2), 100);
    }
    else if(mtr == MTR2)
    {
      motor_driver_set_pwm_dc(&(mtr->CCR3), 100-pct);
      motor_driver_set_pwm_dc(&(mtr->CCR2), 100);
    }
  }
}

void motor_driver_go_backward(double pct, TIM_TypeDef* mtr, int8_t polarity)
{
  if(polarity == 1)
  {
    if(mtr == MTR1)
    {
      motor_driver_set_pwm_dc(&(mtr->CCR1), 100-pct);
      motor_driver_set_pwm_dc(&(mtr->CCR2), 100);
    }
    else if(mtr == MTR2)
    {
      motor_driver_set_pwm_dc(&(mtr->CCR3), 100-pct);
      motor_driver_set_pwm_dc(&(mtr->CCR2), 100);
    }
  }
  else if (polarity == -1)
  {
    if(mtr == MTR1)
    {
      motor_driver_set_pwm_dc(&(mtr->CCR1), 100);
      motor_driver_set_pwm_dc(&(mtr->CCR2), 100-pct);
    }
    else if(mtr == MTR2)
    {
      motor_driver_set_pwm_dc(&(mtr->CCR3), 100);
      motor_driver_set_pwm_dc(&(mtr->CCR2), 100-pct);
    }    
  }
}

//Calculates the highest acceptable voltage input
//based on the global VOLTAGE_IN and the information
//in the relevant struct.
//WARNING: Voltage limit is based on the motor's datasheet,
//DO NOT EXCEED
void motor_driver_calc_safe_vlt(motor_control_descriptor* motor)
{
  double voltageIn = (double)VOLTAGE_IN;
  double rawVoltageCap = (double)(motor->voltageLimit)/voltageIn;
  double voltageCap = rawVoltageCap > 1 ? 100 : (rawVoltageCap*100);
  motor->voltagePctCap = (uint8_t)voltageCap;

}