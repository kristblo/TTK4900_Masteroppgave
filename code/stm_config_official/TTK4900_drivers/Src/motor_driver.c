#include "motor_driver.h"


#if ACTIVE_UNIT == TORSO
static motor_control_descriptor motor0 =
  {
    .motorId = 0,
    .voltageLimit = 35,
    .voltagePctCap = 30,
    .motorTimer = TIM15,
    .encoderTimer = TIM3,
    .encoderInitCount = 0,
    .encoderTotalInit = 0,
    .encoderTotalSetpoint = 0,
    .encoderTotalCount = 0,
    .encoderPreviousCount = 0,
    .motorName = "rail"
  };
static motor_control_descriptor motor1 =
  {
    .motorId = 1,
    .voltageLimit = 50,
    .voltagePctCap = 20,
    .motorTimer = TIM1,
    .encoderTimer = TIM8,
    .encoderInitCount = 0x7FFF,
    .encoderTotalInit = 0x7FFF,
    .encoderTotalSetpoint = 0x7FFF,
    .encoderTotalCount = 0,
    .encoderPreviousCount = 0,
    .motorName = "shoulder"
  };
#endif

void motor_interface_controller_init(uint8_t motorSelect)
{
  if(motorSelect == 0)
  {
    motor_driver_controller_init(&motor0);
  }
  else if(motorSelect == 1)
  {
    motor_driver_controller_init(&motor1);
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
    motor_driver_go_forward(mPower, motor->motorTimer);
  }
  else
  {
    motor_driver_go_backward(mPower, motor->motorTimer);
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

void motor_driver_controller_init(motor_control_descriptor* motor)
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

}

void motor_driver_go_forward(double pct, TIM_TypeDef* mtr)
{
  // char* debugbuf[200];
  // sprintf(debugbuf, "double: %i\n\r", (int)pct);
  // uart_send_string(debugbuf);
#if MTR_POL == 1
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

#elif MTR_POL == -1
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
#endif
}

void motor_driver_go_backward(double pct, TIM_TypeDef* mtr)
{
#if MTR_POL == 1
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
#elif MTR_POL == -1
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

#endif
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