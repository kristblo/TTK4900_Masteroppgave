#include "adc_driver.h"

static current_measurement_descriptor motor_ipropi0 =
{
  .VrefA = 3.3,
  .Ripropi = 510,
  .Aipropi = 1575,
  .Nadc = 4096,
  .adc = &hadc2,
  .conversionConst = 0.001003005
};

static current_measurement_descriptor motor_ipropi1 =
{
  .VrefA = 3.3,
  .Ripropi = 510,
  .Aipropi = 1575,
  .Nadc = 4096,
  .adc = &hadc1,
  .conversionConst = 0.001003005
};

current_measurement_descriptor* sensors[2] =
{
  &motor_ipropi0,
  &motor_ipropi1
};


double adc_interface_get_current(uint8_t sensorSelect)
{
  return (sensors[sensorSelect])->lastMeasurement;
}

void adc_interface_update_current(uint8_t sensorSelect)
{
  adc_driver_update_reading(sensors[sensorSelect]);
  adc_driver_update_measurement(sensors[sensorSelect]);
}


void adc_driver_update_measurement(current_measurement_descriptor* sensor)
{
  sensor->lastMeasurement = adc_driver_calculate_current(sensor);
}


double adc_driver_calculate_current(current_measurement_descriptor* sensor)
{
  double current = ((double)(sensor->lastReading))*(sensor->conversionConst);

  return current;

}

void adc_driver_update_reading(current_measurement_descriptor* sensor)
{
  sensor->lastReading = HAL_ADC_GetValue(sensor->adc);
}


/// @brief Turns off the motor relay on overcurrent
/// @param hadc Any ADC
HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, 0);
}