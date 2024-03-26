#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

//External library includes
#include "stdint.h"
#include "math.h"

//CubeMX generated includes
#include "adc.h"

//TTK4900 library includes
#include "unit_config.h"
#include "uart_driver.h"


typedef struct
{
  /// @brief Analog reference voltage, V
  double VrefA;
  
  /// @brief Current sense resistor, Ohm
  double Ripropi;
  
  /// @brief Current sense proportional current, uA/A
  double Aipropi;
  
  /// @brief ADC saturation point
  uint32_t Nadc;

  /// @brief ADC for current measurement
  ADC_HandleTypeDef* adc;

  /// @brief Constant number for conversion of ADC value to Ampere
  double conversionConst;

  /// @brief Latest ADC raw value
  uint32_t lastReading;

  /// @brief Latest calculated current 
  double lastMeasurement;
} current_measurement_descriptor;

double adc_interface_get_current(uint8_t sensorSelect);
void adc_interface_update_current(uint8_t sensorSelect);


double adc_driver_calculate_current(current_measurement_descriptor* sensor, uint32_t rawVal);
void adc_driver_update_reading(current_measurement_descriptor* sensor);
void adc_driver_update_measurement(current_measurement_descriptor* sensor);
#endif //ADC_DRIVER_H