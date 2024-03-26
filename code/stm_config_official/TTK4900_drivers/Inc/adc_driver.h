#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H
/**
  ******************************************************************************
  * @file    adc_driver.h
  * @brief   This file contains all the function prototypes and struct
  *           definitions for the adc_driver.c file
  *        
  ******************************************************************************
  * @attention
  *
  * ADC driver for the TTK4900 Master project of Kristian Blom, spring
  * semester of 2024. This driver is tailored for use with the current sense
  * pin of the DRV8251A motor driver
  *
  ******************************************************************************
  */

//External library includes
#include "stdint.h"
#include "math.h"

//CubeMX generated includes
#include "adc.h"

//TTK4900 library includes
#include "unit_config.h"
#include "uart_driver.h"


/// @brief Database with key information for the motor current sensing ADCs
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


//////////////////
//Public functions
//////////////////

/// @brief Module external interface function to read the most recently calculated current
/// @param sensorSelect Select from one of two motor current sensor ADCs
/// @return Latest calculated current measurement
double adc_interface_get_current(uint8_t sensorSelect);


/// @brief Module external interface function to trigger a current calculation
/// @param sensorSelect Select from of two motor current sensor ADCs
void adc_interface_update_current(uint8_t sensorSelect);


///////////////////
//Private functions
///////////////////

/// @brief Calculates current in Ampere based on an ADC raw value
/// @param sensor Pointer to the relevant sensor struct
/// @return Ampere
double adc_driver_calculate_current(current_measurement_descriptor* sensor);


/// @brief Trigger a reading of the relevant ADC, insert into sensor struct
/// @param sensor Pointer to the relevant sensor struct
void adc_driver_update_reading(current_measurement_descriptor* sensor);


/// @brief Calculate lastest current measurement, insert into struct
/// @param sensor Pointer to the relevant sensor struct
void adc_driver_update_measurement(current_measurement_descriptor* sensor);
#endif //ADC_DRIVER_H