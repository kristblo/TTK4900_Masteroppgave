#ifndef ACCELEROMETER_DRIVER_H
#define ACCELEROMETER_DRIVER_H
/**
  ******************************************************************************
  * @file    accelerometer_driver.h
  * @brief   This file contains all the function prototypes and struct
  *           definitions for the accelerometer_driver.c file
  *        
  ******************************************************************************
  * @attention
  *
  * IMU driver for the TTK4900 Master project of Kristian Blom, spring
  * semester of 2024. The driver specifies a struct of relevant register
  * addresses from the LSM6DSM IMU as well as functions using the I2C
  * peripheral to read these addresses.
  *
  ******************************************************************************
  */


#include "uart_driver.h"
#include "unit_config.h"
#include "i2c.h"

#include "stdint.h"
#include "string.h"

/// @brief Key information about the IMUs
typedef struct
{
  /// @brief Pointer to the I2C bus peripheral
  I2C_HandleTypeDef* i2cHandle;
  
  /// @brief Read address of the IMU on the I2C bus
  uint16_t readAddr;
  
  /// @brief Write address of the IMU on the I2C bus
  uint16_t writeAddr;
  
  /// @brief Start address of the IMU's X axis accelerometer register 
  uint8_t xAccAddr;
  
  /// @brief Start address of the IMU's Y axis accelerometer register
  uint8_t yAccAddr;
  
  /// @brief Start address of the IMU's Z axis accelerometer register
  uint8_t zAccAddr;
  
  /// @brief Start address of the IMU's X axis rotation rate register
  uint8_t xRotAddr;
  
  /// @brief Start address of the IMU's Y axis rotation rate register
  uint8_t yRotAddr;
  
  /// @brief Start address of the IMU's Z axis rotation rate register
  uint8_t zRotAddr;
} imu_descriptor;


//////////////////
//Public functions
//////////////////


/// @brief Module external interface function to read a byte register
/// @param regAddr Address to read
/// @return Value of the register
uint8_t accl_interface_read_byte(uint8_t regAddr);


/// @brief Module external interface function to read a two-byte register
/// @param regAddr Start address of the read
/// @return Value of the register
uint16_t accl_interface_read_register(uint8_t regAddr);


/// @brief Module external interface function to read the IMU's X axis acceleration
/// @return Raw acceleration value as determined by the IMU's configured acceleration resolution
int16_t accl_interface_get_x_acc();


/// @brief Module external interface function to read the IMU's Y axis acceleration
/// @return Raw acceleration value as determined by the IMU's configured acceleration resolution
int16_t accl_interface_get_y_acc();


/// @brief Module external interface function to read the IMU's Z axis acceleration
/// @return Raw acceleration value as determined by the IMU's configured acceleration resolution
int16_t accl_interface_get_z_acc();


/// @brief Module external interface function to read the IMU's X axis rotation
/// @return Raw rotation value as determined by the IMU's configured rotation resolution
int16_t accl_interface_get_x_rot();


/// @brief Module external interface function to read the IMU's Y axis rotation
/// @return Raw rotation value as determined by the IMU's configured rotation resolution
int16_t accl_interface_get_y_rot();


/// @brief Module external interface function to read the IMU's Z axis rotation
/// @return Raw rotation value as determined by the IMU's configured rotation resolution
int16_t accl_interface_get_z_rot();


/// @brief Module external interface function to write a byte to an IMU register
/// @param regAddr Address to write
/// @param data Data to write
void accl_interface_set_byte(uint8_t regAddr, uint8_t data);



///////////////////
//Private functions
///////////////////


/// @brief Read a single byte register from the IMU
/// @param imu Pointer to the relevant IMU struct
/// @param regAddr Address to read
/// @return Value of the register
uint8_t accl_driver_read_byte(imu_descriptor* imu, uint8_t regAddr);


/// @brief Read a two byte register from the IMU
/// @param imu Pointer to the relevant IMU struct
/// @param regAddr Address to start read
/// @return Concatenated values of the two registers (left shift + bitwOR)
uint16_t accl_driver_read_register(imu_descriptor* imu, uint8_t regAddr);

/// @brief Write single byte register of the IMU
/// @param imu Pointer to the relevant IMU struct
/// @param regAddr Address to write
/// @param data Data to write
void accl_driver_set_byte(imu_descriptor* imu, uint8_t regAddr, uint8_t data);


/// @brief Read X axis acceleration register
/// @param imu Pointer to the relevant IMU struct
/// @return Acceleration raw value
int16_t accl_driver_get_x_acc(imu_descriptor* imu);


/// @brief Read Y axis acceleration register
/// @param imu Pointer to the relevant IMU struct
/// @return Acceleration raw value
int16_t accl_driver_get_y_acc(imu_descriptor* imu);


/// @brief Read X axis acceleration register
/// @param imu Pointer to the relevant IMU struct
/// @return Acceleration raw value
int16_t accl_driver_get_z_acc(imu_descriptor* imu);


/// @brief Read X axis rotation register
/// @param imu Pointer to the relevant IMU struct
/// @return Rotation raw value
int16_t accl_driver_get_x_rot(imu_descriptor* imu);


/// @brief Read Y axis rotation register
/// @param imu Pointer to the relevant IMU struct
/// @return Rotation raw value
int16_t accl_driver_get_y_rot(imu_descriptor* imu);


/// @brief Read Z axis rotation register
/// @param imu Pointer to the relevant IMU struct
/// @return Rotation raw value
int16_t accl_driver_get_z_rot(imu_descriptor* imu);

#endif //ACCELEROMETER_DRIVER_H