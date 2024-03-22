#ifndef ACCELEROMETER_DRIVER_H
#define ACCELEROMETER_DRIVER_H

#include "uart_driver.h"
#include "unit_config.h"
#include "i2c.h"

#include "stdint.h"
#include "string.h"


typedef struct
{
  I2C_HandleTypeDef* i2cHandle;
  uint16_t readAddr;
  uint16_t writeAddr;
  uint8_t xAccAddr;
  uint8_t yAccAddr;
  uint8_t zAccAddr;
  uint8_t xRotAddr;
  uint8_t yRotAddr;
  uint8_t zRotAddr;
  int16_t yCalibOffset;
  uint16_t gResolution; //Min/max measurable acceleration
  uint16_t rotResolution; //Degrees per second
} accelerometer_descriptor;

//Interface read bytes
uint8_t accl_interface_read_byte(uint8_t regAddr);

//Interface read register
uint16_t accl_interface_read_register(uint8_t regAddr);

//Interface read n bytes
void accl_interface_read_bytes(uint8_t startAddr, uint8_t numBytes, uint8_t* rxBuffer);

//Interface get acc
int16_t accl_interface_get_x_acc();
int16_t accl_interface_get_y_acc();
int16_t accl_interface_get_z_acc();

//Interface get rot
int16_t accl_interface_get_x_rot();
int16_t accl_interface_get_y_rot();
int16_t accl_interface_get_z_rot();

//Interface set byte
void accl_interface_set_byte(uint8_t regAddr, uint8_t data);

//Read a single byte address
uint8_t accl_driver_read_byte(accelerometer_descriptor* accelerometer, uint8_t regAddr);

//Read a 2 byte register address and concatenate results
uint16_t accl_driver_read_register(accelerometer_descriptor* accelerometer, uint8_t regAddr);

//Read n bytes
void accl_driver_read_bytes(accelerometer_descriptor* accelerometer, uint8_t startAddr, uint8_t numBytes, uint8_t* rxBuffer);

//Write single byte address
void accl_driver_set_byte(accelerometer_descriptor* accelerometer, uint8_t regAddr, uint8_t data);

//Write a 2 byte address
void accl_driver_set_register(accelerometer_descriptor* accelerometer);

//Write n bytes starting from address
void acc_driver_set_bytes(accelerometer_descriptor* accelerometer);

//Read X axis acceleration
int16_t accl_driver_get_x_acc(accelerometer_descriptor* accelerometer);

//Read Y axis acceleration
int16_t accl_driver_get_y_acc(accelerometer_descriptor* accelerometer);

//Read Z axis acceleration
int16_t accl_driver_get_z_acc(accelerometer_descriptor* accelerometer);

//Read X axis angular rate
int16_t accl_driver_get_x_rot(accelerometer_descriptor* accelerometer);

//Read Y axis angular rate
int16_t accl_driver_get_y_rot(accelerometer_descriptor* accelerometer);

//Read Z axis angular rate
int16_t accl_driver_get_z_rot(accelerometer_descriptor* accelerometer);

#endif //ACCELEROMETER_DRIVER_H