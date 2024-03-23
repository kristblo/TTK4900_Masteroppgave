#include "accelerometer_driver.h"

static accelerometer_descriptor shoulder_accelerometer =
{
  .i2cHandle = &hi2c3,
  .readAddr = 0xD5,
  .writeAddr = 0xD4,
  .xRotAddr = 0x22,
  .yRotAddr = 0x24,
  .zRotAddr = 0x26,
  .xAccAddr = 0x28,
  .yAccAddr = 0x2A,
  .zAccAddr = 0x2C,
  .yCalibOffset = -2500,
  .gResolution = 2,
  .rotResolution = 250
};

int16_t accl_interface_get_x_rot()
{
  return accl_driver_get_x_rot(&shoulder_accelerometer);
}
int16_t accl_interface_get_y_rot()
{
  return accl_driver_get_y_rot(&shoulder_accelerometer);
}
int16_t accl_interface_get_z_rot()
{
  return accl_driver_get_z_rot(&shoulder_accelerometer);
}


uint8_t accl_interface_read_byte(uint8_t regAddr)
{
  return accl_driver_read_byte(&shoulder_accelerometer, regAddr);
}

uint16_t accl_interface_read_register(uint8_t regAddr)
{
  return accl_driver_read_register(&shoulder_accelerometer, regAddr);
}

void accl_interface_read_bytes(uint8_t startAddr, uint8_t numBytes, uint8_t* rxBuffer)
{
  return accl_driver_read_bytes(&shoulder_accelerometer, startAddr, numBytes, rxBuffer);
}

int16_t accl_interface_get_x_acc()
{
  return accl_driver_get_x_acc(&shoulder_accelerometer);
}
int16_t accl_interface_get_y_acc()
{
  return accl_driver_get_y_acc(&shoulder_accelerometer);
}
int16_t accl_interface_get_z_acc()
{
  return accl_driver_get_z_acc(&shoulder_accelerometer);
}
void accl_interface_set_byte(uint8_t regAddr, uint8_t data)
{
  accl_driver_set_byte(&shoulder_accelerometer, regAddr, data);
}

//Read a single byte address
uint8_t accl_driver_read_byte(accelerometer_descriptor* accelerometer, uint8_t regAddr)
{
  uint8_t txBuffer[1] = {regAddr};
  uint8_t rxBuffer[1];
  if(HAL_I2C_Master_Transmit(accelerometer->i2cHandle, 
                              accelerometer->readAddr,
                              txBuffer,
                              1,
                              100) != HAL_OK)
  {
#if GLOBAL_DEBUG
  uart_send_string("I2C transmit failed\n\r");
#endif  
  }

  if(HAL_I2C_Master_Receive(accelerometer->i2cHandle,
                            accelerometer->readAddr,
                            rxBuffer,
                            1,
                            100) != HAL_OK)
  {
#if GLOBAL_DEBUG
  uart_send_string("I2C receive failed\n\r");
#endif      
  }

  return rxBuffer[0];
}

void accl_driver_set_byte(accelerometer_descriptor* accelerometer, uint8_t regAddr, uint8_t data)
{
  uint8_t txBuffer[2] = {regAddr, data};
  if(HAL_I2C_Master_Transmit(accelerometer->i2cHandle,
                              accelerometer->writeAddr,
                              txBuffer,
                              2,
                              100) != HAL_OK)
  {
#if GLOBAL_DEBUG    
    uart_send_string("I2C write byte failed\n\r");
#endif

  }
}

//Read a 2 byte register address and concatenate results
uint16_t accl_driver_read_register(accelerometer_descriptor* accelerometer, uint8_t regAddr)
{
  uint8_t low = accl_driver_read_byte(accelerometer, regAddr);
  uint8_t high = accl_driver_read_byte(accelerometer, regAddr+1);
  uint16_t regVal = (uint16_t)((high << 8) | low);

  return regVal;
}

void accl_driver_read_bytes(accelerometer_descriptor* accelerometer, uint8_t startAddr, uint8_t numBytes, uint8_t* rxBuffer)
{
  uint8_t txBuffer = {startAddr};
  
  if(HAL_I2C_Master_Transmit(accelerometer->i2cHandle, 
                              accelerometer->readAddr,
                              txBuffer,
                              1,
                              100) != HAL_OK)
  {
#if GLOBAL_DEBUG
  uart_send_string("I2C multitransmit failed\n\r");
#endif  
  }

  if(HAL_I2C_Master_Receive(accelerometer->i2cHandle,
                            accelerometer->readAddr,
                            rxBuffer,
                            numBytes,
                            100) != HAL_OK)
  {
#if GLOBAL_DEBUG
  uart_send_string("I2C multireceive failed\n\r");
#endif      
  }
}

//Read X axis acceleration
int16_t accl_driver_get_x_acc(accelerometer_descriptor* accelerometer)
{
  return (int16_t)accl_driver_read_register(accelerometer, accelerometer->xAccAddr);
}

//Read Y axis acceleration
int16_t accl_driver_get_y_acc(accelerometer_descriptor* accelerometer)
{
  return (int16_t)accl_driver_read_register(accelerometer, accelerometer->yAccAddr);
}

//Read Z axis acceleration
int16_t accl_driver_get_z_acc(accelerometer_descriptor* accelerometer)
{
  return (int16_t)accl_driver_read_register(accelerometer, accelerometer->zAccAddr);
}

//Read X axis angular rate
int16_t accl_driver_get_x_rot(accelerometer_descriptor* accelerometer)
{
  return (int16_t)accl_driver_read_register(accelerometer, accelerometer->xRotAddr);
}

//Read Y axis angular rate
int16_t accl_driver_get_y_rot(accelerometer_descriptor* accelerometer)
{
  return (int16_t)accl_driver_read_register(accelerometer, accelerometer->yRotAddr);
}

//Read Z axis angular rate
int16_t accl_driver_get_z_rot(accelerometer_descriptor* accelerometer)
{
  return (int16_t)accl_driver_read_register(accelerometer, accelerometer->zRotAddr);
}