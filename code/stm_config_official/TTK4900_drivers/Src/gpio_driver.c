#include "gpio_driver.h"
#include "can_driver.h"

#if GLOBAL_DEBUG
  #include "uart_driver.h"
#endif

//Originally declared in stm32fxx_hal_gpio.h
HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{
  if(GPIO_pin == END_SW_Pin)
  {
    gpio_end_switch_handler();
#if GLOBAL_DEBUG
    uart_send_string("End switch triggered\n\r");
#endif
  }
  if(GPIO_pin == OPT_SW1_Pin)
  {
    gpio_twist_switch_handler();
#if GLOBAL_DEBUG
    uart_send_string("Twist switch triggered\n\r");
#endif

  }
}

void gpio_end_switch_handler()
{
  //char* data = "HELLO ES";
  
  //Equivalent of "twist stp e -15000"
  uint8_t data[8] = {0x65, 0x68, 0xC5, 0xFF, 0xFF, 0,0,0};
  uint8_t id = 0xA5;
  can_interface_queue_tx(MOTOR_POS_SP, data, id);
  
}

void gpio_twist_switch_handler()
{
  char* data = "HELLO TW";
  //can_interface_send_msg(data);
}