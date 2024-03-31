#include "gpio_driver.h"

#if GLOBAL_DEBUG
  #include "uart_driver.h"
#endif

//Originally declared in stm32fxx_hal_gpio.h
HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{
  if(GPIO_pin == END_SW_Pin)
  {
#if GLOBAL_DEBUG
    uart_send_string("End switch triggered\n\r");
#endif
    gpio_end_switch_handler();
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
  //Turn the rail motor off asap

  state_interface_set_es_flag();
  // motor_interface_set_total_count(0, 0);
  // controller_interface_set_setpoint(0, 0);
  // controller_interface_set_error(0, 0);
  // controller_interface_set_power(0, 0);
  
}

void gpio_twist_switch_handler()
{
  char* data = "HELLO TW";
  //can_interface_send_msg(data);
}