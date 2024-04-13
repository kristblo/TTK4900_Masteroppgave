#include "gpio_driver.h"

#if GLOBAL_DEBUG
  #include "uart_driver.h"
#endif

//Originally declared in stm32fxx_hal_gpio.h
HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{
  if(GPIO_pin == END_SW_Pin)
  {
    gpio_end_switch_handler();
#if (HW_INTERFACE == UART_INTERFACE)  && (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("End switch triggered\n\r");
#endif
  }
  if(GPIO_pin == OPT_SW1_Pin)
  {
    gpio_twist_switch_handler();
#if (HW_INTERFACE == UART_INTERFACE)  && (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("Twist switch triggered\n\r");
#endif

  }
}

void gpio_end_switch_handler()
{
  state_interface_set_es_flag(); 
}

void gpio_twist_switch_handler()
{
  state_interface_set_tw_flag();
}