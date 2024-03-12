#include "uart_driver.h"

void uart_send_string(char* str)
{
  int stringLength = 0;
  char* p_character = str;

  while(*p_character)
  {
    stringLength++;
    p_character++;
  }

  HAL_UART_Transmit(&huart5, str, stringLength, 10);
}
