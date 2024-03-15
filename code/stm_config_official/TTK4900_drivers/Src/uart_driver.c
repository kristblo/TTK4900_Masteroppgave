#include "uart_driver.h"
#include "string_cmd_parser.h"

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

void uart_parse_input(char* input, 
                      uint8_t* buffer, 
                      uint8_t bufferLength, 
                      uint8_t* bufferPos)
{

  uint8_t flag_cr = (input[0] == '\r') ? (uint8_t)1 : (uint8_t)0;
  if(flag_cr)
  {
    string_cmd_processor(buffer);
    uint8_t erasePos = 0;
    while(erasePos < bufferLength)
    {
      buffer[erasePos] = (uint8_t)0;
      erasePos++;
    }
    *bufferPos = 0;
    flag_cr = 0;
  }
  else
  {
    buffer[*bufferPos] = input[0];
    *bufferPos = (++(*bufferPos))%bufferLength;
  }

}

//TODO: Consider making volatile
static uint8_t bufferPosition = 0;
static uint8_t uartRxBuffer[1];
static uint8_t uartRxHoldingBuffer[64];
void uart_hmi_rx_handler()
{
  HAL_UART_Receive_IT(&huart5, uartRxBuffer, 1);
  uart_parse_input(uartRxBuffer, uartRxHoldingBuffer, 64, &bufferPosition);
  uart_send_string("\33[2K\r");
  HAL_UART_Transmit_IT(&huart5, uartRxHoldingBuffer, 64);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if SW_INTERFACE == CMD_MODE_TERMINAL
  uart_hmi_rx_handler();
#elif SW_INTERFACE == CMD_MODE_ROS
  //Do something else
#endif
}


void uart_hmi_init()
{
  HAL_UART_Receive_IT(&huart5, uartRxBuffer, 1);
}

void uart_ros_rx_handler()
{

}
void uart_ros_init()
{

}