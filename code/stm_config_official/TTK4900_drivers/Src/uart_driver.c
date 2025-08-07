#include "uart_driver.h"
//#include "string_cmd_parser.h"

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

void uart_parse_hmi_input(char* input, 
                      uint8_t* buffer, 
                      uint8_t bufferLength, 
                      uint8_t* bufferPos)
{

  uint8_t flag_cr = (input[0] == '\r') ? (uint8_t)1 : (uint8_t)0;
  uint8_t flag_bs = (input[0] == '\b' || input[0] == 127) ? (uint8_t)1 : (uint8_t)0;
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
  else if(flag_bs)
  {
    *bufferPos = (--(*bufferPos))%bufferLength;
    buffer[*bufferPos] = (uint8_t)0;
  }
  else
  {
    buffer[*bufferPos] = input[0];
    *bufferPos = (++(*bufferPos))%bufferLength;
  }

}


void uart_parse_ros_input(char* input)
{
  char header[3];
  memcpy(&header[0], &input[0], 3);

  char* debug[64];
  
  //Breaks the strcmp if not present...
  for(int i = 0; i < 32; i++)
  {
    //sprintf(debug, "header: %X\n\r", input[i]);
    uart_send_string(debug);
  }

  if((int)strcmp(header, "num") == 0)
  //if(header[0] == 'n')
  {
    //uart_send_string("num\n\r");
    float pos_rail;
    memcpy(&pos_rail, &input[3], 4);
    // float pos_shoulder;
    // memcpy(&pos_rail, &input[7], 4);
    controller_interface_set_setpoint(0, pos_rail*1000); //incoming in meters
    //controller_interface_set_setpoint(1, pos_shoulder);
    
  }
  else if ((int)strcmp(header, "hom") == 0)
  //else if (header[0] == 'h');
  {
    uart_send_string("home\n\r");
    state_interface_set_global_state(GS_CALIBRATING);
    state_interface_set_calibration_state(CS_RAIL);    
  }
  // for(int i = 0; i < 32; i++)
  // {
  //   input[i] = '\0';
  // }
}


//TODO: Consider making volatile
static uint8_t bufferPosition = 0;
static uint8_t uartRxBuffer[1];
static uint8_t uartRxHoldingBuffer[64];
void uart_hmi_rx_handler()
{
  HAL_UART_Receive_IT(&huart5, uartRxBuffer, 1);
  uart_parse_hmi_input(uartRxBuffer, uartRxHoldingBuffer, 64, &bufferPosition);
  uart_send_string("\33[2K\r");
  HAL_UART_Transmit_IT(&huart5, uartRxHoldingBuffer, 64);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if SW_INTERFACE == CMD_MODE_TERMINAL
  uart_hmi_rx_handler();
#elif SW_INTERFACE == CMD_MODE_ROS
  uart_ros_rx_handler();
#endif
}


void uart_hmi_init()
{
  HAL_UART_Receive_IT(&huart5, uartRxBuffer, 1);
}


volatile uint8_t uartRosRxBuffer[32];
void uart_ros_rx_handler()
{
  //uart_send_string("ros msg rx handler\n\r");
  if(HAL_UART_Receive_IT(&huart5, uartRosRxBuffer, 32) != HAL_OK)
  {
    uart_send_string("uart not ok\n\r");
  }
  ros_interface_set_rxBuffer(&uartRosRxBuffer);
  ros_interface_set_newMsgFlag();

}
void uart_ros_init()
{
  HAL_UART_Receive_IT(&huart5, uartRosRxBuffer, 32);
}