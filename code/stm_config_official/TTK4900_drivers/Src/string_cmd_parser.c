#include "string_cmd_parser.h"

#if GLOBAL_DEBUG
  #include "uart_driver.h"
#endif

void string_cmd_processor_base(char* inputString, uint8_t stringLength)
{

  //Make a copy of input for safekeeping
  char stringToProcess[64];
  strcpy(stringToProcess, inputString);

  //Separate input tokens
  char tokenDelimiter[] = " ";
  char* p_character = strtok(stringToProcess, tokenDelimiter);
  char tokensToProcess[64][64]; //TODO: Consider choosing a less arbitrary value
  int numTokens = 0;
  while(p_character != NULL)
  {
    strcpy(tokensToProcess[numTokens], p_character);
    p_character = strtok(NULL, tokenDelimiter);
    ++numTokens;
  }
  //Process individual command strings
  for(int i = 0; i < NUM_STRING_COMMANDS; i++)
  {
    if((int)strcmp(tokensToProcess[0], (stringCmdList[i].cmdString)) == 0)
    {
      stringCmdList[i].cmdFuncPointer(tokensToProcess);
    }
  }

  //Flush token array
  memset(tokensToProcess, '\0', sizeof(tokensToProcess));
}

void string_cmd_processor_wrp(string_cmd_processor_args* input)
{
  uint8_t stringLength_out = input->stringLength ? input->stringLength : 64;
  char* string_out = input->inputString ? input->inputString : "ERROR EMPTY STRING";

  string_cmd_processor_base(string_out, stringLength_out);
}

void string_cmd_rail(char (*inputTokens)[64])
{
#if HW_INTERFACE == UART_INTERFACE
  uart_send_string("Got rail command\n\r");
#endif

#if ACTIVE_UNIT == TORSO
  string_cmd_category_local_motor(0, inputTokens);
#endif


}

void string_cmd_shoulder(char (*inputTokens)[64])
{
#if HW_INTERFACE == UART_INTERFACE && GLOBAL_DEBUG
  uart_send_string("Got shoulder command\n\r");
#endif

#if ACTIVE_UNIT == TORSO
  string_cmd_category_local_motor(1, inputTokens);
#endif

}

void string_cmd_elbow(char (*inputTokens)[64])
{
#if HW_INTERFACE == UART_INTERFACE
  uart_send_string("Got elbow command\n\r");
#endif

#if ACTIVE_UNIT == SHOULDER
  string_cmd_category_local_motor(1, inputTokens);
#else
  string_cmd_category_remote_motor(2, inputTokens);
#endif

}


void string_cmd_wrist(char (*inputTokens)[64])
{
#if HW_INTERFACE == UART_INTERFACE
  uart_send_string("Got wrist command\n\r");
#endif

#if ACTIVE_UNIT == SHOULDER
  string_cmd_category_local_motor(0, inputTokens);
#else
  string_cmd_category_remote_motor(3, inputTokens);  
#endif
}


void string_cmd_twist(char (*inputTokens)[64])
{
#if HW_INTERFACE == UART_INTERFACE
  uart_send_string("Got twist command\n\r");
#endif

#if ACTIVE_UNIT == HAND
  string_cmd_category_local_motor(1, inputTokens);
#else
  string_cmd_category_remote_motor(4, inputTokens);
#endif
}


void string_cmd_pinch(char (*inputTokens)[64])
{
#if HW_INTERFACE == UART_INTERFACE
  uart_send_string("Got pinch command\n\r");
#endif

#if ACTIVE_UNIT == HAND
  string_cmd_category_local_motor(0, inputTokens);
#else
  string_cmd_category_remote_motor(5, inputTokens);
#endif
}


void string_cmd_can(char (*inputTokens)[64])
{
#if HW_INTERFACE == UART_INTERFACE
  uart_send_string("Got can command\n\r");
#endif
}
void string_cmd_stop(char (*inputTokens)[64])
{
  //TODO: STOP non-local motors
  motor_interface_set_power(0, 1, 0);
  motor_interface_set_power(1, 1, 0);
#if HW_INTERFACE == UART_INTERFACE
  uart_send_string("Got STOP command\n\r");
#endif
}

void string_cmd_category_local_motor(uint8_t motor, char (*inputTokens)[64])
{
  if((int)strcmp(inputTokens[1], "fwd") == 0)
  {
    float power = (float)atof(inputTokens[2]);
    motor_interface_set_power(motor, 1, power);
  }
  else if((int)strcmp(inputTokens[1], "bwd") == 0)
  {
    float power = (float)atof(inputTokens[2]);
    motor_interface_set_power(motor, 0, power);
  }
  else if((int)strcmp(inputTokens[1], "stp") == 0)
  {
    int32_t setpoint = (int32_t)atoi(inputTokens[2]);
    motor_interface_set_setpoint(motor, setpoint);
  }
  else
  {  
#if HW_INTERFACE == UART_INTERFACE && GLOBAL_DEBUG
    uart_send_string("ERROR no valid direction\n\r");
#endif    
  }
}
void string_cmd_category_remote_motor(uint8_t motor, char (*inputTokens)[64])
{
  
  uint8_t data[8];
  data[0] = 'M'; //M for motor
  data[1] = motor << 4;

  if((int)strcmp(inputTokens[1], "fwd") == 0)
  {
    data[1] |= 0x1;
    int32_t power = (int32_t)atoi(inputTokens[2]);
    uint8_t powerToBytes[4];
    memcpy(powerToBytes, &power, 4);
    for(int i = 0; i < 4; i++)
    {
      data[i+2] = powerToBytes[i];
    }
    can_interface_send_msg(data);

    // int32_t decoded_int;
    // memcpy(&decoded_int, &data[2], 4);
    // float decoded_f = (float)decoded_int;
    // char* debug[64];
    // sprintf(debug, "decoded: %X\n\r", ((data[1] & 0x0F)));
    // uart_send_string(debug);
    
  }
  else if((int)strcmp(inputTokens[1], "bwd") == 0)
  {
    data[1] |= 0x2;
    int32_t power = (int32_t)atoi(inputTokens[2]);
    uint8_t powerToBytes[4];
    memcpy(powerToBytes, &power, 4);
    for(int i = 0; i < 4; i++)
    {
      data[i+2] = powerToBytes[i];
    }
    can_interface_send_msg(data);
    
  }
  else if((int)strcmp(inputTokens[1], "stp") == 0)
  {
    data[1] |= 0x3;
    int32_t setpoint = (int32_t)atoi(inputTokens[2]);
    uint8_t setpointToBytes[4];
    memcpy(setpointToBytes, &setpoint, 4);
    for(int i = 0; i < 4; i++)
    {
      data[i+2] = setpointToBytes[i];
    }
    can_interface_send_msg(data);
  }
  else
  {  
#if HW_INTERFACE == UART_INTERFACE && GLOBAL_DEBUG
    uart_send_string("ERROR no valid direction\n\r");
#endif    
  }
  
}