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
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got rail command\n\r");
#endif

#if ACTIVE_UNIT == TORSO
  string_cmd_category_local_motor(0, inputTokens);
#endif


}

void string_cmd_shoulder(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got shoulder command\n\r");
#endif

#if ACTIVE_UNIT == TORSO
  string_cmd_category_local_motor(1, inputTokens);
#else
  string_cmd_category_remote_motor(1, inputTokens);
#endif

}

void string_cmd_elbow(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got elbow command\n\r");
#endif

#if ACTIVE_UNIT == SHOULDER
  string_cmd_category_local_motor(1, inputTokens);
#else
  string_cmd_category_remote_motor(3, inputTokens);
#endif

}


void string_cmd_wrist(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got wrist command\n\r");
#endif

#if ACTIVE_UNIT == SHOULDER
  string_cmd_category_local_motor(0, inputTokens);
#else
  string_cmd_category_remote_motor(2, inputTokens);  
#endif
}


void string_cmd_twist(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got twist command\n\r");
#endif

#if ACTIVE_UNIT == HAND
  string_cmd_category_local_motor(1, inputTokens);
#else
  string_cmd_category_remote_motor(5, inputTokens);
#endif
}


void string_cmd_pinch(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got pinch command\n\r");
#endif

#if ACTIVE_UNIT == HAND
  string_cmd_category_local_motor(0, inputTokens);
#else
  string_cmd_category_remote_motor(4, inputTokens);
#endif
}


void string_cmd_can(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got can command\n\r");
#endif
}

void string_cmd_acc1(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got acc1 command\n\r");
#endif  

  //uint8_t cmd = (uint8_t)atoi(inputTokens[1]);
  uint8_t cmd = (uint8_t)strtol(inputTokens[1], '\0', 16);

#if ACTIVE_UNIT == TORSO
  uint8_t accSelect = 0;
  uint8_t data[8];
  data[0] = cmd;
  int32_t id = (accSelect << CAN_ACC_CMD_OFFSET) | ACC_REG_REQ;
  can_interface_queue_tx(ACC_REG_REQ, data, id);

#elif ACTIVE_UNIT == SHOULDER
  int16_t regVal = accl_interface_read_register(cmd);
  char* debug[64];
  sprintf(debug, "Regval: %i\n\r", regVal);
  uart_send_string(debug);

#elif ACTIVE_UNIT == HAND
  int16_t regVal = accl_interface_read_register(cmd);
  char* debug[64];
  sprintf(debug, "Regval: %i\n\r", regVal);
  uart_send_string(debug);

#endif

}

void string_cmd_stop(char (*inputTokens)[64])
{
  //TODO: STOP non-local motors
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, 0);
  state_interface_set_global_state(GS_IDLE);
  state_interface_broadcast_global_state();
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got STOP command\n\r");
#endif
}

void string_cmd_rly(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Got relay command\n\r");

  uint8_t relay = (uint8_t)atoi(inputTokens[1]);
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, relay);
#endif  
}


void string_cmd_home(char (*inputTokens)[64])
{
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
  uart_send_string("Starting calibration\n\r");
#endif
  state_interface_set_global_state(GS_CALIBRATING);
  state_interface_set_calibration_state(CS_RAIL);
}


void string_cmd_state(char (*inputTokens)[64])
{
  if((int)strcmp(inputTokens[1], "operate") == 0)
  {
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("Global state: operating\n\r");
#endif
    state_interface_set_global_state(GS_OPERATING);
    state_interface_broadcast_global_state();
  }
  if((int)strcmp(inputTokens[1], "calibrate") == 0)
  {
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("Global state: calibrating\n\r");
#endif
    state_interface_set_global_state(GS_CALIBRATING);
    state_interface_set_calibration_state(CS_RAIL);
  }
  if((int)strcmp(inputTokens[1], "idle") == 0)
  {
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("Global state: idle\n\r");
#endif
    state_interface_set_global_state(GS_IDLE);
    state_interface_broadcast_global_state();
  }
}


void string_cmd_category_local_motor(uint8_t motor, char (*inputTokens)[64])
{
  
  uint8_t mode = inputTokens[2][0];
  if((int)strcmp(inputTokens[1], "fwd") == 0)
  {
      float power = (float)atof(inputTokens[3]);
      motor_interface_set_power(motor, 1, power); 
  }
  else if((int)strcmp(inputTokens[1], "bwd") == 0)
  {
    //TODO: deprecate bwd
    float power = (float)atof(inputTokens[3]);
    motor_interface_set_power(motor, 0, power);
  }
  else if((int)strcmp(inputTokens[1], "stp") == 0)
  {
    if(mode == 'e')
    {      
      int32_t setpoint = (int32_t)atoi(inputTokens[3]);
      int32_t resolution = motor_interface_get_resolution(motor);
      float setpointRads = (float)(setpoint/resolution);
      controller_interface_set_setpoint(motor, setpointRads);

    }
    else if(mode == 'r')
    {
      float setpoint = (float)atof(inputTokens[3]);
      controller_interface_set_setpoint(motor, setpoint);
    }
    else
    {
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)      
      uart_send_string("ERROR no valid setpoint mode: e, r\n\r");
#endif      
    }
  }
  else
  {  
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("ERROR no valid direction\n\r");
    uart_send_string("Motor cmd format: <motor> <cmd> <args>\n\r");
#endif    
  }
}
void string_cmd_category_remote_motor(uint8_t motor, char (*inputTokens)[64])
{
  
  uint8_t data[8];

  if((int)strcmp(inputTokens[1], "fwd") == 0)
  {
    int32_t power = (int32_t)atoi(inputTokens[2]);
    memcpy(data, &power, 4);
    uint32_t id = (motor << CAN_MOTOR_CMD_OFFSET) | MOTOR_VLT_SP;
    
    can_interface_queue_tx(MOTOR_VLT_SP, data, id);    
  }
  else if((int)strcmp(inputTokens[1], "stp") == 0)
  {
    uint8_t mode = inputTokens[2][0];
    float setpoint = (float)atof(inputTokens[3]);
    memcpy(data, &mode, 1);
    memcpy(&data[1], &setpoint, 4);
    uint32_t id = (motor << CAN_MOTOR_CMD_OFFSET) | JOINT_POS_SP;
    
    can_interface_queue_tx(JOINT_POS_SP, data, id);
  }
  else
  {  
#if GLOBAL_DEBUG && (SW_INTERFACE == CMD_MODE_TERMINAL)
    uart_send_string("ERROR no valid direction\n\r");
#endif    
  }
  
}