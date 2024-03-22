#ifndef STRING_CMD_PARSER_H
#define STRING_CMD_PARSER_H
//External library includes
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "stdint.h"

//CubeMX generated includes

//TTK4900 library includes
#include "unit_config.h"
#include "motor_driver.h"
#include "can_driver.h"

//------FILE BEGIN------

//Generate default values for string processors
typedef struct
{
  char* inputString[64];
  uint8_t stringLength;

} string_cmd_processor_args;
void string_cmd_processor_wrp(string_cmd_processor_args* input);
void string_cmd_processor_base(char* inputString, uint8_t stringLength);
#define string_cmd_processor(...) string_cmd_processor_wrp((string_cmd_processor_args*){__VA_ARGS__})

void string_cmd_category_local_motor(uint8_t motor, char (*inputTokens)[64]);
void string_cmd_category_remote_motor(uint8_t motor, char (*inputTokens)[64]);
void string_cmd_category_adc();
void string_cmd_category_accelerometer();
void string_cmd_category_encoder();

//List of valid commands to process
#define NUM_STRING_COMMANDS 9
void string_cmd_rail(char (*inputTokens)[64]);
void string_cmd_shoulder(char (*inputTokens)[64]);
void string_cmd_elbow(char (*inputTokens)[64]);
void string_cmd_wrist(char (*inputTokens)[64]);
void string_cmd_twist(char (*inputTokens)[64]);
void string_cmd_pinch(char (*inputTokens)[64]);
void string_cmd_can(char (*inputTokens)[64]);
void string_cmd_stop(char (*inputTokens)[64]);
void string_cmd_acc1(char (*inputTokens)[64]);
typedef struct 
{
  char* cmdString;
  void (*cmdFuncPointer)();
} string_cmd_pair;

//Pairing of string commands and executive functions
static string_cmd_pair stringCmdList[NUM_STRING_COMMANDS] =
  {
  {"rail", string_cmd_rail},
  {"shoulder", string_cmd_shoulder},
  {"elbow", string_cmd_elbow},
  {"wrist", string_cmd_wrist},
  {"twist", string_cmd_twist},
  {"pinch", string_cmd_pinch},
  {"can", string_cmd_can},
  {"S", string_cmd_stop},
  {"acc1", string_cmd_acc1}
  };

#endif //STRING_CMD_PARSER_H