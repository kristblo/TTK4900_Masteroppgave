#ifndef STRING_CMD_PARSER_H
#define STRING_CMD_PARSER_H

/**
  ******************************************************************************
  * @file    string_cmd_parser.h
  * @brief   This file contains all the function prototypes and struct
  *           definitions for the string_cmd_parser.c file
  *        
  ******************************************************************************
  * @attention
  *
  * Keyboard input string parser for the TTK4900 Master project of Kristian Blom, 
  * spring semester of 2024. The parser makes use of the joint controller, motor
  * driver, and the STM32's UART peripheral as well as C string libraries to enable 
  * debugging of the arm during development. It lets the user write specified
  * commands to a serial interface such as PuTTy to control joints directly,
  * as well as make sensor readouts on demand.
  * 
  * The string_cmd_pair list defines valid commands available to the user,
  * which may be expanded at will. When using the arm with a serial interface,
  * an input string beginning with one of the defined words will be handled by
  * the corresponding function. In the interest of standardisation, all
  * input strings limited to 64 characters.
  *
  ******************************************************************************
  */



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
#include "joint_controller.h"
#include "state_machine.h"

//------FILE BEGIN------

/// @brief Wrapper struct to enable a variable number of arguments to the string processor
typedef struct
{
  /// @brief String to be processed
  char* inputString[64];
  
  /// @brief Length of the string to be processed
  uint8_t stringLength;

} string_cmd_processor_args;


/// @brief Wrapper function to enable a variable number of arguments to the string processor
/// @param input Pointer to arguments
void string_cmd_processor_wrp(string_cmd_processor_args* input);


/// @brief Starting point for string processing, splits and incoming string into tokens delimited by " "
/// @param inputString The string to be processed
/// @param stringLength Length of the string to be processed
void string_cmd_processor_base(char* inputString, uint8_t stringLength);
#define string_cmd_processor(...) string_cmd_processor_wrp((string_cmd_processor_args*){__VA_ARGS__})


/// @brief Handles commands to a motor/joint controller local to the STM32 connected to the serial interface
/// @param motor One of two motors, 0 or 1
/// @param inputTokens Tokens received from the string processor
void string_cmd_category_local_motor(uint8_t motor, char (*inputTokens)[64]);


/// @brief Handles commands to a motor/joint controller accessible via CAN from the STM32 connected to the serial interface
/// @param motor One of two motors, 0 or 1
/// @param inputTokens Tokens received from the string processor
void string_cmd_category_remote_motor(uint8_t motor, char (*inputTokens)[64]);


/// @brief Handles commands to an ADC (NOTE: Not implemented)
void string_cmd_category_adc();

/// @brief Handles commands to an accelerometer (NOTE: Not implemented)
void string_cmd_category_accelerometer();

//Number of valid commands to process
#define NUM_STRING_COMMANDS 0xB

/// @brief Called when "rail" token is registered; concerns the rail linear joint
/// @param inputTokens Arguments to parse
void string_cmd_rail(char (*inputTokens)[64]);

/// @brief Called when "shoulder" token is registered; concerns the shoulder joint
/// @param inputTokens Arguments to parse
void string_cmd_shoulder(char (*inputTokens)[64]);


/// @brief Called when "elbow" token is registered; concerns the elbow joint
/// @param inputTokens Arguments to parse
void string_cmd_elbow(char (*inputTokens)[64]);

/// @brief Called when "wrist" token is registered; concerns the wrist joint
/// @param inputTokens Arguments to parse
void string_cmd_wrist(char (*inputTokens)[64]);


/// @brief Called when "twist" token is registered; concerns the twist joint
/// @param inputTokens Arguments to parse
void string_cmd_twist(char (*inputTokens)[64]);


/// @brief Called when "pinch" token is registered; concerns the pinch joint
/// @param inputTokens Arguments to parse
void string_cmd_pinch(char (*inputTokens)[64]);


/// @brief Called when "can" token is registered; concerns the CAN bus
/// @param inputTokens Arguments to parse
void string_cmd_can(char (*inputTokens)[64]);


/// @brief Called when "S" token is registered; soft emergency stop for all joints
/// @param inputTokens Arguments to parse (none)
void string_cmd_stop(char (*inputTokens)[64]);


/// @brief Called when "acc1" token is registered; concerns the shoulder accelerometer
/// @param inputTokens Arguments to parse
void string_cmd_acc1(char (*inputTokens)[64]);


/// @brief Called when "relay" token is registered; concerns the motor driver enable relays
/// @param inputTokens Arguments to parse
void string_cmd_rly(char (*inputTokens)[64]);


/// @brief Called when "home" token is registered; starts the home zero calibration
/// @param inputTokens Arguments to parse
void string_cmd_home(char (*inputTokens)[64]);




/// @brief Pairs a command string token with a function pointer
typedef struct 
{
  /// @brief String token
  char* cmdString;

  /// @brief Corresponding handler function
  void (*cmdFuncPointer)();
} string_cmd_pair;


/// @brief Pairing of string commands and handler functions
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
  {"acc1", string_cmd_acc1},
  {"relay", string_cmd_rly},
  {"home", string_cmd_home}
  };

#endif //STRING_CMD_PARSER_H