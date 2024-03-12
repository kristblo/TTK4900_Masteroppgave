//Global defines
#define MTR1 TIM15
#define MTR2 TIM1
#define ENC1 TIM3
#define ENC2 TIM8

#define CTR_PRD 7200
#define GLOBAL_DEBUG 1



//Variables specific to each of the three control units
#define TORSO 0
#define SHOULDER 1
#define HAND 2
#define ACTIVE_UNIT HAND //Change before flashing another unit

#if ACTIVE_UNIT == TORSO
  #define CAN_TXID 0x10A //CAN transmit ID
  #define CAN_FILTER_IDH 0x0 //CAN message filtering
  #define MTR_POL -1 //Motor polarity, i.e. whether "forward" is CW or CCW
  #define UART_INPUT 1 //UART input parsing


#elif ACTIVE_UNIT == SHOULDER
  #define CAN_TXID 0x10B //CAN transmit ID
  #define CAN_FILTER_IDH 0x10A //CAN message filtering
  #define MTR_POL 1 //Motor polarity, i.e. whether "forward" is CW or CCW
  #define UART_INPUT 1 //UART input parsing



#elif ACTIVE_UNIT == HAND
  #define CAN_TXID 0x10C //CAN transmit ID
  #define CAN_FILTER_IDH 0x10A //CAN message filtering
  #define MTR_POL 1 //Motor polarity, i.e. whether "forward" is CW or CCW
  #define UART_INPUT 1 //UART input parsing
#else
  #error NO VALID CONTROL UNIT SELECTED
#endif

//Peripheral activation
void activate_peripherals();