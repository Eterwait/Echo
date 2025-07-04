#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "gpio.h"

/*
* GLOBAL VARIABLES
*/
extern uint16_t rxBufferCounter;
extern uint32_t calculated_CRC_32;
extern uint32_t recieved_CRC_32;
extern uint32_t adc;
extern uint16_t added_test_value;

/*
* ARRAYS
*/
extern uint8_t txBuffer[1000];
extern uint8_t rxBuffer[1000];
extern uint16_t adcBuffer[4];
extern uint8_t adcForTransmitt[2];

#endif /* INC_GLOBALS_H_ */
