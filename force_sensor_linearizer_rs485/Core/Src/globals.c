#include "globals.h"

/*
* GLOBAL VARIABLES
*/
uint16_t rxBufferCounter = 0;
uint32_t calculated_CRC_32 = 0;
uint32_t recieved_CRC_32 = 0;
uint32_t adc = 0;
uint16_t added_test_value = 666;
/*
* ARRAYS
*/
uint8_t txBuffer[1000] = {0};
uint8_t rxBuffer[1000] = {0};
uint16_t adcBuffer[4] = {0};
uint8_t adcForTransmitt[2] = {0};
