#ifndef INC_FORCE_SENSOR_LINERALAZER_LIB_H_
#define INC_FORCE_SENSOR_LINERALAZER_LIB_H_

#include "force_sensor_lineralazer_lib.h"
#include "usart.h"

void transmitData(USART_TypeDef *USARTx, uint8_t *data, int length);
void clearBufferAndCounter(void);
void readADCchannels(void);

#endif /* INC_FORCE_SENSOR_LINERALAZER_LIB_H_ */
