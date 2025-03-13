#ifndef INC_BUNRAKY_LIB_H_
#define INC_BUNRAKY_LIB_H_

#include "bunraky_lib.h"
#include "usart.h"

void transmitData(USART_TypeDef *USARTx, uint8_t *data, int length);
uint8_t readForseSensor(void);
void clearUsbBuffers(void);
void clearBufferAndCounter(void);
void readADCchannels(void);
void calibration_ADC_values(void);
void is_min_max_calibrated(void);
uint32_t calibration_function(uint8_t DoF_number, uint8_t min_max);
void process_calibration(uint8_t usb_cmd, uint8_t DoF);
uint32_t wrote_zero_to_dof(uint8_t DoF_number);
void load_calibration_values(void);
void set_new_force(void);
void stop_motor(void);
uint8_t init_load_calibration_values(void);
void information_leds_activation(void);
void erase_flash(uint32_t sector);
void flash_read_words(uint32_t address, uint32_t *dest, uint8_t count);
uint32_t flash_write_words(uint32_t sector, uint32_t address, uint32_t *idata, uint8_t count);
void depug_piip(void);
void start_melody(void);
#endif /* INC_BUNRAKY_LIB_H_ */
