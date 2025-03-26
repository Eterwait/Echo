#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_
#include <stdint.h>

/*
* FLASH
*/
#define ADDR_FLASH_SECTOR_6 0x08040000  // Start of sector 6
//  FLASH_SECTOR_6
#define ADDR_FLASH_SECTOR_7 0x08060000  // Start of sector 7
//  FLASH_SECTOR_7

/*
* GLOBAL VARIABLES
*/

extern uint16_t rxBufferCounter;
extern uint32_t calculated_CRC_32;
extern uint32_t recieved_CRC_32;
extern uint16_t pwm;
extern uint8_t flagUart;
extern uint8_t force_sensor_adress;
extern uint8_t command_to_read_force;
extern uint8_t stop_force_control_flag;
extern uint8_t left_hand_activation;
extern uint8_t right_hand_activation;

extern uint8_t start_dataset_collection_flag;
extern uint8_t sense_flag;

/*
* ADC CALIBRATION VARIABLES
*/

extern uint32_t ADC_calibration_buffer_uint32_flash[16];
extern uint16_t zero_points[16];
extern uint16_t CW_limits[16];
extern uint16_t CCW_limits[16];
extern uint32_t CW_limits_uint32_flash[16];
extern uint32_t CCW_limits_uint32_flash[16];

// Current PWM BORDERS
extern uint16_t PWM_MIN;
extern uint16_t PWM_MAX;

/*
* ARRAYS
*/
extern uint32_t CRC_uint32_calc_buffer[4];
extern int16_t adcBuffer[16];
extern uint16_t force_sensor_4_value[4];
extern uint8_t USB_Tx_buffer[100];
extern uint8_t test_usb_buffer[10];
extern uint8_t txBuffer[1000];
extern uint8_t rxBuffer[1000];
extern uint8_t dataForTransmitt[1000];

#endif /* INC_GLOBALS_H_ */
