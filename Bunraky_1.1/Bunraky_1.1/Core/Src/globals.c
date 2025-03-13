#include "globals.h"
#include <stdint.h>
/*
* GLOBAL VARIABLES
*/

uint16_t rxBufferCounter = 0;
uint32_t calculated_CRC_32 = 0;
uint32_t recieved_CRC_32 = 0;
uint16_t pwm = 0;
uint8_t flagUart = 0;
uint8_t stop_force_control_flag = 0;
uint8_t force_sensor_adress = 10;
uint8_t command_to_read_force = 'r';
uint8_t left_hand_activation = 1;
uint8_t right_hand_activation = 1;
uint8_t start_dataset_collection_flag = 0;
uint8_t sense_flag = 0;


/*
* ADC CALIBRATION VARIABLES
*/

/*
* All from potentiometer view
*
* DOF1 left position 0
* DOF1 right position 0
*
* DOF2 left position 0
* DOF2 right position 0
*
* DOF3 left position 0
* DOF3 right position 0
*
* DOF4 left position 0
* DOF4 right position 0
*
* DOF5 left position 0
* DOF5 right position 0
*
* DOF6 left position 0
* DOF6 right position 0
*
* DOF7 open position 0
* DOF7 close position 0
*
*/
uint32_t ADC_calibration_buffer_uint32_flash[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint16_t zero_points[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// CW = clock wise
// CCW = counter clock wise
uint16_t CW_limits[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t CCW_limits[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t CW_limits_uint32_flash[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t CCW_limits_uint32_flash[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Current PWM BORDERS
uint16_t PWM_MIN = 0;
uint16_t PWM_MAX = 400;

/*
* ARRAYS
*/
uint32_t CRC_uint32_calc_buffer[4] = {0};
int16_t adcBuffer[16] = {0};
uint16_t force_sensor_4_value[4] = {0};
uint8_t USB_Tx_buffer[100] = {0};
uint8_t test_usb_buffer[10] = {6};
uint8_t txBuffer[1000] = {0};
uint8_t rxBuffer[1000] = {0};
uint8_t dataForTransmitt[1000] = {0};
