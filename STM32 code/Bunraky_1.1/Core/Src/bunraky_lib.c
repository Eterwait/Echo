#include "bunraky_lib.h"
#include "globals.h"
#include "stdio.h"
#include "string.h"
#include <math.h>
#include "gpio.h"
#include "crc.h"
#include "adc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

void transmitData(USART_TypeDef *USARTx, uint8_t *data, int length)
{
	LL_GPIO_SetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Tx mode
	for (int i = 0; i < length; i++)
	{
		while (!LL_USART_IsActiveFlag_TXE(USARTx));
		LL_USART_TransmitData8(USARTx, data[i]);
	}
	while (!LL_USART_IsActiveFlag_TC(USARTx))
		;
	LL_USART_ClearFlag_TC(USARTx);
	LL_GPIO_ResetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Rx mode
}

uint8_t readForseSensor(void)
{
	memset(force_sensor_4_value, 0, sizeof(force_sensor_4_value)); // Clear force sensors buffer
	memcpy(txBuffer, (uint8_t*)(&command_to_read_force), sizeof(command_to_read_force)); //*
	transmitData(USART6, (uint8_t*)txBuffer, (sizeof(command_to_read_force)));
	LL_mDelay(2);

	for(uint8_t i=0; i<4; i++)
		memcpy(&force_sensor_4_value[i], rxBuffer + (i * sizeof(uint16_t)), sizeof(uint16_t));
	memcpy(&recieved_CRC_32, rxBuffer + (4 * sizeof(uint16_t)), sizeof(uint32_t));

	for (uint8_t i = 0; i < 4; i++)
		CRC_uint32_calc_buffer[i] = (uint32_t)force_sensor_4_value[i];
	calculated_CRC_32 = HAL_CRC_Calculate(&hcrc, &CRC_uint32_calc_buffer, 4);
	if (recieved_CRC_32 == calculated_CRC_32)
		return 1;
	else
		return 0;

}

void clearUsbBuffers(void)
{
	 memset(USB_Tx_buffer, 0, sizeof(USB_Tx_buffer));  // Clear USB Tx buffer
	 memset(USB_Rx_buffer, 0, sizeof(USB_Rx_buffer)); // Clear USB Rx buffer
}

void clearBufferAndCounter(void)
	{
		memset(rxBuffer, 0, sizeof(rxBuffer));
		rxBufferCounter = 0;
		memset(txBuffer, 0, sizeof(txBuffer));
		recieved_CRC_32 = 0;
		memset(CRC_uint32_calc_buffer, 0, sizeof(CRC_uint32_calc_buffer));
	}

void set_new_force(void)
	{
		float pwm_calculations_right_hand = 0;
		float pwm_calculations_left_hand = 0;

		if (right_hand_activation==1)
		{
		    if (force_sensor_4_value[0] > 4096)
		    	force_sensor_4_value[0] = 4096;

		    if (force_sensor_4_value[0] < 60)
		    	force_sensor_4_value[0] = 0;

		    pwm_calculations_right_hand = force_sensor_4_value[0];
		    pwm_calculations_right_hand = (uint16_t)roundf((pwm_calculations_right_hand * 350.0f) / 4096.0f); // Scale value from 0-4096 to 0-350
		}

		if (left_hand_activation==1)
		{
		    if (force_sensor_4_value[3] > 4096)
		    	force_sensor_4_value[3] = 4096;

		    if (force_sensor_4_value[3] < 60)
		    	force_sensor_4_value[3] = 0;

		    pwm_calculations_left_hand = force_sensor_4_value[3];
		    pwm_calculations_left_hand = (uint16_t)roundf((pwm_calculations_left_hand * 350.0f) / 4096.0f); // Scale value from 0-4096 to 0-350
		}


	    LL_TIM_OC_SetCompareCH1(TIM2, pwm_calculations_right_hand); // for right arm
	    LL_TIM_OC_SetCompareCH1(TIM3, pwm_calculations_left_hand); // for left arm
	}

void stop_motor(void)
{
	LL_TIM_OC_SetCompareCH1(TIM2, 0);
	LL_TIM_OC_SetCompareCH2(TIM3,0);
}

void readADCchannels(void)
{
	memset(adcBuffer, 0, sizeof(adcBuffer));

    for (uint8_t i = 0; i < 16; i++)
		{
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adcBuffer[i] = HAL_ADC_GetValue(&hadc1);
		}

    HAL_ADC_Stop(&hadc1);
}

void is_min_max_calibrated(void)
{
	if(left_hand_activation==0)
	{
		for(uint8_t i=0; i<8; i++)
			adcBuffer[i] = 0;
	}

	if(right_hand_activation==0)
	{
		for(uint8_t i=8; i<16; i++)
			adcBuffer[i] = 0;
	}
}

void calibration_ADC_values(void)
{
	for(uint8_t i=0; i<16; i++)
	{
		if (adcBuffer[i] < CCW_limits[i])
			adcBuffer[i] = CCW_limits[i];

		if (adcBuffer[i] > CW_limits[i])
			adcBuffer[i] = CW_limits[i];

		adcBuffer[i] = adcBuffer[i] - CCW_limits[i];

		if((i!=7) && (i!=15))
			adcBuffer[i] = adcBuffer[i] - zero_points[i];
	}

	if(left_hand_activation==1)
	{
		adcBuffer[0] *= 1;
		adcBuffer[1] *= 1;
		adcBuffer[2] *= 1;
		adcBuffer[3] *= 1;
		adcBuffer[4] *= 1;
		adcBuffer[5] *= 1;
		adcBuffer[6] *= 1;
		// Left gripper value should be: 0 - 255
		adcBuffer[7] = (int16_t)roundf((adcBuffer[7] * 255) / (CW_limits[7] - CCW_limits[7]));
		// Invert value (because of construction constraints)
//		adcBuffer[7] = 255 - adcBuffer[7];
	}

	if(right_hand_activation==1)
	{
		adcBuffer[8]  *= 1;
		adcBuffer[9]  *= 1;
		adcBuffer[10] *= 1;
		adcBuffer[11] *= 1;
		adcBuffer[12] *= 1;
		adcBuffer[13] *= 1;
		adcBuffer[14] *= 1;
		// Right gripper value should be: 0 - 255
		adcBuffer[15] = (int16_t)roundf((adcBuffer[15] * 255) / (CW_limits[15] - CCW_limits[15]));
		// Invert value (because of construction constraints)
//		adcBuffer[15] = 255 - adcBuffer[15];
	}
}


uint32_t calibration_function(uint8_t DoF_number, uint8_t min_max)
{
	 uint8_t count_write_flash = sizeof(CCW_limits_uint32_flash) / sizeof(CCW_limits_uint32_flash[0]);
	 readADCchannels();
	 load_calibration_values();
	 if (DoF_number>16)
		 return 1;
	 else if(min_max==0) // CCW position calibration
	 {
		 CCW_limits_uint32_flash[DoF_number] = (uint32_t) adcBuffer[DoF_number];
		 return flash_write_words(FLASH_SECTOR_6, ADDR_FLASH_SECTOR_6, CCW_limits_uint32_flash, count_write_flash); // if error == 0 == Recording was successful;
	 }
	 else if (min_max==1) // CW position calibration
	 {
		 CW_limits_uint32_flash[DoF_number] = (uint32_t) adcBuffer[DoF_number];
		 return flash_write_words(FLASH_SECTOR_7, ADDR_FLASH_SECTOR_7, CW_limits_uint32_flash, count_write_flash); // if error == 0 == Recording was successful;
	 }
}


void process_calibration(uint8_t usb_cmd, uint8_t DoF)
{
/*
* @brief Processes calibration on received command.
* @param usb_cmd Command from USB (expects '0' for MIN or '1' for MAX).
* @param DoF DOF number used in calibration_function.
*/
	uint32_t error = 0;

    if (DoF < 1 || DoF > 16)
        return;

	// Convert 1-8 to 0-15 for array work
	DoF -= 1;

    if (usb_cmd == '0') // Call calibration function for CCW (second parameter = 0)
        error = calibration_function(DoF, 0);

    else if (usb_cmd == '1') // Call calibration function for CW (second parameter = 1)
        error = calibration_function(DoF, 1);

    // Copy the error to USB_Tx_buffer to send via USB
    memcpy(USB_Tx_buffer, (uint8_t*)(&error), sizeof(error));

    // Wait until USB transfer is complete
    while (CDC_Transmit_FS(USB_Tx_buffer, sizeof(error)) == USBD_BUSY)
        ;

    // Clear buffers and counters
    clearBufferAndCounter();
    clearUsbBuffers();
    depug_piip();
}


uint32_t wrote_zero_to_dof(uint8_t DoF_number)
{
	 uint32_t error_1 = 0;
	 uint32_t error_2 = 0;
	 uint8_t count_write_flash = sizeof(CCW_limits_uint32_flash) / sizeof(CCW_limits_uint32_flash[0]);

	 load_calibration_values();

	if (DoF_number < 1 || DoF_number > 16)
		return;

	// Convert 1-8 to 0-15 for array work
	 DoF_number--;

	 CCW_limits_uint32_flash[DoF_number] = 0;
	 error_1 = flash_write_words(FLASH_SECTOR_6, ADDR_FLASH_SECTOR_6, CCW_limits_uint32_flash, count_write_flash); // if error == 0 == Recording was successful;

	 CW_limits_uint32_flash[DoF_number] = 0;
	 error_2 = flash_write_words(FLASH_SECTOR_7, ADDR_FLASH_SECTOR_7, CW_limits_uint32_flash, count_write_flash); // if error == 0 == Recording was successful;

	 if((error_1!=0)||(error_2!=0))
		 return 1;
	 else
		 return 0;
}


uint8_t init_load_calibration_values(void)
{
	uint8_t error_values_flag = 0;
	// Load calibration values from flash
	uint8_t count_read_flash = sizeof(CCW_limits_uint32_flash) / sizeof(CCW_limits_uint32_flash[0]);
	flash_read_words(ADDR_FLASH_SECTOR_6, CCW_limits_uint32_flash, count_read_flash);
	// 32 - Number of bytes for the first 8 uint32_t numbers for an array CCW_limits_uint32_flash
	// Its needed to read next portion of flash memory, where locate CW_limits_uint32_flash
	flash_read_words(ADDR_FLASH_SECTOR_7, CW_limits_uint32_flash, count_read_flash);

	// If left hand have not calibrated, nothing will works
	for(uint8_t i=0; i<8; i++)
	{
		if (CCW_limits_uint32_flash[i]>4096)
			left_hand_activation = 0;
		if (CW_limits_uint32_flash[i]>4096)
			left_hand_activation = 0;
	}

	// If right hand have not calibrated, nothing will works
	for(uint8_t i=8; i<16; i++)
	{
		if (CCW_limits_uint32_flash[i]>4096)
			right_hand_activation = 0;
		if (CW_limits_uint32_flash[i]>4096)
			right_hand_activation = 0;
	}


	// Explicitly convert to the required type
	if (left_hand_activation==1)
	{
		for(uint8_t i=0; i<8; i++)
		{
			CW_limits[i] = (uint16_t) CW_limits_uint32_flash[i];
			CCW_limits[i] = (uint16_t) CCW_limits_uint32_flash[i];
		}
	}

	// Explicitly convert to the required type
	if (right_hand_activation==1)
	{
		for(uint8_t i=8; i<16; i++)
		{
			CW_limits[i] = (uint16_t) CW_limits_uint32_flash[i];
			CCW_limits[i] = (uint16_t) CCW_limits_uint32_flash[i];
		}
	}

	for(uint8_t i=0; i<16; i++)
	{
		if(i==2)
			zero_points[i] = (uint16_t)roundf((CW_limits[i] - CCW_limits[i]) * (40.0f / 260.0f)); // angles of 3 DOF joint
		else if(i==10)
			zero_points[i] = (uint16_t)roundf((CW_limits[i] - CCW_limits[i]) * (220.0f / 260.0f));// angles of 3 DOF joint
		else
			zero_points[i] = (uint16_t)roundf((CW_limits[i] - CCW_limits[i]) / 2.0f);

	}


	if (error_values_flag == 0)
		return 0;
	else
		return 1;
}

void load_calibration_values(void)
{
	// Load calibration values from flash
	uint8_t count_read_flash = sizeof(CCW_limits_uint32_flash) / sizeof(CCW_limits_uint32_flash[0]);
	flash_read_words(ADDR_FLASH_SECTOR_6, CCW_limits_uint32_flash, count_read_flash);
	// 32 - Number of bytes for the first 8 uint32_t numbers for an array CCW_limits_uint32_flash
	// Its needed to read next portion of flash memory, where locate CW_limits_uint32_flash
	flash_read_words(ADDR_FLASH_SECTOR_7, CW_limits_uint32_flash, count_read_flash);


	for(uint8_t i = 0; i<count_read_flash; i++)
	{
		CCW_limits[i] = (uint16_t) CCW_limits_uint32_flash[i];
		CW_limits[i] = (uint16_t) CW_limits_uint32_flash[i];
	}

}

void information_leds_activation(void)
{
	if (sense_flag==0)
	{
		// Sense Lvl 1 LED activation
		LL_GPIO_SetOutputPin(Sense_Lvl_1_GPIO_Port, Sense_Lvl_1_Pin);
		LL_GPIO_ResetOutputPin(Sense_Lvl_2_GPIO_Port, Sense_Lvl_2_Pin);
		LL_GPIO_ResetOutputPin(Sense_Lvl_3_GPIO_Port, Sense_Lvl_3_Pin);
	}
	else if (sense_flag==1)
	{
		// Sense Lvl 2 LED activation
		LL_GPIO_ResetOutputPin(Sense_Lvl_1_GPIO_Port, Sense_Lvl_1_Pin);
		LL_GPIO_SetOutputPin(Sense_Lvl_2_GPIO_Port, Sense_Lvl_2_Pin);
		LL_GPIO_ResetOutputPin(Sense_Lvl_3_GPIO_Port, Sense_Lvl_3_Pin);
	}
	else
	{
		// Sense Lvl 3 LED activation
		LL_GPIO_ResetOutputPin(Sense_Lvl_1_GPIO_Port, Sense_Lvl_1_Pin);
		LL_GPIO_ResetOutputPin(Sense_Lvl_2_GPIO_Port, Sense_Lvl_2_Pin);
		LL_GPIO_SetOutputPin(Sense_Lvl_3_GPIO_Port, Sense_Lvl_3_Pin);
	}

	// Data collection LED activation
	if(start_dataset_collection_flag==0)
		LL_GPIO_ResetOutputPin(start_dataset_collection_led_GPIO_Port, start_dataset_collection_led_Pin);
	else
		LL_GPIO_SetOutputPin(start_dataset_collection_led_GPIO_Port, start_dataset_collection_led_Pin);
}


void erase_flash(uint32_t sector)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;
	// Settings of clearing flash
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;  // Clear sector
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = sector; // Point to sector
	EraseInitStruct.NbSectors = 1;  // Number of sector

	HAL_FLASH_Unlock();

	// Clearing
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		HAL_FLASH_Lock();
	}

	HAL_FLASH_Lock();
}

void flash_read_words(uint32_t address, uint32_t *dest, uint8_t count)
{
	/*
		* @brief Reads an array of 32-bit integers from flash (starting at MYADDR).
		* @param address: starting address to read.
		* @param dest: pointer to the array where the read data will be written.
		* @param count: number of 32-bit words to read.
		* @retval No return value.
		* @note Reading from flash is done directly, since it is mapped into the address space.
	*/
    for (uint8_t i = 0; i < count; i++)
    {
        dest[i] = *(volatile uint32_t *)address;  // Or you can use __IO uint32_t * (analogous to volatile uint32_t *)
        address += 4;  // Move to the next 32-bit word
    }
}

uint32_t flash_write_words(uint32_t sector, uint32_t address, uint32_t *idata, uint8_t count)
{
/*
	* @brief Writes an array of 32-bit integers to flash (sector 6).
	* @param address: starting address of the write (must be the start of sector 6 or 7).
	* @param idata: pointer to an array of 32-bit data.
	* @param count: number of 32-bit elements in the array.
	* @retval 0 if the write was successful, or the HAL_FLASH_GetError() error code on error.
	* Tip: ise "uint8_t count = sizeof(my_array) / sizeof(my_array[0]);" for count calculation.
*/
    uint32_t error;
    uint32_t PageError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct;

    // Determine if sector correct, only 6 or 7 allowed
    if ((sector == FLASH_SECTOR_6) || (sector == FLASH_SECTOR_7))
    {
        HAL_FLASH_Unlock();

    	// Settings of clearing flash
    	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;  // Clear sector
    	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    	EraseInitStruct.Sector = sector; // Point to sector
    	EraseInitStruct.NbSectors = 1;  // Number of sector

        if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
        {
            error = HAL_FLASH_GetError();
            HAL_FLASH_Lock();
            return error;
        }

        uint32_t temp_debug_value = 0;
        // Write data: uint32_t
        for (uint8_t i = 0; i < count; i++)
        {
        	temp_debug_value = idata[i];
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, idata[i]) != HAL_OK)
            {
                error = HAL_FLASH_GetError();
                HAL_FLASH_Lock();
                return error;
            }
            address += 4;  // Move to the next 32-bit word
        }

        HAL_FLASH_Lock();
        return 0;  // Recording was successful
    }
    else
        return 1;  // Error: invalid address
}

void depug_piip(void)
{
	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(35);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
}

void start_melody(void)
{
//	LL_mDelay(1500);
	uint8_t delay_ms = 16;

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_mDelay(70);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_GPIO_SetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);
	LL_GPIO_ResetOutputPin(Buzzer_GPIO_Port, Buzzer_Pin);
	LL_mDelay(delay_ms);

	LL_mDelay(200);
}
